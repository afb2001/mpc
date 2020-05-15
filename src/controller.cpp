#include <thread>
#include <queue>
#include <future>
#include "controller.h"
#include <cassert>
#include <cfloat>

using namespace std;

Controller::Controller(ControlReceiver *controlReceiver) {
    m_ControlReceiver = controlReceiver;
    // load it up with an empty future
    m_LastMpc = std::async(std::launch::async, []{});
}

Controller::~Controller() = default;

double Controller::getTime()
{
    struct timespec t{};
    clock_gettime(CLOCK_REALTIME, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}

VehicleState Controller::mpc(double& r, double& t, State startCopy, const DubinsPlan& referenceTrajectory, double endTime, long trajectoryNumber)
{
    std::pair<double, double> currentEstimate = m_CurrentEstimator.getCurrent(startCopy);

//    std::cerr << "Current estimated to be " << currentEstimate.first << ", " << currentEstimate.second << std::endl;

    assert(!referenceTrajectory.empty()); // make sure reference trajectory is long enough

    // intentional use of integer division (m_Rudders / 2)
    const double rudderGranularity = 1.0 / (m_Rudders / 2), throttleGranularity = 1.0 / (m_Throttles - 1);
    const double minRudder = -1, midRudder = 0, maxRudder = 1;
    const double minThrottle = 0, maxThrottle = 1;

    struct MpcState {
    public:
        VehicleState state;
        double LastRudder{}, LastThrottle{};
        double Score{};
        MpcState(): state(State()){};
    };

    std::vector<MpcState> overallBestTrajectory; // has to be dynamic because we don't know how many iterations we'll get
    double initialRudders[6] = {minRudder, midRudder, maxRudder, 0, 0, 0}; // min, mid, max
    const int nInitialRudders = 3;
    double initialThrottles[5] = {minThrottle, maxThrottle, 0, 0, 0}; // min, max
    const int nInitialThrottles = 2;

    int iterations = 0;
    auto trajectoryStartTime = referenceTrajectory.get().front().getStartTime();
    // intentional use of integer cast here, despite my IDE complaining
    const int maxIterations = (trajectoryStartTime - startCopy.time() + referenceTrajectory.totalTime()) / c_ScoringTimeStep - 1;
    // set up storage (no heap memory)
    // each "stack frame" in DFS is represented by a slot in these arrays, so I can skip overhead of recursion
    State interpolated[maxIterations];
    MpcState currentTrajectory[maxIterations + 1], bestTrajectory[maxIterations + 1]; // currentTrajectory holds start too
    currentTrajectory[0].state = startCopy;
    currentTrajectory[0].LastRudder = m_LastRudder;
    currentTrajectory[0].LastThrottle = m_LastThrottle;
    currentTrajectory[0].Score = 0;
    double ruddersArray[maxIterations][6], throttlesArray[maxIterations][5];
    int nRuddersArray[maxIterations], nThrottlesArray[maxIterations];
    int rudderIndices[maxIterations], throttleIndices[maxIterations];

    while (iterations < maxIterations) { // don't even bother to check the time here
        auto minScore = DBL_MAX;
        // cut out when the reference trajectory is updated
        if (!validTrajectoryNumber(trajectoryNumber)) {
//            cerr << "Trajectory number " << trajectoryNumber << " not valid." << endl;
            return State(); // skip cleanup because we're supposed to terminate the thread
        }

        // initialize all the slots
        std::copy(initialRudders, initialRudders + 6, ruddersArray[iterations]);
        std::copy(initialThrottles, initialThrottles + 5, throttlesArray[iterations]);
        nRuddersArray[iterations] = nInitialRudders;
        nThrottlesArray[iterations] = nInitialThrottles;
        rudderIndices[iterations] = 0;
        throttleIndices[iterations] = 0; // not necessary as default for int is zero but I want to be sure
        interpolated[iterations].time() = startCopy.time() + ((iterations + 1) * c_ScoringTimeStep);
        referenceTrajectory.sample(interpolated[iterations]);
//        interpolated[iterations] = interpolateTo(startCopy.time() + ((iterations + 1) * c_ScoringTimeStep), referenceTrajectoryCopy);
        int index = 0;
        while (index >= 0) {
            if (m_ControlReceiver->getTime() >= endTime) {
                goto mpcCleanup; // skip assigning best trajectory because we didn't finish the iteration
            }
            auto& current = currentTrajectory[index];
            auto& rudderIndex = rudderIndices[index];
            auto& throttleIndex = throttleIndices[index];
            auto& rudders = ruddersArray[index];
            auto& throttles = throttlesArray[index];
            auto& nRudders = nRuddersArray[index];
            auto& nThrottles = nThrottlesArray[index];
            if (rudderIndex == nRudders && throttleIndex == nThrottles) {
                // climb down a frame
                rudderIndex = 0; throttleIndex = 0;
                index--;
                continue;
            }
            if (rudderIndex == 0 && throttleIndex == 0) {
                // this is the first time through with this configuration, so set up rudders and throttles
                nRudders = 3;
                nThrottles = 2; // min, (mid), max
                const auto lowRudder = current.LastRudder - rudderGranularity;
                const auto highRudder = current.LastRudder + rudderGranularity;
                if (minRudder + c_Tolerance < lowRudder && fabs(midRudder - lowRudder) > c_Tolerance)
                    rudders[nRudders++] = lowRudder;
                if (fabs(midRudder - highRudder) > c_Tolerance && maxRudder > highRudder + c_Tolerance)
                    rudders[nRudders++] = highRudder;
                if (fabs(minRudder - current.LastRudder) > c_Tolerance &&
                    fabs(midRudder - current.LastRudder) > c_Tolerance &&
                    fabs(maxRudder - current.LastRudder) > c_Tolerance)
                    rudders[nRudders++] = current.LastRudder;
                const auto lowThrottle = current.LastThrottle - throttleGranularity;
                const auto highThrottle = current.LastThrottle + throttleGranularity;
                if (minThrottle + c_Tolerance < lowThrottle)
                    throttles[nThrottles++] = lowThrottle;
                if (maxThrottle > highThrottle + c_Tolerance)
                    throttles[nThrottles++] = highThrottle;
                if (fabs(minThrottle - current.LastThrottle) > c_Tolerance &&
                    fabs(maxThrottle - current.LastThrottle) > c_Tolerance)
                    throttles[nThrottles++] = current.LastThrottle;
            }
            auto& next = currentTrajectory[index + 1];
            // simulate next state with next controls
            next.state = current.state.simulate(
                    rudders[rudderIndex], throttles[throttleIndex], c_ScoringTimeStep, currentEstimate);
            // store the rudders
            next.LastRudder = rudders[rudderIndex];
            next.LastThrottle = throttles[throttleIndex];
            // calculate and update the score
            auto score = compareStates(interpolated[index], next.state);
            next.Score = current.Score + score;
            // handle if we're at the end of the line
            if (index == iterations) {
                if (score < minScore) {
                    std::copy(currentTrajectory, currentTrajectory + iterations + 2, bestTrajectory);
                    minScore = score;
                }
            } else {
                // climb up a frame
                index++;
            }

            // set up for the next time we'll use this frame
            rudderIndex++;
            if (rudderIndex == nRudders) {
                throttleIndex++;
                if (throttleIndex != nThrottles) {
                    rudderIndex = 0;
                }
            }
        }
        // copy best trajectory into overarching best trajectory
        iterations++; // increase depth
        overallBestTrajectory = std::vector<MpcState>(iterations + 1); // set capacity
        std::copy(bestTrajectory, bestTrajectory + iterations + 1, overallBestTrajectory.begin());
    }

mpcCleanup: // shush I'm using it sparingly and appropriately

//    std::cerr << "Managed " << iterations << " iterations of limited-branching MPC (" << c_ScoringTimeStep * iterations
//        << " seconds in the future)" << std::endl;

    assert(overallBestTrajectory.size() > 1);
    r = overallBestTrajectory[1].LastRudder; t = overallBestTrajectory[1].LastThrottle;
    std::vector<State> forDisplay;
    forDisplay.reserve(overallBestTrajectory.size());
for (const auto& s : overallBestTrajectory) forDisplay.emplace_back(s.state.state);
    m_ControlReceiver->displayTrajectory(forDisplay, false, m_Achievable);
    assert(std::isfinite(r) && std::isfinite(t));
    static_assert(c_ScoringTimeStep == 1.0, "Assume scoring time-step is equal to the planner's planning time");
    return overallBestTrajectory[1].state; // based on c_ScoringTimeStep
}

void Controller::terminate()
{
    std::unique_lock<mutex> lock(m_TrajectoryNumberMutex);
    m_TrajectoryNumber = -1;
    m_CurrentEstimator.resetCurrentEstimate(); // we don't know how long we'll be down so the current may have changed
}

void Controller::updatePosition(State state) {
    m_CurrentLocationMutex.lock();
    m_CurrentLocation = state;
    m_CurrentLocationMutex.unlock();
}

void Controller::updateConfig(int rudders, int throttles, double distanceWeight, double headingWeight,
                              double speedWeight,
                              double achievableThreshold, bool currentEstimation) {
    m_Rudders = rudders; m_Throttles = throttles;
    m_DistanceWeight = distanceWeight; m_HeadingWeight = headingWeight; m_SpeedWeight = speedWeight;
    m_AchievableScoreThreshold = achievableThreshold;
    if (currentEstimation) m_CurrentEstimator.enable(); else m_CurrentEstimator.disable();
}

double Controller::compareStates(const State& s1, const VehicleState& s2) const {
    // ignore differences in time
    auto headingDiff = fabs(s1.headingDifference(s2.courseMadeGood));
    auto speedDiff = fabs(s1.speed() - s2.state.speed());
    auto dx = s1.x() - s2.state.x();
    auto dy = s1.y() - s2.state.y();
    auto d = sqrt(dx * dx + dy * dy);
    return m_DistanceWeight * d + m_HeadingWeight * headingDiff + m_SpeedWeight * speedDiff;
}

bool Controller::validTrajectoryNumber(long trajectoryNumber) {
    std::unique_lock<mutex> lock1(m_TrajectoryNumberMutex);
    return trajectoryNumber == m_TrajectoryNumber;
}

State Controller::updateReferenceTrajectory(const DubinsPlan& referenceTrajectory, long trajectoryNumber) {
    if (referenceTrajectory.empty()){
        // Not long enough for MPC. Balk out.
        // NOTE: previous MPC thread may still be running; that can time out on its own
        std::cerr << "Reference trajectory empty and cannot be used for MPC. Reference trajectory will not be updated." << std::endl;
        return State();
    }

    // set the trajectory number, which lets any already-running MPC thread(s) know to terminate
    setTrajectoryNumber(trajectoryNumber);

    // get the next start state
    auto start = getStateAfterCurrentControl();

    double r = 0, t = 0;
    // do first round of MPC so we can return a state based on the new reference trajectory
    auto result = mpc(r, t, start.state, referenceTrajectory, m_ControlReceiver->getTime() + c_PlanningTime, trajectoryNumber);

    sendControls(r, t);

    // join the last thread used for MPC
    // should be able to wait for no time at all but I'll give it some leeway for now
    auto status = m_LastMpc.wait_for(std::chrono::milliseconds((int)(1000 * c_PlanningTime)));
    switch (status) {
        case std::future_status::ready:
            // good
            m_LastMpc.get(); // not sure this is necessary
//            cerr << "Got last iteration's future" << endl;
            break;
        case std::future_status::timeout:
            // bad
            throw std::runtime_error("Old MPC thread refused to die");
            break;
        default:
            // shouldn't ever get here
            std::cerr << "Shouldn't ever get here" << std::endl;
            break;
    }
    // Kick off the new MPC thread
    // Copies reference trajectory so we shouldn't have to deal with issues of a reference to a local variable going out of scope
    // There are a bunch of different ways of doing this - I went with std::async so I could hold onto the future object
    // and potentially report failures. Occasionally get that runtime error up there and I don't know why yet.
    m_LastMpc = std::async(std::launch::async, [=]{ runMpc(referenceTrajectory, start.state, result.state, trajectoryNumber); });

    // determine if reference trajectory seems feasible ("close enough")
    State stateOnReferenceTrajectory;
    stateOnReferenceTrajectory.time() = result.state.time();
    referenceTrajectory.sample(stateOnReferenceTrajectory);
    auto score = compareStates(stateOnReferenceTrajectory, result);
    if (score <= m_AchievableScoreThreshold) {
        // Controller deems us close enough
        result = stateOnReferenceTrajectory;
        m_Achievable = true;
    } else {
        // if the score threshold is set to zero we always plan from controller's prediction, so don't report failure
        m_Achievable = m_AchievableScoreThreshold == 0;
        if (!m_Achievable)
            std::cerr << "Controller doesn't think we can make the reference trajectory (score = " << score << ")" << std::endl;
    }

    return result.state;
}

void Controller::setTrajectoryNumber(long trajectoryNumber) {
    unique_lock<mutex> lock(m_TrajectoryNumberMutex);
    // updating the trajectory number stops the previous MPC thread
    m_TrajectoryNumber = trajectoryNumber;
}

VehicleState Controller::getStateAfterCurrentControl() {
    State startCopy;
    {
        std::unique_lock<mutex> lock(m_CurrentLocationMutex);
        startCopy = m_CurrentLocation;
    }
    // TODO! -- figure out concurrency issues here - last controls should be protected somehow
    VehicleState start(startCopy);
    auto current = m_CurrentEstimator.getCurrent(startCopy);
    // simulate last issued control to how long it will take us to compute the next one
    start.simulate(m_LastRudder, m_LastThrottle, c_PlanningTime, current);
    return start;
}

void Controller::sendControls(double r, double t) {
    // TODO! -- concurrency safety?
    m_LastRudder = r;
    m_LastThrottle = t;
    m_ControlReceiver->receiveControl(r, t);
    std::unique_lock<mutex> lock(m_CurrentLocationMutex);
    m_CurrentEstimator.updateEstimate(m_CurrentLocation, r, t);
}

void Controller::runMpc(DubinsPlan trajectory, State start, State result, long trajectoryNumber) {
    auto endTime = m_ControlReceiver->getTime() + c_ReferenceTrajectoryExpirationTime;
    double r = 0, t = 0;
    while (m_ControlReceiver->getTime() < endTime) {
        if (!validTrajectoryNumber(trajectoryNumber)) break;
        auto stateAfterCurrentControl = getStateAfterCurrentControl();
        if (!trajectory.containsTime(stateAfterCurrentControl.state.time() + 3 * c_ScoringTimeStep)) break; // why 3? Seems like a good number of iterations
        mpc(r, t, stateAfterCurrentControl.state, trajectory, m_ControlReceiver->getTime() + c_PlanningTime,
            trajectoryNumber);
        sendControls(r, t);
    }
    if (m_ControlReceiver->getTime() >= endTime) {
        std::cerr << "Controller's reference trajectory appears to have timed out. No more controls will be issued" << std::endl;
    }
//    cerr << "Ending an old thread running MPC" << endl;
}



