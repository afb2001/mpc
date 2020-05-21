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

Controller::MpcState Controller::mpc(State startState, const DubinsPlan& referenceTrajectory, double endTime,
                         long trajectoryNumber)
{
    std::pair<double, double> currentEstimate = m_CurrentEstimator.getCurrent(startState);

//    std::cerr << "Current estimated to be " << currentEstimate.first << ", " << currentEstimate.second << std::endl;

    if (referenceTrajectory.empty()) throw std::runtime_error("Cannot run MPC on an empty plan");

    // copy config for thread safety
    double throttleGranularity, rudderGranularity;
    {
        std::unique_lock<std::mutex> lock(m_ConfigMutex);
        rudderGranularity = m_RudderGranularity;
        throttleGranularity = m_ThrottleGranularity;
    }

    // has to be dynamic because we don't know how many iterations we'll get
    // contains not only states, but also controls and scores
    std::vector<MpcState> overallBestTrajectory;
    // At each level of search we'll have a slightly different set of controls. These arrays contain the extreme values
    // which will always be present (and the counts let us easily check how many there are).
    const int maxTotalRudders = 6, maxTotalThrottles = 5; // no magic numbers
    double initialRudders[maxTotalRudders] = {m_MinRudder, m_MidRudder, m_MaxRudder, 0, 0, 0}; // min, mid, max
    const int nInitialRudders = 3;
    double initialThrottles[maxTotalThrottles] = {m_MinThrottle, m_MaxThrottle, 0, 0, 0}; // min, max
    const int nInitialThrottles = 2;

    int iterations = 0;
    auto trajectoryStartTime = referenceTrajectory.get().front().getStartTime();
    // intentional use of integer cast here, despite my IDE complaining
    const int maxIterations = (trajectoryStartTime - startState.time() + referenceTrajectory.totalTime()) / c_ScoringTimeStep - 1;

    // set up storage (no heap memory)
    // each "stack frame" in DFS is represented by a slot in these arrays, so I can skip overhead of recursion
    // allocate space to cache states along reference trajectory at which we compute trajectory scores
    State cachedScoringCheckpoints[maxIterations];
    // allocate space for the trajectory in consideration and the best trajectory (at max length)
    MpcState currentTrajectory[maxIterations + 1], bestTrajectory[maxIterations + 1]; // currentTrajectory holds start too
    // set start state in trajectory in consideration
    currentTrajectory[0].state = startState;
    currentTrajectory[0].LastRudder = m_LastRudder;
    currentTrajectory[0].LastThrottle = m_LastThrottle;
    currentTrajectory[0].Score = 0;
    // These arrays hold which controls are in use at each depth (as well as counts for each depth for each control).
    // The indices keep track of which controls we've tried at each depth so far.
    double ruddersArray[maxIterations][maxTotalRudders], throttlesArray[maxIterations][maxTotalThrottles];
    int nRuddersArray[maxIterations], nThrottlesArray[maxIterations];
    int rudderIndices[maxIterations], throttleIndices[maxIterations];

    while (iterations < maxIterations) { // don't even bother to check the time here
        auto minScore = DBL_MAX;
        // cut out when the reference trajectory is updated
        if (!validTrajectoryNumber(trajectoryNumber)) {
//            cerr << "Trajectory number " << trajectoryNumber << " not valid." << endl;
            return {}; // skip cleanup because we're supposed to terminate the thread
        }

        // initialize all the slots
        std::copy(initialRudders, initialRudders + maxTotalRudders, ruddersArray[iterations]);
        std::copy(initialThrottles, initialThrottles + maxTotalThrottles, throttlesArray[iterations]);
        nRuddersArray[iterations] = nInitialRudders;
        nThrottlesArray[iterations] = nInitialThrottles;
        rudderIndices[iterations] = 0;
        throttleIndices[iterations] = 0; // not necessary as default for int is zero but I want to be sure
        // set time in scoring checkpoint so we can sample the reference trajectory
        cachedScoringCheckpoints[iterations].time() = startState.time() + ((iterations + 1) * c_ScoringTimeStep);
        // sample the reference trajectory at the scoring checkpoint
        referenceTrajectory.sample(cachedScoringCheckpoints[iterations]);
        int searchDepthIndex = 0;
        while (searchDepthIndex >= 0) {
            if (m_ControlReceiver->getTime() >= endTime) {
                goto mpcCleanup; // skip assigning best trajectory because we didn't finish the iteration
            }
            // This block grabs references to everything we're going to need at this depth. This isn't strictly
            // necessary, but it cleans things up a little (no repetitive indexing) and supports the illusion that we
            // have a call stack (this is like loading local variables in a stack frame).
            auto& currentMpcState = currentTrajectory[searchDepthIndex];
            auto& rudderIndex = rudderIndices[searchDepthIndex];
            auto& throttleIndex = throttleIndices[searchDepthIndex];
            auto& rudders = ruddersArray[searchDepthIndex];
            auto& throttles = throttlesArray[searchDepthIndex];
            auto& nRudders = nRuddersArray[searchDepthIndex];
            auto& nThrottles = nThrottlesArray[searchDepthIndex];
            if (rudderIndex == nRudders && throttleIndex == nThrottles) {
                // Climb down a frame, up the tree, depending on how you're envisioning this.
                // Equivalent of popping the call stack in the imaginary recursive version.
                rudderIndex = 0; throttleIndex = 0;
                searchDepthIndex--;
                continue;
            }
            if (rudderIndex == 0 && throttleIndex == 0) {
                // This is the first time through with this configuration, so set up rudders and throttles.
                // Starting with the extreme control values, add the last control and the neighboring controls
                // if they're unique (not similar to the extreme controls). This uses a small tolerance to account for
                // floating point error.
                nRudders = nInitialRudders; nThrottles = nInitialThrottles; // min, (mid), max
                const auto lowRudder = currentMpcState.LastRudder - rudderGranularity;
                const auto highRudder = currentMpcState.LastRudder + rudderGranularity;
                if (m_MinRudder + c_Tolerance < lowRudder && fabs(m_MidRudder - lowRudder) > c_Tolerance)
                    rudders[nRudders++] = lowRudder;
                if (fabs(m_MidRudder - highRudder) > c_Tolerance && m_MaxRudder > highRudder + c_Tolerance)
                    rudders[nRudders++] = highRudder;
                if (fabs(m_MinRudder - currentMpcState.LastRudder) > c_Tolerance &&
                    fabs(m_MidRudder - currentMpcState.LastRudder) > c_Tolerance &&
                    fabs(m_MaxRudder - currentMpcState.LastRudder) > c_Tolerance)
                    rudders[nRudders++] = currentMpcState.LastRudder;
                const auto lowThrottle = currentMpcState.LastThrottle - throttleGranularity;
                const auto highThrottle = currentMpcState.LastThrottle + throttleGranularity;
                if (m_MinThrottle + c_Tolerance < lowThrottle)
                    throttles[nThrottles++] = lowThrottle;
                if (m_MaxThrottle > highThrottle + c_Tolerance)
                    throttles[nThrottles++] = highThrottle;
                if (fabs(m_MinThrottle - currentMpcState.LastThrottle) > c_Tolerance &&
                    fabs(m_MaxThrottle - currentMpcState.LastThrottle) > c_Tolerance)
                    throttles[nThrottles++] = currentMpcState.LastThrottle;
            }
            auto& next = currentTrajectory[searchDepthIndex + 1];
            // simulate next state with next controls
            next.state = currentMpcState.state.simulate(
                    rudders[rudderIndex], throttles[throttleIndex], c_ScoringTimeStep, currentEstimate);
            // store the rudders
            next.LastRudder = rudders[rudderIndex];
            next.LastThrottle = throttles[throttleIndex];
            // calculate and update the score
            auto score = compareStates(cachedScoringCheckpoints[searchDepthIndex], next.state);
            next.Score = currentMpcState.Score + score;
            // If we're at the max depth,
            if (searchDepthIndex == iterations) {
                // and we found a new best trajectory,
                if (score < minScore) {
                    // save it
                    // (currentTrajectory + iterations + 2 because currentTrajectory iterations is a zero-based count
                    // and copy() assumes the final pointer is like end() in that it contains no data)
                    std::copy(currentTrajectory, currentTrajectory + iterations + 2, bestTrajectory);
                    minScore = score;
                }
            } else { // otherwise (not at max depth):
                // Climb up a frame, down the tree, depending on how you're envisioning this.
                // Equivalent of pushing a new frame on the call stack in the imaginary recursive version.
                searchDepthIndex++;
            }

            // set up for the next time we'll use this frame
            rudderIndex++;
            if (rudderIndex == nRudders) {
                // increment throttle if we've done all the rudders
                throttleIndex++;
                if (throttleIndex != nThrottles) {
                    // wrap the rudder around if we haven't done all the throttles
                    rudderIndex = 0;
                }
            }
        }
        // copy best trajectory into overarching best trajectory
        iterations++; // increase max depth
        // only use iterations + 1 here (see above copy call) because we just incremented it
        overallBestTrajectory = std::vector<MpcState>(iterations + 1); // set capacity
        std::copy(bestTrajectory, bestTrajectory + iterations + 1, overallBestTrajectory.begin());
    }

mpcCleanup: // shush I'm using it sparingly and appropriately

//    std::cerr << "Managed " << iterations << " iterations of limited-branching MPC (" << c_ScoringTimeStep * iterations
//        << " seconds in the future)" << std::endl;

    if (overallBestTrajectory.size() < 2) throw std::runtime_error("MPC failed to complete a single iteration");
    std::vector<State> forDisplay;
    forDisplay.reserve(overallBestTrajectory.size());
    for (const auto& s : overallBestTrajectory) forDisplay.emplace_back(s.state.state);
    m_ControlReceiver->displayTrajectory(forDisplay, false, m_Achievable);
    if (!std::isfinite(overallBestTrajectory[1].LastRudder) || !std::isfinite(overallBestTrajectory[1].LastThrottle))
        throw std::runtime_error("MPC selected non-finite controls");
    static_assert(c_ScoringTimeStep == 1.0, "Assume scoring time-step is equal to the planner's planning time");
    return overallBestTrajectory[1]; // based on c_ScoringTimeStep
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

void Controller::updateConfig(double rudderGranularity, double throttleGranularity, double distanceWeight, double headingWeight,
                              double speedWeight, double achievableThreshold, bool currentEstimation) {
    std::unique_lock<std::mutex> lock(m_ConfigMutex);
    m_RudderGranularity = rudderGranularity; m_ThrottleGranularity = throttleGranularity;
    m_DistanceWeight = distanceWeight; m_HeadingWeight = headingWeight; m_SpeedWeight = speedWeight;
    m_AchievableScoreThreshold = achievableThreshold;
    if (currentEstimation) m_CurrentEstimator.enable(); else m_CurrentEstimator.disable();
}

double Controller::compareStates(const State& s1, const VehicleState& s2) const {
    std::unique_lock<std::mutex> lock(m_ConfigMutex);
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
        return {};
    }

    // set the trajectory number, which lets any already-running MPC thread(s) know to terminate
    setTrajectoryNumber(trajectoryNumber);

    // get the next start state
    auto start = getStateAfterCurrentControl();

    // do first round of MPC so we can return a state based on the new reference trajectory
    auto result = mpc(start.state, referenceTrajectory, m_ControlReceiver->getTime() + m_PlanningTime,
                      trajectoryNumber);

    sendControls(result.LastRudder, result.LastThrottle);

    // join the last thread used for MPC
    // should be able to wait for no time at all but I'll give it some leeway for now
    auto status = m_LastMpc.wait_for(std::chrono::milliseconds((int)(1000 * m_PlanningTime)));
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
    m_LastMpc = std::async(std::launch::async, [=]{ runMpc(referenceTrajectory, trajectoryNumber); });

    // determine if reference trajectory seems feasible ("close enough")
    State stateOnReferenceTrajectory;
    stateOnReferenceTrajectory.time() = result.state.state.time();
    referenceTrajectory.sample(stateOnReferenceTrajectory);
    auto score = compareStates(stateOnReferenceTrajectory, result.state);
    {
        std::unique_lock<std::mutex> lock(m_ConfigMutex);
        if (score <= m_AchievableScoreThreshold) {
            // Controller deems us close enough
            result.state = stateOnReferenceTrajectory;
            m_Achievable = true;
        } else {
            // if the score threshold is set to zero we always plan from controller's prediction, so don't report failure
            m_Achievable = m_AchievableScoreThreshold == 0;
            if (!m_Achievable)
                std::cerr << "Controller doesn't think we can make the reference trajectory (score = " << score << ")"
                          << std::endl;
        }
    }

    return result.state.state;
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
    start.simulate(m_LastRudder, m_LastThrottle, m_PlanningTime, current);
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

void Controller::runMpc(DubinsPlan trajectory, long trajectoryNumber) {
    auto endTime = m_ControlReceiver->getTime() + c_ReferenceTrajectoryExpirationTime;
    while (m_ControlReceiver->getTime() < endTime) {
        if (!validTrajectoryNumber(trajectoryNumber)) break;
        auto stateAfterCurrentControl = getStateAfterCurrentControl();
        if (!trajectory.containsTime(stateAfterCurrentControl.state.time() + 3 * c_ScoringTimeStep)) break; // why 3? Seems like a good number of iterations
        auto result = mpc(stateAfterCurrentControl.state, trajectory, m_ControlReceiver->getTime() + m_PlanningTime,
            trajectoryNumber);
        sendControls(result.LastRudder, result.LastThrottle);
    }
    if (m_ControlReceiver->getTime() >= endTime) {
        std::cerr << "Controller's reference trajectory appears to have timed out. No more controls will be issued" << std::endl;
    }
//    cerr << "Ending an old thread running MPC" << endl;
}



