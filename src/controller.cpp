#include <thread>
#include <path_planner/Trajectory.h>
#include <path_planner/TrajectoryDisplayer.h>
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

void Controller::mpc(double& r, double& t, State startCopy, std::vector<State> referenceTrajectoryCopy, double endTime, long trajectoryNumber)
{
    std::pair<double, double> currentEstimate = m_CurrentEstimator.getCurrent(startCopy);

//    std::cerr << "Current estimated to be " << currentEstimate.first << ", " << currentEstimate.second << std::endl;

//    cerr << "Starting MPC (3)" << endl;
    assert(referenceTrajectoryCopy.size() >= 2); // make sure reference trajectory is long enough

    // intentional use of integer division (m_Rudders / 2)
    double rudderGranularity = 1.0 / (m_Rudders / 2), throttleGranularity = 1.0 / (m_Throttles - 1);
    double minRudder = -1, midRudder = 0, maxRudder = 1;
    double minThrottle = 0, maxThrottle = 1;

    // Use a queue to do breadth first search in the space of future controls, keeping track of only the first control
    // in the sequence, which is what is to be emitted, and the most recent control, so we know which controls to expand
    // with next.
    struct MpcState {
    public:
        VehicleState State;
        double InitialRudder, InitialThrottle;
        double LastRudder, LastThrottle;
        int Depth;
        double PrevScore;
    };

    std::queue<MpcState> open; // "open" for open list
    MpcState startMpcState = {
            startCopy,
            NAN,
            NAN,
            m_LastRudder,
            m_LastThrottle,
            0,
            0,
    };
    open.push(startMpcState);

    int iterations = 1; // will go through loop once for start and then for each descendent before r and t are updated
    auto minScore = DBL_MAX;
    double bestCurrentRudder = 0, bestCurrentThrottle = 0;
    State lastInterpolated; // cache last interpolated state on reference trajectory
    while (m_ControlReceiver->getTime() < endTime) {
        // cut out when the reference trajectory is updated
        if (!validTrajectoryNumber(trajectoryNumber)) {
//            cerr << "Trajectory number " << trajectoryNumber << " not valid." << endl;
            return;
        }
        assert(!open.empty());
        auto s = open.front(); open.pop();

        if (s.Depth > iterations) {
            assert(s.Depth == iterations + 1);
            // new iteration
            iterations = s.Depth;
            minScore = DBL_MAX;
            // since the iteration is done, assign the best rudder and throttle
            r = bestCurrentRudder;
            t = bestCurrentThrottle;
//            std::cerr << "Assigned r and t" << std::endl;
        }

        bool filledTrajectory = iterations > referenceTrajectoryCopy.size();
        if (filledTrajectory) {
            cerr << "Filled entire trajectory" << endl;
            break;
        }

        // set up (unique) rudders
        // use tolerance to avoid very similar values due to floating point error
        std::vector<double> rudders = {minRudder, midRudder, maxRudder};
        auto lowRudder = s.LastRudder - rudderGranularity;
        auto highRudder = s.LastRudder + rudderGranularity;
        if (minRudder + c_Tolerance < lowRudder && fabs(midRudder - lowRudder) > c_Tolerance) rudders.push_back(lowRudder);
        if (fabs(midRudder - highRudder) > c_Tolerance && maxRudder > highRudder + c_Tolerance) rudders.push_back(highRudder);
        if (fabs(minRudder - s.LastRudder) > c_Tolerance &&
            fabs(midRudder - s.LastRudder) > c_Tolerance &&
            fabs(maxRudder - s.LastRudder) > c_Tolerance) rudders.push_back(s.LastRudder);

        // and (unique) throttles
        std::vector<double> throttles = {minThrottle, maxThrottle};
        auto lowThrottle = s.LastThrottle - throttleGranularity;
        auto highThrottle = s.LastThrottle + throttleGranularity;
        if (minThrottle + c_Tolerance < lowThrottle) throttles.push_back(lowThrottle);
        if (maxThrottle > highThrottle + c_Tolerance) throttles.push_back(highThrottle);
        if (fabs(minThrottle - s.LastThrottle) > c_Tolerance &&
            fabs(maxThrottle - s.LastThrottle) > c_Tolerance) throttles.push_back(s.LastThrottle);

        // If we're before the state given to the planner, full weight, otherwise 1/10th weight.
        double weight = 1.0; // s.State.time < referenceTrajectoryCopy[1].time ? 1.0 : 0.1;
        if (lastInterpolated.time() != s.State.state.time()) {
            lastInterpolated = interpolateTo(s.State.state.time(), referenceTrajectoryCopy);
        }
        auto score = compareStates(lastInterpolated, s.State) * weight + s.PrevScore;
        if (s.Depth > 0) {
            if (score < minScore) {
                bestCurrentRudder = s.InitialRudder;
                bestCurrentThrottle = s.InitialThrottle;
                minScore = score;
//                std::cerr << "Found new best trajectory with initial controls " << bestCurrentRudder << ", " << bestCurrentThrottle << std::endl;
            }
        } else {
            score = 0;
        }
        // expansion for breadth first search
        for (auto rudder : rudders) {
            for (auto throttle : throttles) {
                auto next = s.State.simulate(rudder, throttle, c_ScoringTimeStep, currentEstimate);
                MpcState nextMpcState{
                    next,
                    s.InitialRudder,
                    s.InitialThrottle,
                    rudder,
                    throttle,
                    s.Depth + 1,
                    score,
                    };
                if (s.Depth == 0) {
                    // assign initial rudder and throttle on first iteration
                    nextMpcState.InitialRudder = rudder; nextMpcState.InitialThrottle = throttle;
                }
                open.push(nextMpcState);
            }
        }
    }

//    std::cerr << "Managed " << iterations << " iterations of limited-branching MPC (" << c_ScoringTimeStep * iterations
//        << " seconds in the future)" << std::endl;

    assert(std::isfinite(r) && std::isfinite(t));
}

VehicleState Controller::mpc2(double& r, double& t, State startCopy, DubinsPlan referenceTrajectoryCopy, double endTime, long trajectoryNumber)
{
    std::pair<double, double> currentEstimate = m_CurrentEstimator.getCurrent(startCopy);

//    std::cerr << "Current estimated to be " << currentEstimate.first << ", " << currentEstimate.second << std::endl;

//    cerr << "Starting MPC" << endl;
    assert(!referenceTrajectoryCopy.empty()); // make sure reference trajectory is long enough

//    cerr << "Starting mpc with plan:" << endl;
//    for (const auto& w : referenceTrajectoryCopy.get()) {
//        auto d = w.unwrap();
//        cerr << "\t" << d.qi[0] << " " << d.qi[1] << " " << d.qi[2] << " " << fmod(w.getStartTime(), 1000) << " " << fmod(w.getEndTime(), 1000) << endl;
//    }

    // intentional use of integer division (m_Rudders / 2)
    const double rudderGranularity = 1.0 / (m_Rudders / 2), throttleGranularity = 1.0 / (m_Throttles - 1);
    double minRudder = -1, midRudder = 0, maxRudder = 1;
    double minThrottle = 0, maxThrottle = 1;

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
    auto trajectoryStartTime = referenceTrajectoryCopy.get().front().getStartTime();
    const int maxIterations = (trajectoryStartTime - startCopy.time() + referenceTrajectoryCopy.totalTime()) / c_ScoringTimeStep - 1;
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
        referenceTrajectoryCopy.sample(interpolated[iterations]);
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
    for (const auto& s : overallBestTrajectory) forDisplay.emplace_back(s.state.state);
    m_ControlReceiver->displayTrajectory(forDisplay, false, m_Achievable);
    assert(std::isfinite(r) && std::isfinite(t));
    static_assert(c_ScoringTimeStep == 1.0);
    return overallBestTrajectory[1].state; // based on c_ScoringTimeStep
//    return interpolateTo(startCopy.time() - c_PlanningTime, forDisplay);
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
                              double achievableThreshold) {
    m_Rudders = rudders; m_Throttles = throttles;
    m_DistanceWeight = distanceWeight; m_HeadingWeight = headingWeight; m_SpeedWeight = speedWeight;
    m_AchievableScoreThreshold = achievableThreshold;
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

State Controller::interpolateTo(double desiredTime, const std::vector<State>& trajectory) {
    return StateInterpolater::interpolateTo(desiredTime, trajectory);
}

bool Controller::validTrajectoryNumber(long trajectoryNumber) {
    std::unique_lock<mutex> lock1(m_TrajectoryNumberMutex);
    return trajectoryNumber == m_TrajectoryNumber;
}

State Controller::initialMpc(double& r, double& t, State startCopy, const std::vector<State>& referenceTrajectoryCopy,
                             double endTime) {

    static_assert(c_ScoringTimeStep == 1, "This method makes an assumption about the scoring time step");

//    cerr << "Starting MPC (4)" << endl;
    std::pair<double, double> currentEstimate = m_CurrentEstimator.getCurrent(startCopy);

//    std::cerr << "Current estimated to be " << currentEstimate.first << ", " << currentEstimate.second << std::endl;

    assert(referenceTrajectoryCopy.size() >= 2); // make sure reference trajectory is long enough

    // intentional use of integer division (m_Rudders / 2)
    double rudderGranularity = 1.0 / (m_Rudders / 2), throttleGranularity = 1.0 / (m_Throttles - 1);
    double minRudder = -1, midRudder = 0, maxRudder = 1;
    double minThrottle = 0, maxThrottle = 1;

    // Use a queue to do breadth first search in the space of future controls, keeping track of only the first control
    // in the sequence, which is what is to be emitted, and the most recent control, so we know which controls to expand
    // with next.
    struct MpcState {
    public:
        VehicleState State;
        VehicleState OneSecondOut;
        double InitialRudder, InitialThrottle;
        double LastRudder, LastThrottle;
        int Depth;
        double PrevScore;
    };

    std::queue<MpcState> open; // "open" for open list
    MpcState startMpcState = {
            startCopy,
            State(),
            NAN,
            NAN,
            m_LastRudder,
            m_LastThrottle,
            0,
            0,
    };
    open.push(startMpcState);

    int iterations = 1; // will go through loop once for start and then for each descendent before r and t are updated
    auto minScore = DBL_MAX;
    double bestCurrentRudder = 0, bestCurrentThrottle = 0;
    State lastInterpolated; // cache last interpolated state on reference trajectory
    State result, intermediateResult; // state one second in the future
    while (m_ControlReceiver->getTime() < endTime) {
        assert(!open.empty());
//        cerr << "Current time, " << m_ControlReceiver->getTime() << ", still less than MPC end time, " << endTime << endl;
        auto s = open.front(); open.pop();

        if (s.Depth > iterations) {
            assert(s.Depth == iterations + 1);
            // new iteration
            iterations = s.Depth;
            minScore = DBL_MAX;
            // since the iteration is done, assign the best rudder and throttle
            r = bestCurrentRudder;
            t = bestCurrentThrottle;
            result = intermediateResult;
        }

        bool filledTrajectory = iterations > referenceTrajectoryCopy.size();
        if (filledTrajectory) {
            cerr << "Filled entire trajectory" << endl;
            break;
        }

        // set up (unique) rudders
        // use tolerance to avoid very similar values due to floating point error
        std::vector<double> rudders = {minRudder, midRudder, maxRudder};
        auto lowRudder = s.LastRudder - rudderGranularity;
        auto highRudder = s.LastRudder + rudderGranularity;
        if (minRudder + c_Tolerance < lowRudder && fabs(midRudder - lowRudder) > c_Tolerance) rudders.push_back(lowRudder);
        if (fabs(midRudder - highRudder) > c_Tolerance && maxRudder > highRudder + c_Tolerance) rudders.push_back(highRudder);
        if (fabs(minRudder - s.LastRudder) > c_Tolerance &&
            fabs(midRudder - s.LastRudder) > c_Tolerance &&
            fabs(maxRudder - s.LastRudder) > c_Tolerance) rudders.push_back(s.LastRudder);

        // and (unique) throttles
        std::vector<double> throttles = {minThrottle, maxThrottle};
        auto lowThrottle = s.LastThrottle - throttleGranularity;
        auto highThrottle = s.LastThrottle + throttleGranularity;
        if (minThrottle + c_Tolerance < lowThrottle) throttles.push_back(lowThrottle);
        if (maxThrottle > highThrottle + c_Tolerance) throttles.push_back(highThrottle);
        if (fabs(minThrottle - s.LastThrottle) > c_Tolerance &&
            fabs(maxThrottle - s.LastThrottle) > c_Tolerance) throttles.push_back(s.LastThrottle);

        // If we're before the state given to the planner, full weight, otherwise 1/10th weight.
        double weight = 1.0; //s.State.time() < referenceTrajectoryCopy[1].time() ? 1.0 : 0.1;
        if (lastInterpolated.time() != s.State.state.time()) {
            lastInterpolated = interpolateTo(s.State.state.time(), referenceTrajectoryCopy);
        }
        auto score = compareStates(lastInterpolated, s.State) * weight + s.PrevScore;
        if (score < 0 || !isfinite(score)) {
            std::cerr << "Uh oh. Score is " << score << std::endl;
            std::cerr << "States: " << lastInterpolated.toString() << ", " << s.State.toString() << std::endl;
        }
        assert(isfinite(score));
        assert(score >= 0 && "Score should be non-negative");
        if (s.Depth > 0) {
            if (score < minScore) {
                bestCurrentRudder = s.InitialRudder;
                bestCurrentThrottle = s.InitialThrottle;
                minScore = score;
                intermediateResult = s.OneSecondOut.state;
//                std::cerr << "Found new best trajectory with initial controls " << bestCurrentRudder << ", " << bestCurrentThrottle << std::endl;
            }
        } else {
            score = 0;
        }
        // expansion for breadth first search
        for (auto rudder : rudders) {
            for (auto throttle : throttles) {
                auto next = s.State.simulate(rudder, throttle, c_ScoringTimeStep, currentEstimate);
                // If we're before iteration 2 we haven't gotten to one second out yet. On iteration 2, that is 1s in
                // the future, so assign the field to the current state. Past that, grab the previous value
//                auto oneSecondOut = iterations < 2? VehicleState(State(-1)) :
//                        iterations == 2? s.State : s.OneSecondOut;
                auto oneSecondOut = iterations == 1? s.State : s.OneSecondOut;
                MpcState nextMpcState{
                        next,
                        oneSecondOut,
                        s.InitialRudder,
                        s.InitialThrottle,
                        rudder,
                        throttle,
                        s.Depth + 1,
                        score,
                };
                if (s.Depth == 0) {
                    // assign initial rudder and throttle on first iteration
                    nextMpcState.InitialRudder = rudder; nextMpcState.InitialThrottle = throttle;
                }
                open.push(nextMpcState);
            }
        }
    }

//    std::cerr << "Managed " << iterations << " iterations of limited-branching MPC (" << c_ScoringTimeStep * iterations
//              << " seconds in the future)" << std::endl;

    assert(std::isfinite(r) && std::isfinite(t));

    return result;
}

State Controller::updateReferenceTrajectory(const DubinsPlan& plan, long trajectoryNumber) {
//    cerr << "Controller received reference trajectory: ";
//    for (const auto s : trajectory) cerr << "\n" << s.toString();
//    cerr << endl;

    if (plan.empty()){
//    if (trajectory.size() < 2) {
        // Not long enough for MPC. Balk out.
        // NOTE: previous MPC thread may still be running; that can time out on its own
        std::cerr << "Reference trajectory empty and cannot be used for MPC. Reference trajectory will not be updated." << std::endl;
        return State();
    }

//    std::cerr << "Starting MPC with plan starting at time " << plan.get().front().getStartTime() << " and going to " << plan.get().back().getEndTime() << std::endl;

    // new scope to use RAII
    setTrajectoryNumber(trajectoryNumber);

    auto start = getStateAfterCurrentControl();

    double r = 0, t = 0;
//    cerr << "Doing initial mpc..." << endl;
    auto result = mpc2(r, t, start.state, plan, m_ControlReceiver->getTime() + c_PlanningTime, trajectoryNumber); // initialMpc(r, t, start, trajectory, m_ControlReceiver->getTime() + c_PlanningTime);
//    cerr << "Finished initial mpc" << endl;

    sendControls(r, t);

    // join the last thread used for MPC
    // should be able to wait for no time at all but I'll give it some leeway for now
    auto status = m_LastMpc.wait_for(std::chrono::milliseconds((int)(1000 * c_PlanningTime)));
    switch (status) {
        case std::future_status::ready:
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
    // kick off the new MPC thread
    // copies reference trajectory so we shouldn't have to deal with issues of a reference to a local variable going out of scope
    m_LastMpc = std::async(std::launch::async, [=]{ runMpc(plan, start.state, result.state, trajectoryNumber); });

//    auto mpcThread = thread([=]{ runMpc(trajectory, start, result, trajectoryNumber); });
//    mpcThread.detach();

    State interpolated;
    interpolated.time() = result.state.time();
    plan.sample(interpolated);
    auto score = compareStates(interpolated, result);
    if (score <= m_AchievableScoreThreshold) {
        result = interpolated;
        m_Achievable = true;
//        result.time() = -2; // invalid time meaning we're close enough
//        std::cerr << "Controller deems us close enough (score = " << score << ")" << std::endl;
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
//    cerr << "Sending controls " << r << ", " << t << endl;
    m_LastRudder = r;
    m_LastThrottle = t;
    m_ControlReceiver->receiveControl(r, t);
    std::unique_lock<mutex> lock(m_CurrentLocationMutex);
    m_CurrentEstimator.updateEstimate(m_CurrentLocation, r, t);
}

void Controller::runMpc(DubinsPlan trajectory, State start, State result, long trajectoryNumber) {
//    cerr << "Starting new thread for MPC loop" << endl;
    // Set updated start state and the state we're passing on to the planner in appropriate spots in the reference trajectory
//    int startIndex = -1, goalIndex = -1;
//    for (int i = 1; i < trajectory.size(); i++) {
//        if (start.time() >= trajectory[i].time()) continue;
//        else if (startIndex == -1){
//            startIndex = i - 1;
//            trajectory[startIndex] = start;
//        }
//        if (result.time() < trajectory[i].time() && goalIndex == -1) {
//            goalIndex = i - 1;
//            trajectory[goalIndex] = result;
//        }
//        if (goalIndex != -1 && startIndex != -1) break;
//    }
    auto endTime = m_ControlReceiver->getTime() + c_ReferenceTrajectoryExpirationTime;
    double r = 0, t = 0;
    while (m_ControlReceiver->getTime() < endTime) {
        if (!validTrajectoryNumber(trajectoryNumber)) break;
        auto stateAfterCurrentControl = getStateAfterCurrentControl();
//        for (int i = startIndex; i < trajectory.size(); i++) {
//            if (stateAfterCurrentControl.state.time() < trajectory[i].time()) {
//                if (i - 1 != goalIndex) { // don't overwrite the state we gave to the planner
//                    startIndex = i - 1;
//                    trajectory[startIndex] = stateAfterCurrentControl;
//                }
//                break;
//            }
//        }
        if (!trajectory.containsTime(stateAfterCurrentControl.state.time() + 3 * c_ScoringTimeStep)) break; // why 3? Seems like a good number of iterations
        mpc2(r, t, stateAfterCurrentControl.state, trajectory, m_ControlReceiver->getTime() + c_PlanningTime, trajectoryNumber);
        sendControls(r, t);
    }
    if (m_ControlReceiver->getTime() >= endTime) {
        std::cerr << "Controller's reference trajectory appears to have timed out. No more controls will be issued" << std::endl;
    }
//    cerr << "Ending an old thread running MPC" << endl;
}



