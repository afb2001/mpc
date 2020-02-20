#include <thread>
#include <path_planner/Trajectory.h>
#include <path_planner/TrajectoryDisplayer.h>
#include <queue>
#include <future>
#include "controller.h"

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

void Controller::updateConfig(int rudders, int throttles,
                              double distanceWeight, double headingWeight, double speedWeight) {
    m_Rudders = rudders; m_Throttles = throttles;
    m_DistanceWeight = distanceWeight; m_HeadingWeight = headingWeight; m_SpeedWeight = speedWeight;
}

double Controller::compareStates(const State& s1, const VehicleState& s2) const {
    return compareStates(s1, s2.state);
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
                intermediateResult = s.OneSecondOut;
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

State Controller::updateReferenceTrajectory(const vector<State>& trajectory, long trajectoryNumber) {
//    cerr << "Controller received reference trajectory: ";
//    for (const auto s : trajectory) cerr << "\n" << s.toString();
//    cerr << endl;

    if (trajectory.size() < 2) {
        // Not long enough for MPC. Balk out.
        // NOTE: previous MPC thread may still be running; that can time out on its own
        std::cerr << "Reference trajectory of length " << trajectory.size() << " too short for MPC. Reference trajectory will not be updated." << std::endl;
        return State();
    }
    // new scope to use RAII
    setTrajectoryNumber(trajectoryNumber);

    auto start = getStateAfterCurrentControl();

    double r = 0, t = 0;
//    cerr << "Doing initial mpc..." << endl;
    auto result = initialMpc(r, t, start, trajectory, m_ControlReceiver->getTime() + c_PlanningTime);
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
    m_LastMpc = std::async(std::launch::async, [=]{ runMpc(trajectory, start, result, trajectoryNumber); });

//    auto mpcThread = thread([=]{ runMpc(trajectory, start, result, trajectoryNumber); });
//    mpcThread.detach();

    auto interpolated = interpolateTo(result.time(), trajectory);
    auto score = compareStates(result, interpolated);
    if (score <= c_CloseEnoughScoreThreshold) {
        result = interpolated;
//        result.time() = -2; // invalid time meaning we're close enough
//        std::cerr << "Controller deems us close enough (score = " << score << ")" << std::endl;
    } else {
        std::cerr << "Controller doesn't think we can make the reference trajectory (score = " << score << ")" << std::endl;
    }

    return result;
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

void Controller::runMpc(std::vector<State> trajectory, State start, State result, long trajectoryNumber) {
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
        mpc(r, t, stateAfterCurrentControl, trajectory, m_ControlReceiver->getTime() + c_PlanningTime, trajectoryNumber);
        sendControls(r, t);
    }
    if (m_ControlReceiver->getTime() >= endTime) {
        std::cerr << "Controller's reference trajectory appears to have timed out. No more controls will be issued" << std::endl;
    }
//    cerr << "Ending an old thread running MPC" << endl;
}

double Controller::compareStates(const State& s1, const State& s2) const {
    // ignore differences in time
    double headingDiff = fabs(fmod((s1.heading() - s2.heading()), 2 * M_PI));
    auto speedDiff = fabs(s1.speed() - s2.speed());
    auto dx = s1.x() - s2.x();
    auto dy = s1.y() - s2.y();
    auto d = sqrt(dx * dx + dy * dy);
    return m_DistanceWeight * d + m_HeadingWeight * headingDiff * m_SpeedWeight * speedDiff;
}



