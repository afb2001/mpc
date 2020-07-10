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
    m_Output = &std::cerr; // default output
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
    // just checking here too so we can skip stuff
    if (!validTrajectoryNumber(trajectoryNumber)) {
        return {};
    }
//    std::pair<double, double> disturbanceEstimate = m_DisturbanceEstimator.getCurrent(startState);

//    *m_Output << "Current estimated to be " << m_DisturbanceEstimate.first << ", " << m_DisturbanceEstimate.second << std::endl;

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
//            *m_Output << "Trajectory number " << trajectoryNumber << " not valid." << endl;
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
                    rudders[rudderIndex], throttles[throttleIndex], c_ScoringTimeStep, m_DisturbanceEstimate);
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

//    m_Output << "Managed " << iterations << " iterations of limited-branching MPC (" << c_ScoringTimeStep * iterations
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
    m_DisturbanceEstimator.resetEstimate(); // we don't know how long we'll be down so the current may have changed
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
    if (currentEstimation) m_DisturbanceEstimator.enable(); else m_DisturbanceEstimator.disable();
}

double Controller::compareStates(const State& s1, const VehicleState& s2) const {
    std::unique_lock<std::mutex> lock(m_ConfigMutex);
    // ignore differences in time
    auto headingDiff = fabs(s1.headingDifference(s2.courseMadeGood));
    auto speedDiff = fabs(s1.speed() - s2.speedOverGround);
    auto dx = s1.x() - s2.state.x();
    auto dy = s1.y() - s2.state.y();
    // d is now exponential in distance. This means I probably ought to expose a second parameter but I don't have time.
    auto d = exp(sqrt(dx * dx + dy * dy));
    return m_DistanceWeight * d + m_HeadingWeight * headingDiff + m_SpeedWeight * speedDiff;
}

bool Controller::validTrajectoryNumber(long trajectoryNumber) {
    std::unique_lock<mutex> lock1(m_TrajectoryNumberMutex);
    return trajectoryNumber == m_TrajectoryNumber;
}

State Controller::updateReferenceTrajectory(const DubinsPlan& referenceTrajectory, long trajectoryNumber,
                                            bool getFutureStateEstimate) {
    if (referenceTrajectory.empty()){
        // Not long enough for MPC. Balk out.
        // NOTE: previous MPC thread may still be running; that can time out on its own
        *m_Output << "Reference trajectory empty and cannot be used for MPC. Reference trajectory will not be updated." << std::endl;
        return {};
    }

    // set the trajectory number, which lets any already-running MPC thread(s) know to terminate
    setTrajectoryNumber(trajectoryNumber);

    // get the next start state
    auto start = getStateAfterCurrentControl();

    MpcState result;

    if (getFutureStateEstimate) {
        // do first round of MPC so we can return a state based on the new reference trajectory
        result = mpc(start.state, referenceTrajectory, m_ControlReceiver->getTime() + m_PlanningTime,
                          trajectoryNumber);

        sendControls(result.LastRudder, result.LastThrottle);
    }

    // join the last thread used for MPC
    // should be able to wait for no time at all but I'll give it some leeway for now
    auto status = m_LastMpc.wait_for(std::chrono::milliseconds((int)(1000 * m_PlanningTime)));
    switch (status) {
        case std::future_status::ready:
            // good
            m_LastMpc.get(); // not sure this is necessary
            break;
        case std::future_status::timeout:
            // bad
            throw std::runtime_error("Old MPC thread refused to die");
            break;
        default:
            // shouldn't ever get here
            *m_Output << "Shouldn't ever get here" << std::endl;
            break;
    }

    // Kick off the new MPC thread
    // Copies reference trajectory so we shouldn't have to deal with issues of a reference to a local variable going out of scope
    // There are a bunch of different ways of doing this - I went with std::async so I could hold onto the future object
    // and potentially report failures. Occasionally get that runtime error up there and I don't know why yet.
    m_LastMpc = std::async(std::launch::async, [=]{ runMpc(referenceTrajectory, trajectoryNumber); });

    if (getFutureStateEstimate) {
        // Determine if reference trajectory seems feasible ("close enough")
        // It is beneficial to do this because of the nature of the planner - it uses Dubins curves to compute trajectories
        // between poses. Since Dubins curves use minimal radii, while walking along a curve a slight perturbation in the
        // starting or ending pose can result in a substantially longer curve if the original suffix is infeasible. While
        // the controller is hopefully pretty good, we can never hope to get rid of all actuation noise, and it turns out
        // that this issue presents itself on the scale of floating point inaccuracy anyway. If we're in a situation where
        // we expect to be finding the same plan several times in a row (advancing slowly in time), the difference is the
        // controller's estimation of our position at the start of the planning iteration can differ enough from the previous
        // plan such that the plan's suffix is infeasible without making a loop. If we allow the controller to  determine
        // that we're pretty close to the reference trajectory, and if we kept going we'd probably do pretty well, we can
        // circumvent the issue by handing the planner a state along the reference trajectory in those cases. If we're far
        // away, we should give a real estimate so we can get back on track. Ideally there should be some kind of mechanism
        // in place that monitors how often we get off track and alerts the operator, because it might be due to a threshold
        // being too low, bad controller scoring parameters, or just a hard sea state to operate in.
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
//                if (!m_Achievable) {
//                    *m_Output << "Controller doesn't think we can make the reference trajectory (score = " << score
//                              << ")"
//                              << std::endl;
//                }
                // set heading to course made good, speed to speed over ground (accounting for current)
                result.state.state.heading() = result.state.courseMadeGood;
                result.state.state.speed() = result.state.speedOverGround;
            }
        }
        return result.state.state;
    } else return {};
}

void Controller::setTrajectoryNumber(long trajectoryNumber) {
    unique_lock<mutex> lock(m_TrajectoryNumberMutex);
    // updating the trajectory number stops the previous MPC thread
    m_TrajectoryNumber = trajectoryNumber;
}

VehicleState Controller::getStateAfterCurrentControl() {
    // TODO! -- need to record several past controls so that they can be applied since the last state update was
    // received, potentially up to a whole second ago if we're using GPS
    State startCopy;
    {
        std::unique_lock<mutex> lock(m_CurrentLocationMutex);
        startCopy = m_CurrentLocation;
    }
    // TODO! -- figure out concurrency issues here - last controls should be protected somehow
    VehicleState start(startCopy);
//    auto disturbance = m_DisturbanceEstimator.getCurrent(startCopy);
    // simulate last issued control to how long it will take us to compute the next one
    start.simulate(m_LastRudder, m_LastThrottle, m_PlanningTime, m_DisturbanceEstimate);
    return start;
}

void Controller::sendControls(double r, double t) {
    // TODO! -- concurrency safety?
    m_LastRudder = r;
    m_LastThrottle = t;
    m_ControlReceiver->receiveControl(r, t);
    std::unique_lock<mutex> lock(m_CurrentLocationMutex);
//    m_DisturbanceEstimator.updateEstimate(m_CurrentLocation, r, t);
}

void Controller::runMpc(DubinsPlan trajectory, long trajectoryNumber) {
    // this is the MPC thread loop
    // receives a copy of the reference trajectory because we don't want a reference to a local variable which will
    // go out of scope while we're using it
    // first, calculate when this reference trajectory expires
    auto endTime = m_ControlReceiver->getTime() + c_ReferenceTrajectoryExpirationTime;
    // while it has not expired,
    while (m_ControlReceiver->getTime() < endTime) {
        // make sure no new trajectory has come in, invalidating this one
        if (!validTrajectoryNumber(trajectoryNumber)) break;
        // simulate controls to estimate our position at the end of MPC (0.1s from now)
        auto stateAfterCurrentControl = getStateAfterCurrentControl();
        // make sure the reference trajectory is long enough to do a depth 3 search
        if (!trajectory.containsTime(stateAfterCurrentControl.state.time() + 3 * c_ScoringTimeStep)) break; // why 3? Seems like a good number of iterations
        // actually run MPC
        auto result = mpc(stateAfterCurrentControl.state, trajectory, m_ControlReceiver->getTime() + m_PlanningTime,
            trajectoryNumber);
        // pass controls to /helm through the node
        sendControls(result.LastRudder, result.LastThrottle);
    }
    if (m_ControlReceiver->getTime() >= endTime) {
        *m_Output << "Controller's reference trajectory appears to have timed out. No more controls will be issued" << std::endl;
        // let node know we timed out
        m_ControlReceiver->timedOut();
    }
//    *m_Output << "Ending an old thread running MPC" << endl;
}

void Controller::updateDisturbanceEstimate(double x, double y) {
    m_DisturbanceEstimate = std::make_pair(x, y);
}



