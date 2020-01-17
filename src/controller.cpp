#include <thread>
#include <path_planner/Trajectory.h>
#include <path_planner/TrajectoryDisplayer.h>
#include <queue>
#include "controller.h"

using namespace std;

Controller::Controller(ControlReceiver *controlReceiver) {
    m_ControlReceiver = controlReceiver;
}

Controller::~Controller() = default;

double Controller::getTime()
{
    struct timespec t{};
    clock_gettime(CLOCK_REALTIME, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}

void Controller::receiveRequest(const std::vector<State>& trajectory, long trajectoryNumber)
{
    mtx.lock();
//    start.set(trajectory->states[0]);
//    m_ReferenceTrajectory.clear();
//    for (int i = 1; i < trajectory->states.size() && i < MAX_LOOKAHEAD_STEPS; i++)
//    {
//        m_ReferenceTrajectory.emplace_back(State(trajectory->states[i]));
//    }
    m_ReferenceTrajectory = trajectory;
    m_NextTrajectoryNumber = trajectoryNumber;
    mtx.unlock();
}

void Controller::mpc(double& r, double& t, State startCopy, vector<State> referenceTrajectoryCopy, double endTime, long trajectoryNumber)
{
    if (referenceTrajectoryCopy.empty())
        return;

    m_FutureStuffMutex.lock();
//    m_CurrentEstimator.updateEstimate2(startCopy, m_PredictedTrajectory);
    m_CurrentEstimator.updateEstimate(startCopy, m_PredictedTrajectory);
//    m_CurrentEstimator.update(startCopy);
    m_FutureStuffMutex.unlock();

//    cerr << "Starting MPC from " << startCopy.toString() << " with reference trajectory of length " << referenceTrajectoryCopy.size() << endl;

    r = 0; t = 0;
    vector<Control> controls;
    vector<VehicleState> futureStates;
    vector<pair<double,double>> futureControls;
    vector<VehicleState> future;
    vector<double> scores;
    vector<vector<VehicleState>> simulatedStates;
    int rudderGranularity = m_Rudders / 2, throttleGranularity = m_Throttles - 1; // 10 rudders, 4 throttles
    int iterations;
    auto minScore = DBL_MAX;
    for (iterations = 1;  m_ControlReceiver->getTime() < endTime; iterations++) {
//        cerr << "Last iteration min score was " << minScore << endl;
//        minScore = DBL_MAX;
        bool filledTrajectory = iterations > referenceTrajectoryCopy.size();
//        if (filledTrajectory) cerr << "Found controls for entire reference trajectory" << endl;
        if (filledTrajectory) {
            rudderGranularity += rudderGranularity;
            throttleGranularity += throttleGranularity;
        }
        auto n = filledTrajectory ? referenceTrajectoryCopy.size() : iterations;
        controls.clear();
        controls = vector<Control>(n, Control(rudderGranularity, throttleGranularity));
        futureStates.clear();
        futureStates.emplace_back(startCopy);
        scores.clear();
        scores.push_back(0);
        simulatedStates.clear();
        simulatedStates.emplace_back();
        int i = 0;
        // i represents the index we are in the reference trajectory (and the controls vector). It could also be used to
        // index futureStates and scores, but those should be able to use back() instead, and conceptually that
        // makes a little more sense to me, as I'm treating them as stacks.
        while (i >= 0) {
//            cerr << "Depth = " << i << endl;
            if (m_ControlReceiver->getTime() >= endTime) {
//                cerr << "Managed " << iterations << " iterations of MPC" << endl;
                return; // let's be as real-time as possible
            }
            auto c = &controls[i];
            // If we're done with this throttle (used all rudders), increment it and reset the rudders
            if (c->rudderDone()) {
                c->incrementThrottle();
                c->resetRudder();
            }
            // If we're done with all the throttles we've finished this node in the "tree" so pop the states and scores
            // stacks and decrement the reference trajectory index
            if (c->throttleDone()) {
                c->resetThrottle();
                i--;
                futureStates.pop_back();
                scores.pop_back();
                simulatedStates.pop_back();
                if (i >= 0) controls[i].incrementRudder();
                continue;
            }
            // generate the next state and score it
            double timeDiff = referenceTrajectoryCopy[i].time - futureStates.back().time; // back() should be same as [i]
            if(timeDiff < 0) {
                cerr << "Time difference between the following states is below zero:\n";
                cerr << referenceTrajectoryCopy[i].toString() << '\n' << futureStates.back().toString() << endl;
            }
            vector<VehicleState> simulated;
//            auto newState = futureStates.back().simulate(c->getRudder(), c->getThrottle(), timeDiff,
//                    m_CurrentEstimator.getCurrentDirection(),
//                    m_CurrentEstimator.getCurrentSpeed(), simulated);
            auto newState = futureStates.back().simulate(c->getRudder(), c->getThrottle(), timeDiff, m_CurrentEstimator.getCurrent(), simulated);
            double s = compareStates(referenceTrajectoryCopy[i], newState) * getMPCWeight(newState.time - startCopy.time) + scores.back();
//            double s = referenceTrajectoryCopy[i].getDistanceScore(newState) * getMPCWeight(newState.time - startCopy.time) + scores.back();

//            if (c->getRudder() == 0 && c->getThrottle() == 1) cerr << "Score: " << s / n << endl;
//            cerr << "Trying ";
//            for (int k = 0; k <= i; k++) cerr << controls[k].getRudder() << ", " << controls[k].getThrottle() << "; ";
//            cerr << "at depth " << i << " resulting in state " << newState.toString() << " with score " << s / n << endl;
            // If we're not at a leaf in the control tree, push the state and score and increment the trajectory index
            if (i < n - 1) {
                if (s / n <= minScore) { // prune off bad iterations
                    futureStates.push_back(newState);
                    scores.push_back(s);
                    simulatedStates.push_back(simulated);
                    i++;
                } else {
                    c->incrementRudder();
                }
            }
            // otherwise, check the score. If we've set a new low score, set the actual controls and the states for
            // estimating current
            else {
                if (s / n < minScore) {
                    minScore = s / n;
//                    r = controls.front().getRudder();
//                    t = controls.front().getThrottle();
//                    cerr << "New min score " << minScore << " with " << r << ", " << t << endl;
//                    cerr << i + 1 << " states on the reference trajectory considered" << endl;
                    future.clear();
                    // store all this iteration's future states in future (skip the start state)
//                    for (unsigned long k = 0; k < futureStates.size(); k++) future.push_back(futureStates[k]);
                    futureControls.clear();
//                    simulatedStates.push_back(simulated); // didn't do it before, less code to do it here than manually add
                    for (unsigned long k = 0; k < simulatedStates.size(); k++) {
                        for (const auto& state : simulatedStates[k]) {
                            future.push_back(state);
                            futureControls.emplace_back(c->getRudder(), c->getThrottle());
                        }
                    }
                    for (const auto& state : simulated) {
                        future.push_back(state);
                        futureControls.emplace_back(controls.back().getRudder(), controls.back().getThrottle());
                    }
                    // the most recent state wasn't in futureStates, so push it too
//                    future.push_back(newState);
//                    for (auto c : controls) futureControls.emplace_back(c.getRudder(), c.getThrottle());
                }
                // Increment the rudder because we're finished with that control pair at this depth
                c->incrementRudder();
            }
        }
        m_FutureStuffMutex.lock();
        m_PredictedTrajectory = future;
//        cerr << "Controller picked a trajectory of length " << m_PredictedTrajectory.size() << ": " << endl;
//        for (auto s : m_PredictedTrajectory) cerr << s.toString() << endl;
//        cerr << "With controls : ";
//        for (auto c1 : futureControls) cerr << c1.first << ", " << c1.second << "; ";
//        cerr << endl;
//        cerr << "And score " << minScore << endl;
        r = futureControls.front().first;
        t = futureControls.front().second;
        m_FutureControls = futureControls;
        m_TrajectoryNumber = trajectoryNumber;
        m_FutureStuffMutex.unlock();
    }
//    cerr << "Managed " << iterations << " iterations of MPC" << endl;
}

void Controller::mpc3(double& r, double& t, State startCopy, std::vector<State> referenceTrajectoryCopy, double endTime, long trajectoryNumber)
{
    // TODO! -- write a similar method to do MPC the first time when we need to find a state 1s in the future
    // also write something to kick this off in a new thread for a few iterations (see notes)
    // should be able to make reference trajectory copy a const reference. could help performance
    // TODO! -- should simulate current state with last issued controls to determine starting point for this iteration
    // TODO! -- update current estimator

    // TODO! -- acquire current estimate from updated estimator
    std::pair<double, double> currentEstimate = std::make_pair(0.0, 0.0);

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
    State lastInterpolated(-1); // cache last interpolated state on reference trajectory
    while (m_ControlReceiver->getTime() < endTime) {
        // cut out when the reference trajectory is updated
        if (!validTrajectoryNumber(trajectoryNumber)) return;
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
            std::cerr << "Assigned r and t" << std::endl;
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
        if (lastInterpolated.time != s.State.time) {
            lastInterpolated = interpolateTo(s.State.time, referenceTrajectoryCopy);
        }
        auto score = compareStates(lastInterpolated, s.State) * weight + s.PrevScore;
        if (s.Depth > 0) {
            if (score < minScore) {
                bestCurrentRudder = s.InitialRudder;
                bestCurrentThrottle = s.InitialThrottle;
                minScore = score;
                std::cerr << "Found new best trajectory with initial controls " << bestCurrentRudder << ", " << bestCurrentThrottle << std::endl;
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

    std::cerr << "Managed " << iterations << " iterations of limited-branching MPC (" << c_ScoringTimeStep * iterations
        << " seconds in the future)" << std::endl;

    assert(std::isfinite(r) && std::isfinite(t));
}

void Controller::sendAction()
{
    double rudder, throttle;
    while (running)
    {
        if (plan) //use mutex instead of busy waiting // you mean CV?
        {
            // TODO! -- support error cases, find out how to trigger e-stop

            // grab current starting position and reference trajectory
//            cerr << "Controller getting ready to copy trajectory" << endl;
            mtx.lock();
//            cerr << "Controller copying trajectory of length " << m_ReferenceTrajectory.size() << endl;
            State startCopy(m_CurrentLocation);
            vector<State> referenceTrajectoryCopy;
            double lookahead = m_UseBranching? 30 : 6;
            for (const auto& s : m_ReferenceTrajectory)
                if (s.time > m_ControlReceiver->getTime() + 1 &&
                        (referenceTrajectoryCopy.empty() ||  s.time < m_ControlReceiver->getTime() + lookahead))
                    referenceTrajectoryCopy.push_back(s);
//            cerr << "Resulting copied trajectory of length " << referenceTrajectoryCopy.size() << endl;
            if (referenceTrajectoryCopy.empty()) {
//                cerr << "Reference trajectory empty; sleeping for a bit" << endl;
                this_thread::sleep_for(std::chrono::milliseconds(100));
                mtx.unlock();
                continue;
            }
            auto trajectoryNumber = m_NextTrajectoryNumber;
            mtx.unlock();
//            cerr << "Starting control iteration from " << startCopy.toString() << endl;
            // actually do MPC
            if (m_UseBranching) {
                mpc(rudder, throttle, startCopy, referenceTrajectoryCopy, m_ControlReceiver->getTime() + 0.1, trajectoryNumber);
            } else {
                straightMpc(rudder, throttle, startCopy, referenceTrajectoryCopy, m_ControlReceiver->getTime() + 0.1, trajectoryNumber);
            }
            // add control to current estimator
//            m_CurrentEstimator.addControl(rudder, throttle, getTime());
//            cerr << "Controller picked a trajectory of length " << m_PredictedTrajectory.size() << endl;
//            cerr << "with initial rudder " << rudder << " and throttle " << throttle << endl;
            std::vector<State> trajectory;
            for (const auto& v : m_PredictedTrajectory) trajectory.push_back(v);
            if (m_ControlReceiver) m_ControlReceiver->displayTrajectory(trajectory, false);
            else cerr << "Did not display trajectory of length " << m_PredictedTrajectory.size() << endl;
//                cerr << "rudder: " << rudder << endl;
            m_ControlReceiver->receiveControl(rudder, throttle);
        } else {
            this_thread::sleep_for(std::chrono::milliseconds(100));
        }
//        cerr << "Current: " <<  m_CurrentEstimator.getCurrent().first << ", " << m_CurrentEstimator.getCurrent().second << endl;

    }
    cerr << "Ending thread for MPC" << endl;
}

void Controller::startRunning()
{
    if (running) return;
    running = true;
    cerr << "Starting controller" << endl;
    cerr << "Starting thread for MPC" << endl;
    thread thread_for_mpc([=]{ sendAction(); });
    thread_for_mpc.detach();
    cerr << "Done starting controller" << endl;
}

void Controller::terminate()
{
    running = false;
    {
        std::unique_lock<mutex> lock(m_TrajectoryNumberMutex);
        m_TrajectoryNumber = -1;
    }
}

void Controller::startSendingControls()
{
    m_CurrentEstimator.resetCurrentEstimate();
//    mtx.lock();
//    VehicleState startCopy(m_CurrentLocation);
//    mtx.unlock();
//    m_CurrentEstimator.initialize(startCopy);
    plan = true;
}

void Controller::stopSendingControls()
{
    plan = false;
}

double Controller::getMPCWeight(int index) {
//    return 1;
    return MAX_LOOKAHEAD_STEPS + index;
}

State Controller::estimateStateInFuture(double desiredTime, long& trajectoryNumber) {
//    cerr << "Estimating state " << desiredTime - m_ControlReceiver->getTime() << "s in the future" << endl;
//    cerr << "Current location is " << m_CurrentLocation.toString() << endl;
//    cerr << m_FutureControls.size() << endl;
    VehicleState s = State(-1);
    VehicleState prev = State(-1);
    double r, t;
    m_FutureStuffMutex.lock();
    trajectoryNumber = m_TrajectoryNumber;
//    assert(m_PredictedTrajectory.size() - 1 == m_FutureControls.size());
    if (m_FutureControls.empty()) {
        cerr << "Error: cannot estimate a state in the future because future controls are unknown." << endl;
        m_FutureStuffMutex.unlock();
        return s;
    }
    // find furthest future state earlier than desiredTime
    for (int i = 0; i < m_PredictedTrajectory.size(); i++) {
//        cerr << "Checking state " << m_PredictedTrajectory.at(i).toString() << endl;
        if (m_PredictedTrajectory.at(i).time >= desiredTime) {
            // grab previous state
            if (i != 0) {
                s = m_PredictedTrajectory.at(i - 1);
                r = m_FutureControls.at(i - 1).first;
                t = m_FutureControls.at(i - 1).second;
//                cerr << "Picked state " << s.toString() << " to estimate from" << endl;
                break;
            } else {
                cerr << "Error: desired time in the past for entire predicted trajectory." << endl;
                m_FutureStuffMutex.unlock();
                return s;
            }
        }
        prev = s;
    }
    // if we didn't assign in that loop it was the last one
    if (s.time == -1 && !m_PredictedTrajectory.empty()) {
        s = m_PredictedTrajectory.back();
        r = m_FutureControls.back().first;
        t = m_FutureControls.back().second;
//        cerr << "Picked state " << s.toString() << " to estimate from" << endl;
    }
    m_FutureStuffMutex.unlock();
//    cerr << "Picked state " << s.toString() << " to estimate from" << endl;
    // return an estimate at the desired time based on the controls
    vector<VehicleState> simulated;
//    auto ret = s.simulate(r, t, desiredTime - s.time, m_CurrentEstimator.getCurrentDirection(),
//                          m_CurrentEstimator.getCurrentSpeed(), simulated);
    auto ret = s.simulate(r, t, desiredTime - s.time, m_CurrentEstimator.getCurrent(), simulated);
//    // set the heading to ignore current (what we in the business call a *hack*)
//    auto last = simulated.back();
//    last.setHeadingTowards(ret);
//    prev.setHeadingTowards(ret);
//    cerr << "Changing ret.heading from " << ret.heading << " to " << prev.heading << endl;
//    ret.heading = prev.heading;
//    cerr << ret.heading;

    // finding net heading (accounting for current) // WRONG
    // but current estimator is also wrong so need to fix that

    auto dx = ret.speed * cos(ret.yaw()) + m_CurrentEstimator.getCurrent().first;
    auto dy = ret.speed * sin(ret.yaw()) + m_CurrentEstimator.getCurrent().second;

//    auto deltaEV = m_CurrentEstimator.getCurrentSpeed();
//
//    auto dx = ret.speed * cos(ret.yaw()) + deltaEV * sin(m_CurrentEstimator.getCurrentDirection());
//    auto dy = ret.speed * sin(ret.yaw()) + deltaEV * cos(m_CurrentEstimator.getCurrentDirection());
//

    ret.heading = M_PI_2 - atan2(dy, dx);
//    cerr << "Estimated state " << ret.toString() << "\n Which is " << s.distanceTo(ret) << " meters away from that one" << endl;
//    cerr << "with controls r = " << r << ", t = " << t << " and current " <<
//        m_CurrentEstimator.getCurrent().first << ", " << m_CurrentEstimator.getCurrent().second << endl;
    return ret;
}

void Controller::updatePosition(State state) {
    mtx.lock();
    m_CurrentLocation = state;
    mtx.unlock();
}

State Controller::estimateStateInFuture(double desiredTime) {
    long l;
    return estimateStateInFuture(desiredTime, l);
}

void Controller::straightMpc(double& r, double& t, State startCopy, std::vector<State> referenceTrajectoryCopy,
                             double endTime, long trajectoryNumber) {

    r = 0; t = 0;
    if (referenceTrajectoryCopy.empty()) {
//        cerr << "Called MPC with empty reference trajectory. Returning 0, 0" << endl;
        return;
    }

    // update current estimator // why here though?
    m_FutureStuffMutex.lock();
//    m_CurrentEstimator.updateEstimate2(startCopy, m_PredictedTrajectory);
    m_CurrentEstimator.updateEstimate(startCopy, m_PredictedTrajectory);
//    m_CurrentEstimator.update(startCopy);
    m_FutureStuffMutex.unlock();

    // rudders on both sides of zero, and throttles count zero
    int rudderGranularity = m_Rudders / 2, throttleGranularity = m_Throttles - 1;

    VehicleState start(startCopy);

    std::vector<VehicleState> simulated;
    int iterations;
    auto minScore = DBL_MAX;
    for (iterations = 1;  true; iterations++) {
        for (int rudderNum = -rudderGranularity; rudderNum <= rudderGranularity; rudderNum++) {
            double rudder = ((double)rudderNum) / rudderGranularity;
            for (int throttleNum = 0; throttleNum <= throttleGranularity; throttleNum++) {
                double throttle = ((double)throttleNum) / throttleGranularity;
                VehicleState currentState = start;
                simulated.clear();
                double score = 0;
                for (int i = 0; i < referenceTrajectoryCopy.size(); i++) {
                    const auto& reference = referenceTrajectoryCopy[i];
//                    currentState = currentState.simulate(rudder, throttle, reference.time - currentState.time,
//                            m_CurrentEstimator.getCurrentDirection(),
//                            m_CurrentEstimator.getCurrentSpeed(), simulated);
                    currentState = currentState.simulate(rudder, throttle, reference.time - currentState.time, m_CurrentEstimator.getCurrent(), simulated);
                    score += compareStates(reference, currentState) * getMPCWeight(currentState.time - startCopy.time);
//                    score += reference.getDistanceScore(currentState) * getMPCWeight(currentState.time - startCopy.time);
                }
                if (score < minScore) {
                    minScore = score;
                    r = rudder; t = throttle;
//                    cerr << "Picked controls " << rudder << ", " << throttle << " with score " << score << endl;
                    m_FutureStuffMutex.lock();
                    m_FutureControls = vector<pair<double, double>>(simulated.size() - 1, make_pair(rudder, throttle));
                    m_PredictedTrajectory = simulated;
                    m_TrajectoryNumber = trajectoryNumber;
                    m_FutureStuffMutex.unlock();
                }
                if (m_ControlReceiver->getTime() >= endTime) {
//                    cerr << "Managed " << iterations << " iterations of straight MPC" << endl;
                    return;
                }
//                else { cerr << "Not done yet\n"; }
            }
        }
        rudderGranularity *= 2;
        throttleGranularity *= 2;
    }

}

void Controller::updateConfig(int useBranching, double weightSlope, double weightStart, int rudders, int throttles,
                              double distanceWeight, double headingWeight, double speedWeight) {
    m_UseBranching = useBranching;
    m_WeightSlope = weightSlope; m_WeightStart = weightStart;
    m_Rudders = rudders; m_Throttles = throttles;
    m_DistanceWeight = distanceWeight; m_HeadingWeight = headingWeight; m_SpeedWeight = speedWeight;
}

double Controller::getMPCWeight(double timeFromStart) const {
    return m_WeightStart + timeFromStart * m_WeightSlope;
}

double Controller::compareStates(const State& s1, const VehicleState& s2) const {
    // ignore differences in time
    double headingDiff = fabs(fmod((s1.heading - s2.heading), 2 * M_PI));
    auto speedDiff = fabs(s1.speed - s2.speed);
    auto dx = s1.x - s2.x;
    auto dy = s1.y - s2.y;
    auto d = sqrt(dx * dx + dy * dy);
    return m_DistanceWeight * d + m_HeadingWeight * headingDiff * m_SpeedWeight * speedDiff;
}

State Controller::interpolateTo(double desiredTime, const std::vector<State>& trajectory) {
    if (trajectory.size() < 2) throw std::logic_error("Cannot interpolate on a trajectory with fewer than 2 states");
    int i = 1;
    for (; i < trajectory.size() && trajectory[i].time < desiredTime; i++) ;
    if (trajectory[i].time < desiredTime) std::cerr << "Warning: extrapolating instead of interpolating" << std::endl;
    return trajectory[i - 1].interpolate(trajectory[i], desiredTime);
}

State Controller::interpolateTo(double desiredTime, std::list<State>& trajectory) {
    if (trajectory.size() < 2) throw std::logic_error("Cannot interpolate on a trajectory with fewer than 2 states");
    auto it = trajectory.begin()++;
    for (; it != trajectory.end() && it->time < desiredTime; it++) ;
    auto s2 = *(it);
    auto s1 = *(it--);
    auto result = s1.interpolate(s2, desiredTime);
    if ((it++)->time < desiredTime){
        std::cerr << "Warning: extrapolating instead of interpolating" << std::endl;
        trajectory.insert(it++, result);
    } else {
        // interpolating, so *it is a state later than desiredTime
        trajectory.insert(it, result);
    }
    return result;
}

bool Controller::sendControl(double rudder, double throttle, long trajectoryNumber) {
    std::unique_lock<mutex> lock1(m_TrajectoryNumberMutex);
    if (trajectoryNumber != m_TrajectoryNumber) return false;
    m_ControlReceiver->receiveControl(rudder, throttle);
    return true;
}

bool Controller::validTrajectoryNumber(long trajectoryNumber) {
    std::unique_lock<mutex> lock1(m_TrajectoryNumberMutex);
    return trajectoryNumber == m_TrajectoryNumber;
}

State Controller::mpc4(double& r, double& t, State startCopy, std::vector<State> referenceTrajectoryCopy,
                       double endTime) {

    static_assert(c_ScoringTimeStep == 0.5, "This method makes an assumption about the scoring time step");

    // TODO! -- update current estimator
    // TODO! -- acquire current estimate from updated estimator
    std::pair<double, double> currentEstimate = std::make_pair(0.0, 0.0);

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
            State(-1),
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
    State lastInterpolated(-1); // cache last interpolated state on reference trajectory
    State result, intermediateResult; // state one second in the future
    while (m_ControlReceiver->getTime() < endTime) {
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
        double weight = 1.0; //s.State.time < referenceTrajectoryCopy[1].time ? 1.0 : 0.1;
        if (lastInterpolated.time != s.State.time) {
            lastInterpolated = interpolateTo(s.State.time, referenceTrajectoryCopy);
        }
        auto score = compareStates(lastInterpolated, s.State) * weight + s.PrevScore;
        assert(score >= 0 && "Score should be non-negative");
        if (s.Depth > 0) {
            if (score < minScore) {
                bestCurrentRudder = s.InitialRudder;
                bestCurrentThrottle = s.InitialThrottle;
                minScore = score;
                intermediateResult = s.OneSecondOut;
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
                auto oneSecondOut = iterations < 2? VehicleState(State(-1)) :
                        iterations == 2? s.State : s.OneSecondOut;
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

    std::cerr << "Managed " << iterations << " iterations of limited-branching MPC (" << c_ScoringTimeStep * iterations
              << " seconds in the future)" << std::endl;

    assert(std::isfinite(r) && std::isfinite(t));

    return result;
}

State Controller::updateReferenceTrajectory(std::vector<State>& trajectory, long trajectoryNumber) {
    if (trajectory.size() < 2) {
        // Not long enough for MPC. Balk out.
        // NOTE: previous MPC thread may still be running; that can time out on its own
        std::cerr << "Reference trajectory of length " << trajectory.size() << " too short for MPC. Reference trajectory will not be updated." << std::endl;
        return State(-1);
    }
    // new scope to use RAII
    {
        std::unique_lock<mutex> lock(m_TrajectoryNumberMutex); // TODO! -- check to make sure this is enforced elsewhere
        // updating the trajectory number stops the previous MPC thread
        m_TrajectoryNumber = trajectoryNumber;
    }

    auto start = getStateAfterCurrentControl();

    double r, t;
    auto result = mpc4(r, t, start, trajectory, getTime() + c_PlanningTime);

    // kick off the new MPC thread
    auto mpcThread = thread([=, &trajectory]{ // doesn't copy reference trajectory
        // Set updated start state and the state we're passing on to the planner in appropriate spots in the reference trajectory
        int startIndex = -1, goalIndex = -1;
        for (int i = 0; i < trajectory.size(); i++) {
            if (start.time >= trajectory[i].time) continue;
            else if (startIndex == -1){
                startIndex = i - 1;
                trajectory[startIndex] = start;
            }
            if (result.time < trajectory[i].time && goalIndex == -1) {
                goalIndex = i - 1;
                trajectory[goalIndex] = result;
            }
            if (goalIndex != -1 && startIndex != -1) break;
        }
        auto endTime = getTime() + c_ReferenceTrajectoryExpirationTime;
        double r, t;
        while (getTime() < endTime) {
            if (!validTrajectoryNumber(trajectoryNumber)) break;
            auto stateAfterCurrentControl = getStateAfterCurrentControl();
            for (int i = startIndex; i < trajectory.size(); i++) {
                if (stateAfterCurrentControl.time < trajectory[i].time) {
                    if (i - 1 != goalIndex) { // don't overwrite the state we gave to the planner
                        startIndex = i - 1;
                        trajectory[startIndex] = stateAfterCurrentControl;
                    }
                    break;
                }
            }
            mpc4(r, t, stateAfterCurrentControl, trajectory, getTime() + c_PlanningTime);
            sendControls(r, t);
        }
        if (getTime() >= endTime) {
            std::cerr << "Controller's reference trajectory appears to have timed out. No more controls will be issued" << std::endl;
        }
    });
    mpcThread.detach();

    sendControls(r, t);

    return result;
}

VehicleState Controller::getStateAfterCurrentControl() {
    mtx.lock();
    State startCopy(m_CurrentLocation);
    mtx.unlock();

    // TODO! -- figure out concurrency issues here - last controls should be protected somehow
    VehicleState start(startCopy);
    auto current = m_CurrentEstimator.getCurrent();
    // simulate last issued control to how long it will take us to compute the next one
    start.simulate(m_LastRudder, m_LastThrottle, c_PlanningTime, current);
    return start;
}

void Controller::sendControls(double r, double t) {
    // TODO! -- concurrency safety?
    m_LastRudder = r;
    m_LastThrottle = t;
    m_ControlReceiver->receiveControl(r, t);
}



