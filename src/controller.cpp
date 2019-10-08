#include <thread>
#include <path_planner/Trajectory.h>
#include <path_planner/TrajectoryDisplayer.h>
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
    m_CurrentEstimator.updateEstimate(startCopy, m_PredictedTrajectory);
    m_FutureStuffMutex.unlock();

//    cerr << "Starting MPC from " << startCopy.toString() << " with reference trajectory of length " << referenceTrajectoryCopy.size() << endl;

    r = 0; t = 0;
    vector<Control> controls;
    vector<VehicleState> futureStates;
    vector<pair<double,double>> futureControls;
    vector<VehicleState> future;
    vector<double> scores;
    vector<vector<VehicleState>> simulatedStates;
    int rudderGranularity = 5, throttleGranularity = 4; // 10 rudders, 4 throttles
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
            auto newState = futureStates.back().simulate(c->getRudder(), c->getThrottle(), timeDiff, m_CurrentEstimator.getCurrent(), simulated);
            double s = referenceTrajectoryCopy[i].getDistanceScore(newState) * getMPCWeight(i) + scores.back();
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

void Controller::sendAction()
{
    double rudder, throttle;
    while (running)
    {
        if (plan) //use mutex instead of busy waiting // you mean CV?
        {
            // TODO! -- support error cases, find out how to trigger e-stop

            // grab current starting position and reference trajectory
            mtx.lock();
            State startCopy(m_CurrentLocation);
            vector<State> referenceTrajectoryCopy;
            for (const auto& s : m_ReferenceTrajectory)
                if (s.time > m_ControlReceiver->getTime() + 1 && s.time < m_ControlReceiver->getTime() + 6)
                    referenceTrajectoryCopy.push_back(s);
            auto trajectoryNumber = m_NextTrajectoryNumber;
            mtx.unlock();
            // actually do MPC
            straightMpc(rudder, throttle, startCopy, referenceTrajectoryCopy, m_ControlReceiver->getTime() + 0.1, trajectoryNumber);
//            cerr << "Controller picked a trajectory of length " << m_PredictedTrajectory.size() << endl;
            std::vector<State> trajectory;
            for (const auto& v : m_PredictedTrajectory) trajectory.push_back(v);
            if (m_ControlReceiver) m_ControlReceiver->displayTrajectory(trajectory, false);
            else cerr << "Did not display trajectory of length " << m_PredictedTrajectory.size() << endl;
//                cerr << "rudder: " << rudder << endl;
            m_ControlReceiver->receiveControl(rudder, throttle);
        } else {
            this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        cerr << "Current: " <<  m_CurrentEstimator.getCurrent().first << ", " << m_CurrentEstimator.getCurrent().second << endl;

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
}

void Controller::startSendingControls()
{
    m_CurrentEstimator.resetCurrentEstimate();
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
    m_CurrentEstimator.updateEstimate(startCopy, m_PredictedTrajectory);
    m_FutureStuffMutex.unlock();

    int rudderGranularity = 5, throttleGranularity = 4; // 11 rudders, 5 throttles (counts zero)

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
                    currentState = currentState.simulate(rudder, throttle, reference.time - currentState.time, m_CurrentEstimator.getCurrent(), simulated);
                    score += reference.getDistanceScore(currentState) * (1.0 / (i + 1));
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



