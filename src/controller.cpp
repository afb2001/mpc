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

void Controller::receiveRequest(const std::vector<State>& trajectory)
{
    mtx.lock();
//    start.set(trajectory->states[0]);
//    m_ReferenceTrajectory.clear();
//    for (int i = 1; i < trajectory->states.size() && i < MAX_LOOKAHEAD_STEPS; i++)
//    {
//        m_ReferenceTrajectory.emplace_back(State(trajectory->states[i]));
//    }
    m_ReferenceTrajectory = trajectory;
    mtx.unlock();
}

void Controller::mpc(double& r, double& t, State startCopy, vector<State> referenceTrajectoryCopy, double endTime)
{
    if (referenceTrajectoryCopy.empty())
        return;

    m_FutureStuffMutex.lock();
    m_CurrentEstimator.updateEstimate(startCopy, m_PredictedTrajectory);
    m_FutureStuffMutex.unlock();

//    cerr << "Starting MPC from " << startCopy.toString() << endl;

    vector<Control> controls;
    vector<VehicleState> futureStates;
    vector<pair<double,double>> futureControls;
    vector<VehicleState> future;
    vector<double> scores;
    int rudderGranularity = 5, throttleGranularity = 4; // 10 rudders, 4 throttles
    int iterations;
    auto minScore = DBL_MAX;
    for (iterations = 1;  getTime() < endTime; iterations++) {
//        cerr << "Last iteration min score was " << minScore << endl;
//        minScore = DBL_MAX;
        bool filledTrajectory = iterations > referenceTrajectoryCopy.size();
        if (filledTrajectory) {
            rudderGranularity += rudderGranularity;
            throttleGranularity += throttleGranularity;
        }
        int n = filledTrajectory ? referenceTrajectoryCopy.size() : iterations;
        controls.clear();
        controls = vector<Control>(n, Control(rudderGranularity, throttleGranularity));
        futureStates.clear();
        futureStates.emplace_back(startCopy);
        scores.clear();
        scores.push_back(0);
        int i = 0;
        // i represents the index we are in the reference trajectory (and the controls vector). It could also be used to
        // index futureStates and scores, but those should be able to use back() instead, and conceptually that
        // makes a little more sense to me, as I'm treating them as stacks.
        while (i >= 0) {
//            cerr << "Depth = " << i << endl;
            if (getTime() >= endTime) return; // let's be as real-time as possible
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
                if (i >= 0) controls[i].incrementRudder();
                continue;
            }
            // generate the next state and score it
            double timeDiff = referenceTrajectoryCopy[i].time - futureStates.back().time; // back() should be same as [i]
            if(timeDiff < 0) {
                cerr << "Time difference between the following states is below zero:\n";
                cerr << referenceTrajectoryCopy[i].toString() << '\n' << futureStates.back().toString() << endl;
            }
            auto newState = futureStates.back().estimate(c->getRudder(), c->getThrottle(), timeDiff, m_CurrentEstimator.getCurrent());
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
                    for (int k = 1; k < futureStates.size(); k++) future.push_back(futureStates[k]);
                    // the most recent state wasn't in futureStates, so push it too
                    future.push_back(newState);
                    futureControls.clear();
                    for (auto c : controls) futureControls.emplace_back(c.getRudder(), c.getThrottle());
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
            State startCopy;
            startCopy.set(m_CurrentLocation);
            vector<State> referenceTrajectoryCopy;
            for (const auto& s : m_ReferenceTrajectory) if (s.time > m_CurrentLocation.time) referenceTrajectoryCopy.push_back(s);
            mtx.unlock();
            // actually do MPC
            mpc(rudder, throttle, startCopy, referenceTrajectoryCopy, getTime() + 0.25);
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
    return MAX_LOOKAHEAD_STEPS - index;
}

State Controller::estimateStateInFuture(double desiredTime) {
//    cerr << "Estimating state " << desiredTime - start.time << "s in the future" << endl;
//    cerr << m_FutureControls.size() << endl;
    VehicleState s = State(-1);
    double r, t;
    m_FutureStuffMutex.lock();
    assert(m_PredictedTrajectory.size() == m_FutureControls.size());
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
    auto ret = s.estimate(r, t, desiredTime - s.time, m_CurrentEstimator.getCurrent());
//    cerr << "Estimated state " << ret.toString() << "\n Which is " << s.distanceTo(ret) << " meters away from that one" << endl;
    return ret;
}

void Controller::updatePosition(State state) {
    mtx.lock();
    m_CurrentLocation = state;
    mtx.unlock();
}



