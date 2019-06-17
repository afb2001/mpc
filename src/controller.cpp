#include <thread>
#include <path_planner/Trajectory.h>
#include <path_planner/TrajectoryDisplayer.h>
#include "controller.h"

using namespace std;

Controller::Controller(ControlReceiver *controlReceiver, TrajectoryDisplayer *trajectoryDisplayer) {
    m_ControlReceiver = controlReceiver;
    m_TrajectoryDisplayer = trajectoryDisplayer;
}

Controller::~Controller() = default;

double Controller::getTime()
{
    struct timespec t{};
    clock_gettime(CLOCK_REALTIME, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}

void Controller::receiveRequest(const path_planner::Trajectory::ConstPtr& trajectory)
{
    mtx.lock();
//    start.set(trajectory->states[0]);
    referenceTrajectory.clear();
    for (int i = 1; i < trajectory->states.size() && i < MAX_LOOKAHEAD_STEPS; i++)
    {
        referenceTrajectory.emplace_back(State(trajectory->states[i]));
    }
    mtx.unlock();
}

void Controller::MPC(double &r, double &t, State startCopy, vector<State> referenceTrajectoryCopy, double endTime)
{
    if (referenceTrajectoryCopy.empty())
        return;

    m_FutureStuffMutex.lock();
    m_CurrentEstimator.updateEstimate(startCopy, m_PredictedTrajectory);
    m_FutureStuffMutex.unlock();

    vector<Control> controls;
    vector<VehicleState> futureStates;
    vector<pair<double,double>> futureControls;
    vector<State> future;
    vector<double> scores;
    int rudderGranularity = 5, throttleGranularity = 4; // 10 rudders, 4 throttles
    int iterations;
    auto minScore = DBL_MAX;
    for (iterations = 1;  getTime() < endTime; iterations++) {
        cerr << "Last iteration min score was " << minScore << endl;
        minScore = DBL_MAX;
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
                continue;
            }
            cerr << "Trying ";
            for (int k = 0; k <= i; k++) cerr << controls[k].getRudder() << ", " << controls[k].getThrottle() << "; ";
            cerr << "at depth " << i << endl;
            // Score the current state
            double timeDiff = referenceTrajectoryCopy[i].time - futureStates[i].time; // back() should be same as [i]
            assert(timeDiff >= 0);
            auto newState = futureStates[i].estimate(c->getRudder(), c->getThrottle(), timeDiff, m_CurrentEstimator.getCurrent());
            double s = referenceTrajectoryCopy[i].getDistanceScore(newState) * getMPCWeight(i) + scores[i];
//            if (c->getRudder() == 0 && c->getThrottle() == 1) cerr << "Score: " << s << endl;
            // If we're not at a leaf in the control tree, push the state and score and increment the trajectory index
            if (i < n - 1) {
                if (s / n <= minScore) { // prune off bad iterations
                    futureStates.push_back(newState);
                    scores.push_back(s);
                    i++;
                }
            }
            // otherwise, check the score. If we've set a new low score, set the actual controls and the states for
            // estimating current
            else {
                if (s / n < minScore) {
                    minScore = s / n;
                    r = (int)(controls.front().getRudder() * 1000.0) / 1000.0;
                    t = (int)(controls.front().getThrottle() * 1000.0) / 1000.0;
//                    cerr << "New min score " << minScore << " with " << r << ", " << t << endl;
//                    cerr << i + 1 << " states on the reference trajectory considered" << endl;
                    future.clear();
                    for (auto a : futureStates) future.push_back(a);
                    futureControls.clear();
                    for (auto c : controls) futureControls.emplace_back(c.getRudder(), c.getThrottle());
                }
            }
            // Increment the rudder because we're finished with that control pair at this depth
            c->incrementRudder();
        }
        m_FutureStuffMutex.lock();
        m_PredictedTrajectory = future;
        cerr << "Controller picked a trajectory of length " << m_PredictedTrajectory.size() << endl;
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
            startCopy.set(start);
            vector<State> referenceTrajectoryCopy(referenceTrajectory);
            mtx.unlock();
            // actually do MPC
            MPC(rudder, throttle, startCopy, referenceTrajectoryCopy, getTime() + 0.25);
            cerr << "Controller picked a trajectory of length " << m_PredictedTrajectory.size() << endl;
            if (m_ControlReceiver) m_TrajectoryDisplayer->displayTrajectory(m_PredictedTrajectory, false);
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
    VehicleState s = State(-1);
    double r, t;
    m_FutureStuffMutex.lock();
    assert(m_PredictedTrajectory.empty() == m_FutureControls.empty());
    if (m_FutureControls.empty()) {
        cerr << "Error: cannot estimate a state in the future because future controls are unknown." << endl;
        m_FutureStuffMutex.unlock();
        return s;
    }
    // find furthest future state earlier than desiredTime
    for (int i = 0; i < m_PredictedTrajectory.size(); i++) {
        if (m_PredictedTrajectory.at(i).time >= desiredTime) {
            // grab previous state
            if (i != 0) {
                s = m_PredictedTrajectory.at(i - 1);
                r = m_FutureControls.at(i - 1).first;
                t = m_FutureControls.at(i - 1).second;
            } else {
                cerr << "Error: desired time in the past for entire predicted trajectory." << endl;
                m_FutureStuffMutex.unlock();
                return s;
            }
            break;
        }
    }
    // if we didn't assign in that loop it was the last one
    if (s.time == -1 && !m_PredictedTrajectory.empty()) {
        s = m_PredictedTrajectory.back();
        r = m_FutureControls.back().first;
        t = m_FutureControls.back().second;
    }
    m_FutureStuffMutex.unlock();
    // take return an estimate at the desired time based on the controls
    return s.estimate(r, t, desiredTime - s.time, m_CurrentEstimator.getCurrent());
}

void Controller::updatePosition(State state) {
    mtx.lock();
    start = state;
    mtx.unlock();
}



