#include <iostream>
#include <fstream>
//#include <iomanip>
#include <limits>
#include <chrono>
#include <thread>
#include <mutex>
#include <queue>
#include <stack>
#include <string>
#include <string.h>
#include <sstream>
#include <cmath>
//#include "State.h"
//#include "VehicleState.h"
#include <cfloat>
#include <functional>
#include <path_planner/Trajectory.h>
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

void Controller::receiveRequest(const path_planner::Trajectory::ConstPtr& trajectory)
{
    mtx.lock();
    start.set(trajectory->states[0]);
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

    m_CurrentEstimator.updateEstimate(startCopy);

//    double duration = 0.05;
//    auto score = DBL_MAX; // higher score => more distance
//    int count = 0;
//    while (getTime() < endTime) {
//        // set rudder and throttle randomly with a chance to use special values for each
//        double throttle = distribution(randomEngine);
//        if (throttle > 0.95) throttle = 0;
//        else if (throttle > 0.85) throttle = 1.0;
//        else throttle = distribution(randomEngine);
//        double rudder = distribution(randomEngine);
//        if (rudder > 0.95) rudder = 1.0;
//        else if (rudder > 0.9) rudder = 0;
//        else if (rudder > 0.85) rudder = -1;
//        else rudder = (2 * distribution(randomEngine)) - 1;
//
//        double x1 = startCopy.x, y1 = startCopy.y, heading1 = startCopy.heading, speed1 = startCopy.speed, starttime = 0, rpm1 = rpm, d_time, temp = 0;
//        vector<State> tempfuture;
//        for (int index = 0; index < referenceTrajectoryCopy.size(); index++)
//        {
//            d_time = referenceTrajectoryCopy[index].otime - startCopy.otime;
//            if(d_time < 0)
//                continue;
//            while (starttime + duration < d_time)
//            {
//                starttime += duration;
//                estimate(rudder, throttle, duration, x1, y1, rpm1, speed1, heading1);
//                if (tempfuture.size() < 10)
//                {
//                    tempfuture.emplace_back(x1, y1, heading1, speed1, startCopy.otime + starttime);
//                }
//            }
//            if (starttime != d_time)
//            {
//                estimate(rudder, throttle, d_time - starttime, x1, y1, rpm1, speed1, heading1);
//            }
//            temp += (pow(x1 - referenceTrajectoryCopy[index].x, 2) +
//                    pow(y1 - referenceTrajectoryCopy[index].y, 2)) * getMPCWeight(index);
//        }
//        if (score > temp)
//        {
//            r = (int)(rudder * 1000.0) / 1000.0;
//            t = (int)(throttle * 1000.0) / 1000.0;
//            score = temp;
//            future = tempfuture;
//        }
//        count++;
//    }

    // timed loop
//    int iterations;
//    auto minScore = DBL_MAX;
//    for (iterations = 1; getTime() < endTime; iterations++) {
//        vector<State> tempFuture(iterations, State());
////        cerr << iterations;
//        double duration = (referenceTrajectoryCopy.back().otime - startCopy.otime) / iterations;
//        double s = score(referenceTrajectoryCopy, tempFuture, VehicleState(startCopy, rpm), duration, minScore, iterations, endTime, r, t) / iterations;
//        if (s < minScore) {
//            minScore = s;
//            future = tempFuture;
//        }
//    }
//    cerr << minScore << endl;

//    future = referenceTrajectoryCopy; // ahhhh

    vector<Control> controls;
    vector<VehicleState> futureStates;
    vector<double> scores;
    int controlGranularity = 10; // 20 rudders, 20 throttles
    int iterations;
    auto minScore = DBL_MAX;
    for (iterations = 1;  getTime() < endTime; iterations++) {
        bool filledTrajectory = iterations >= referenceTrajectoryCopy.size();
        if (filledTrajectory) controlGranularity += controlGranularity;
        int n = filledTrajectory ? referenceTrajectoryCopy.size() : iterations;
        controls = vector<Control>(n, Control(controlGranularity, controlGranularity));
        futureStates.clear();
        futureStates.emplace_back(startCopy);
        scores.clear();
        scores.push_back(0);
        int i = 0;
        // i represents the index we are in the reference trajectory (and the controls vector). It could also be used to
        // index futureStates and scores, but those should be able to use back() instead, and conceptually that
        // makes a little more sense to me, as I'm treating them as stacks.
        while (i >= 0) {
            if (getTime() >= endTime) break; // let's be as real-time as possible
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
            // Score the current state
            double timeDiff = referenceTrajectoryCopy[i].otime - futureStates.back().otime; // back() should be same as [i]
            assert(timeDiff >= 0);
            auto newState = futureStates.back().estimate(c->getRudder(), c->getThrottle(), timeDiff, m_CurrentEstimator.getCurrent());
            double s = referenceTrajectoryCopy[i].getDistanceScore(newState) * getMPCWeight(i) + scores.back();
            // If we're not at a leaf in the control tree, push the state and score and increment the trajectory index
            if (i < n - 1) {
                if (s / i <= minScore) { // short circuit bad iterations
                    futureStates.push_back(newState);
                    scores.push_back(s);
                    i++;
                }
            }
            // otherwise, check the score. If we've set a new low score, set the actual controls and the states for
            // estimating current
            else {
                if (s / n < minScore) {
                    minScore = s;
                    r = (int)(controls.front().getRudder() * 1000.0) / 1000.0;
                    t = (int)(controls.front().getThrottle() * 1000.0) / 1000.0;
                    vector<State> future;
                    for (auto a : futureStates) future.push_back(a);
                    m_CurrentEstimator.updatePredictedTrajectory(future);
                }
            }
            // Increment the rudder because we're finished with that control pair at this depth
            c->incrementRudder();
        }
    }

//    cerr << future.front().x << " " << future.front().y << " " << future.front().otime << endl;
    cerr << "Managed " << iterations << " iterations of MPC" << endl;
}

// always setting controls to be the last ones?
double Controller::score(const vector<State>& referenceTrajectoryCopy, vector<State>& futureStates, VehicleState state, double timeStep, double minScore, int iterations, double endTime, double &r, double &t)
{
    if (iterations < 1) return 0;
//    cerr << "getTime: " << getTime() << " endTime: " << endTime << endl;
     for (int i = -50; i <= 50; i++) {
         double rudder = i / 50.0;
         for (int j = 10; j >= 0; j--) {
             if (getTime() >= endTime) return minScore;
             double throttle = j / 10.0;
             VehicleState newState = state.estimate(rudder, throttle, timeStep, m_CurrentEstimator.getCurrent());
             // score at closest point on reference trajectory
             auto scoreTemp = 0.0; // DBL_MAX;
             for (int index = 0; index < referenceTrajectoryCopy.size(); index++) {
                 if (fabs(referenceTrajectoryCopy[index].otime - newState.otime) < 1.0) {
                     double s = referenceTrajectoryCopy[index].getDistanceScore(newState) * getMPCWeight(index);
//                 if (s < scoreTemp) scoreTemp = s;
                     scoreTemp += s;
                 }
             }

             if (scoreTemp + scoreTemp > minScore) continue; // short circuit bad iterations

             scoreTemp += score(referenceTrajectoryCopy, futureStates, newState, timeStep, minScore, iterations - 1, endTime, r, t);
//             newState.score += scoreTemp;
             if (scoreTemp < minScore) {
//                 cerr << "Got score " << newState.score << endl;
                 minScore = scoreTemp;
                 r = (int) (rudder * 1000.0) / 1000.0;
                 t = (int) (throttle * 1000.0) / 1000.0;
                 // update future states for current estimation
                 futureStates[futureStates.size() - iterations] = newState;
             }
         }
     }
     return minScore;
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
            MPC(rudder, throttle, startCopy, referenceTrajectoryCopy, getTime() + 0.1);
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
    return 2 * MAX_LOOKAHEAD_STEPS - index;
}



