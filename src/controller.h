#ifndef SRC_CONTROLLER_H
#define SRC_CONTROLLER_H

#include "control_receiver.h"
#include "path_planner/State.h"
#include <mutex>

using namespace std;

/**
 * Class which runs model predictive control and maintains everything required for it.
 * Model predictive control runs on its own thread. This class exposes member functions to update the current state of
 * the vehicle and the desired trajectory, but MPC can run regardless of when those are updated as long as it has been
 * started and not paused or terminated. If you want it to do anything useful, though, you need to at least update the
 * position.
 */
class Controller
{
public:

    /**
     * Construct a Controller. It needs an instance of a ControlReceiver to publish rudder and throttle
     * pairs, and a TrajectoryDisplayer to display trajectories.
     * @param controlReceiver interface which accepts rudders and throttles for publishing
     * @param trajectoryDisplayer interface which accepts trajectories for displaying
     */
    explicit Controller(ControlReceiver* controlReceiver);

    ~Controller();

    /**
     * Update the action request with a new trajectory.
     * @param actionRequest new reference trajectory
     */
    void receiveRequest(State* actionRequest);

    /**
     * Update the action request with a new trajectory.
     * @param trajectory new reference trajectory
     */
    void receiveRequest(const path_planner::Trajectory::ConstPtr& trajectory);

    /**
     * Starts a thread for doing MPC and issuing controls.
     */
    void startRunning();

    /**
     * Tell the MPC thread to stop running. This should be done when the controller will no longer be used.
     *
     */
    void terminate();

    /**
     * Set the plan flag. This lets the controller issue controls.
     * It should mean the reference trajectory is being updated.
     */
    void startSendingControls();

    /**
     * Clear than plan flag, stopping the controller from issuing controls.
     * Do this when the reference trajectory is no longer being updated.
     */
    void stopSendingControls();

private:

    ControlReceiver* m_ControlReceiver;

    //model parameter
    double idle_rpm = 0.0;
    double max_rpm = 3200.0;
    double max_rpm_change_rate = 1000.0;
    double prop_ratio = 0.389105058;
    double prop_pitch = 20.0;
    double max_rudder_angle = 30.0;
    double rudder_coefficient = 0.25;
    double rudder_distance = 2.0;
    double mass = 2000.0;
    double max_power = 8948.4;
    double max_speed = 2.75;
    //double probability[4] = {0.5,0.3,0.15,0.05};
    double probability[4] = {0,1,0,0};
    bool debug = true;


    struct pointc
    {
        double x, y, time;
        pointc(double x1, double y1, double time1)
                : x(x1), y(y1), time(time1){};
    };

    double max_prop_speed;
    double max_force;
    double prop_coefficient;
    double drag_coefficient;

    mutex mtx;
    bool running = true;
    bool plan = false;
    State start;
    State actions[4];
//    string path = "";
//    string default_Command = "0,0";
//    int receivePipeFromParent;
    double estimate_effect_speed = 0, estimate_effect_direction = 0;
    double ptime = 0;
    int iteration = 0;
    bool update = true;
    vector<pointc> future;

    double radians(double rudder_angle);

//    void readpath(FILE *readstream);

//    void requestAction();

    void estimate(double &rpm, double throttle, double d_time, double &speed, double rudder, double &heading, double &x, double &y);

    void MPC(double &r, double &t);

    void sendAction();
};


#endif //SRC_CONTROLLER_H
