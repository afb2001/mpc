#ifndef SRC_CONTROLLER_H
#define SRC_CONTROLLER_H

/**
 * This is the maximum length of reference trajectory allowed. Longer trajectories will be truncated.
 */
#define MAX_LOOKAHEAD_STEPS 20

#include "control_receiver.h"
#include "path_planner/State.h"
#include "VehicleState.h"
#include "current_estimator.h"
#include <mutex>
#include <random>
#include "path_planner/Trajectory.h"

using namespace std;

class Controller
{
public:

    explicit Controller(ControlReceiver* controlReceiver);

    ~Controller();

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

    /**
     * Do approximate MPC by scoring potential trajectories starting from startCopy based on the
     * reference trajectory until the current time reaches endTime. The best initial rudder and
     * throttle are assigned to r and t.
     *
     * Over time this function increases first the number of states in the reference trajectory it
     * scores against and then the granularity at which controls are generated. In the lookahead-increasing
     * phase at each iteration an additional state on the trajectory is considered. That phase ends when
     * MAX_LOOKAHEAD_DEPTH is reached, and the control-granularity-increasing phase begins. Note that
     * no interpolation is done between states on the reference trajectory, so the granularity of the
     * trajectory provided will not be altered. Please provide appropriate trajectories.
     *
     * @param r resulting rudder
     * @param t resulting throttle
     * @param startCopy starting point
     * @param referenceTrajectoryCopy reference trajectory
     * @param endTime end time
     */
    void MPC(double &r, double &t, State startCopy, vector<State> referenceTrajectoryCopy, double endTime);

    /**
     * Utility for getting the time. It's public for testing but it really doesn't matter much.
     * @return the current time in seconds
     */
    static double getTime();

private:

    /**
     * This class lets me have a stream of controls to iterate through without having to
     * explicitly do the math where they're used.
     */
    class Control
    {
    public:
        Control() : Control(10, 10) {};
        Control(int rb, int tb) { rudderBase = rb; throttleBase = tb; rudderCounter = -rb; throttleCounter = tb; }
        double incrementRudder() { rudderCounter++; return rudderCounter / rudderBase; }
        double incrementThrottle() { throttleCounter--; return throttleCounter / throttleBase; }
        void resetRudder() { rudderCounter = -(int)rudderBase; }
        void resetThrottle() { throttleCounter = (int)throttleBase; }
        bool rudderDone() { return rudderCounter > rudderBase; }
        bool throttleDone() { return throttleCounter < 0; }
        double getRudder() { return rudderCounter / rudderBase; }
        double getThrottle() { return throttleCounter / throttleBase; }
    private:
        double rudderBase, throttleBase;
        int rudderCounter, throttleCounter;
    };



    ControlReceiver* m_ControlReceiver;

    struct control
    {
        double rudder, throttle;
        control(double r, double t):
            rudder(r), throttle(t){};
    };

    mutex mtx;
    bool running = false;
    bool plan = false;
    State start;
    vector<State> referenceTrajectory;

    CurrentEstimator m_CurrentEstimator;

    static double getMPCWeight(int index);

    void sendAction();
};


#endif //SRC_CONTROLLER_H
