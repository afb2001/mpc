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

/**
 * Class which runs model predictive control and maintains everything required for it.
 * Model predictive control runs on its own thread. This class exposes member functions to update the current state of
 * the vehicle and the desired trajectory, but MPC can run regardless of when those are updated as long as it has been
 * started and not paused or terminated.
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
    explicit Controller(ControlReceiver *controlReceiver, TrajectoryDisplayer *trajectoryDisplayer);

    ~Controller();

    /**
     * Update the reference trajectory.
     * @param trajectory new reference trajectory
     */
    void receiveRequest(const path_planner::Trajectory::ConstPtr& trajectory);

    /**
     * Update the controller's idea of the current state of the vehicle.
     * @param state the updated state
     */
    void updatePosition(State state);

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
    void MPC(double &r, double &t, State startCopy, std::vector<State> referenceTrajectoryCopy, double endTime);

    /**
     * Utility for getting the time. It's public for testing but it really doesn't matter much.
     * @return the current time in seconds
     */
    static double getTime();

    /**
     * Estimate where we will be at the specified time according to the most recent projected trajectory.
     * @param desiredTime the desired time (must be in the future)
     * @return an estimate of the state we'll be in at desiredTime
     */
    State estimateStateInFuture(double desiredTime);

private:

    /**
     * This class lets me have a stream of controls to iterate through without having to
     * explicitly do the math where they're used.
     * No documentation is provided because it's really just internal and self-explanatory.
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
    TrajectoryDisplayer* m_TrajectoryDisplayer;

    std::mutex mtx;
    bool running = false;
    bool plan = false;
    State start;
    std::vector<State> referenceTrajectory;

    std::mutex m_FutureStuffMutex;
    std::vector<std::pair<double,double>> m_FutureControls;
    std::vector<State> m_PredictedTrajectory;

    CurrentEstimator m_CurrentEstimator;

    /**
     * Get the score weight for a state along the reference trajectory at the given index
     * @param index index of the state on the reference trajectory
     * @return a weight for the score
     */
    static double getMPCWeight(int index);

    void sendAction();
};


#endif //SRC_CONTROLLER_H
