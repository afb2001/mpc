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
#include "OtherCurrentEstimator.h"
#include <mutex>
#include <random>
#include "path_planner/Trajectory.h"
#include "path_planner/TrajectoryDisplayer.h"

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
    explicit Controller(ControlReceiver *controlReceiver);

    ~Controller();

    /**
     * Update the reference trajectory.
     * @param trajectory new reference trajectory
     */
    void receiveRequest(const std::vector<State>& trajectory, long trajectoryNumber);

    /**
     * Respond to the new service call that supplies a new reference trajectory and expects an estimated state 1s in
     * the future. Starts a new thread to do MPC until the trajectory number is updated again.
     * @param trajectory
     * @param trajectoryNumber
     * @return a state 1 second in the future
     */
    State updateReferenceTrajectory(const std::vector<State>& trajectory, long trajectoryNumber);

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
    void mpc(double& r, double& t, State startCopy, std::vector<State> referenceTrajectoryCopy, double endTime, long trajectoryNumber = 0);

    /**
     * Different branching MPC. More details forthcoming.
     * @param r
     * @param t
     * @param startCopy
     * @param referenceTrajectoryCopy
     * @param endTime
     * @param trajectoryNumber
     */
    void mpc3(double& r, double& t, State startCopy, std::vector<State> referenceTrajectoryCopy, double endTime, long trajectoryNumber = 0);
    /**
     * (for lack of a better name)
     * Same as mpc3 but returns the state expected to be in 1s in the future
     * @param r
     * @param t
     * @param startCopy
     * @param referenceTrajectoryCopy
     * @param endTime
     * @param trajectoryNumber
     * @return
     */
    State mpc4(double& r, double& t, State startCopy, const std::vector<State>& referenceTrajectoryCopy, double endTime);

    /**
     * MPC but only one control out to the end of the trajectory.
     * Assumes reference trajectory is appropriately spaced. Interpolate reference trajectory beforehand if desired.
     * @param r resulting rudder
     * @param t resulting throttle
     * @param startCopy starting point
     * @param referenceTrajectoryCopy reference trajectory
     * @param endTime end time
     */
    void straightMpc(double& r, double& t, State startCopy, std::vector<State> referenceTrajectoryCopy, double endTime, long trajectoryNumber = 0);

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
    State estimateStateInFuture(double desiredTime, long& trajectoryNumber);

    /**
     * Update the configuration of the controller.
     * @param useBranching
     * @param weightSlope
     * @param weightStart
     * @param rudders
     * @param throttles
     * @param distanceWeight
     * @param headingWeight
     * @param speedWeight
     */
    void updateConfig(int useBranching, double weightSlope, double weightStart, int rudders, int throttles,
                      double distanceWeight, double headingWeight, double speedWeight);

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
        Control(int rb, int tb) { m_RudderBase = rb; m_ThrottleBase = tb; m_RudderCounter = -rb; m_ThrottleCounter = tb; }
        double incrementRudder() { m_RudderCounter++; return m_RudderCounter / m_RudderBase; }
        double incrementThrottle() { m_ThrottleCounter--; return m_ThrottleCounter / m_ThrottleBase; }
        void resetRudder() { m_RudderCounter = -(int)m_RudderBase; }
        void resetThrottle() { m_ThrottleCounter = (int)m_ThrottleBase; }
        bool rudderDone() { return m_RudderCounter > m_RudderBase; }
        bool throttleDone() { return m_ThrottleCounter < 0; }
        double getRudder() { return m_RudderCounter / m_RudderBase; }
        double getThrottle() { return m_ThrottleCounter / m_ThrottleBase; }
    private:
        double m_RudderBase, m_ThrottleBase;
        int m_RudderCounter, m_ThrottleCounter;
    };

    ControlReceiver* m_ControlReceiver;

    // configuration
    bool m_UseBranching;
    double m_WeightSlope, m_WeightStart;
    int m_Rudders, m_Throttles;
    double m_DistanceWeight, m_HeadingWeight, m_SpeedWeight;

    std::mutex mtx;
    bool running = false;
    bool plan = false;
    State m_CurrentLocation;
    std::vector<State> m_ReferenceTrajectory;

    std::mutex m_FutureStuffMutex;
    std::vector<std::pair<double,double>> m_FutureControls;
    std::vector<VehicleState> m_PredictedTrajectory;

    CurrentEstimator m_CurrentEstimator;
//    OtherCurrentEstimator m_CurrentEstimator;

    long m_TrajectoryNumber = 0;
    long m_NextTrajectoryNumber = 0;
    std::mutex m_TrajectoryNumberMutex;

    double m_LastRudder = 0, m_LastThrottle = 0; // should replace with some kind of collection with time stamps

    /**
     * Get the score weight for a state along the reference trajectory at the given index
     * @param index index of the state on the reference trajectory
     * @return a weight for the score
     */
    static double getMPCWeight(int index);
    double getMPCWeight(double timeFromStart) const;

    double compareStates(const State& s1, const VehicleState& s2) const;

    void sendAction();

    bool sendControl(double rudder, double throttle, long trajectoryNumber);

    bool validTrajectoryNumber(long trajectoryNumber);

    VehicleState getStateAfterCurrentControl();

    void sendControls(double r, double t);

    void runMpc(std::vector<State> trajectory, State start, State result, long trajectoryNumber);

    static constexpr double c_ScoringTimeStep = 1;
    static constexpr double c_Tolerance = 1.0e-5;
    static constexpr double c_PlanningTime = 0.1; // Time (seconds) for controller to think between issuing controls
    static constexpr double c_ReferenceTrajectoryExpirationTime = 5;

    /**
     * Interpolate along the given trajectory to the desired time.
     * @param desiredTime
     * @param trajectory
     * @return the interpolated state
     */
    static State interpolateTo(double desiredTime, const std::vector<State>& trajectory);
    /**
     * Interpolate along the given trajectory to the desired time.
     * Stores the result in the trajectory for re-use
     * @param desiredTime
     * @param trajectory
     * @return the interpolated state
     */
    static State interpolateTo(double desiredTime, std::list<State>& trajectory);
};


#endif //SRC_CONTROLLER_H
