#ifndef SRC_CONTROLLER_H
#define SRC_CONTROLLER_H

#include "control_receiver.h"
#include "path_planner_common/State.h"
#include "VehicleState.h"
#include "CurrentEstimator.h"
#include <mutex>
#include <future>
#include <random>
#include <path_planner_common/DubinsPlan.h>

/**
 * Class which runs model predictive control and maintains everything required for it.
 * Model predictive control runs on its own thread, which is launched when it receives an updated reference trajectory.
 * That thread runs until either the next trajectory is received, a timeout is reached, or too little of the trajectory
 * is in the future.
 */
class Controller
{
public:

    /**
     * Struct to hold state-control pair (with score). This was originally local to MPC but is now the return type
     * for that member function.
     */
    struct MpcState {
    public:
        VehicleState state;
        double LastRudder{}, LastThrottle{};
        double Score{};
        MpcState(): state(State()){};
    };

    /**
     * Construct a Controller. It needs an instance of a ControlReceiver to publish rudder and throttle
     * pairs and to display trajectories.
     * @param controlReceiver interface which accepts rudders and throttles for publishing
     */
    explicit Controller(ControlReceiver *controlReceiver);

    ~Controller();

    /**
     * Respond to the new service call that supplies a new reference trajectory and expects an estimated state 1s in
     * the future. Starts a new thread to do MPC until the trajectory number is updated again.
     * @param trajectory
     * @param trajectoryNumber
     * @return a state 1 second in the future
     */
    State updateReferenceTrajectory(const DubinsPlan& referenceTrajectory, long trajectoryNumber);

    /**
     * Update the controller's idea of the current state of the vehicle. This is expected to be called rather
     * frequently, > 10Hz.
     * @param state the updated state
     */
    void updatePosition(State state);

    /**
     * Tell the MPC thread to stop running. This should be done when the controller should no longer control the boat.
     *
     */
    void terminate();

    /**
     * Do approximate MPC by scoring potential trajectories starting from startCopy based on the
     * reference trajectory until the current time reaches endTime. The best initial rudder and
     * throttle are assigned to r and t.
     *
     * This version of MPC uses limited branching piecewise constant controls to simulate potential trajectories. It
     * searches this space in an iterative deepening depth first manner. The implementation is a little obfuscated by
     * my attempt to use no heap-allocated memory, to improve efficiency.
     *
     * Limited branching refers to the fact that it considers only a small number of rudders and throttles at each depth,
     * including some extreme values, those of the previous control, and ones similar to the previous control. This
     * reduces computation time, increasing lookahead depth. It should be noted that the granularity with which controls
     * are selected seems to have a sizeable impact on performance, and that optimal values for control granularity
     * might depend on sea state and other environmental factors.
     *
     * @param startState starting state for MPC
     * @param referenceTrajectory reference trajectory
     * @param endTime computation deadline
     * @param trajectoryNumber
     * @return predicted state for next planning iteration (1s ahead), with controls to issue
     */
    MpcState mpc(State startState, const DubinsPlan& referenceTrajectory, double endTime,
                 long trajectoryNumber);

    /**
     * Utility for getting the time. It's public for testing but it really doesn't matter much.
     * @return the current time in seconds
     */
    static double getTime();

    /**
     * Update the configuration of the controller.
     * @param rudderGranularity
     * @param throttleGranularity
     * @param distanceWeight
     * @param headingWeight
     * @param speedWeight
     */
    void updateConfig(double rudderGranularity, double throttleGranularity, double distanceWeight, double headingWeight, double speedWeight,
                      double achievableThreshold, bool currentEstimation);

    /**
     * Set the trajectory number. Only public for testing.
     * @param trajectoryNumber
     */
    void setTrajectoryNumber(long trajectoryNumber);


    /**
     * Get the similarity score for these two states (lower score => more similar). This uses the weights set in
     * updateConfig. I don't see a use for this outside the controller but it's public to make it easier to test.
     * Maybe it should be its own class but that complicates things a little more.
     * @param s1
     * @param s2
     * @return
     */
    double compareStates(const State& s1, const VehicleState& s2) const;

private:
    // handle on the parent ROS node
    ControlReceiver* m_ControlReceiver;

    // currently running MPC task
    std::future<void> m_LastMpc;

    // whether the
    bool m_Achievable = true;

    // extreme controls
    // they're variables because they could end up as part of the exposed config at some point
    double m_MinRudder = -1, m_MidRudder = 0, m_MaxRudder = 1;
    double m_MinThrottle = 0, m_MaxThrottle = 1;

    /**
     * Time (seconds) for controller to think between issuing controls.
     * A variable in case we want to add it to the exposed config at some point.
     */
    double m_PlanningTime = 0.1;

    // configuration (parameters overwritten by dynamic reconfigure versions - see mpc.cfg for defaults)
    double m_RudderGranularity = 0.0625, m_ThrottleGranularity = 0.125;
    double m_DistanceWeight{}, m_HeadingWeight{}, m_SpeedWeight{};
    // Threshold below which the controller will tell the executive to simply assume the reference trajectory is
    // achievable. Should really be tuned with data somehow.
    double m_AchievableScoreThreshold = 2;
    // mutex to guard config
    // Mutable because we need it in const member functions. This is good use of mutable, I think
    mutable std::mutex m_ConfigMutex;

    // most up-to-date state of the vehicle
    std::mutex m_CurrentLocationMutex;
    State m_CurrentLocation;

    CurrentEstimator m_CurrentEstimator;
//    OtherCurrentEstimator m_CurrentEstimator;

    // There might be a better way to have old threads terminate than checking a counter but this seemed to make sense
    // to me at the time. That is the sole purpose of these member variables.
    long m_TrajectoryNumber = 0;
    std::mutex m_TrajectoryNumberMutex;

    // Might want to hold a collection of timestamped controls so we know what we've done in the past. I think the
    // current estimator does that, though, and right now that's the only place I can see them being useful.
    double m_LastRudder = 0, m_LastThrottle = 0;

    /**
     * Check that the trajectory number is still valid. Once it updates, the thread should terminate.
     * @param trajectoryNumber
     * @return true iff the trajectory number is still valid
     */
    bool validTrajectoryNumber(long trajectoryNumber);

    /**
     * Apply the most recent controls to the most recent state to determine where to start the next MPC iteration.
     * @return
     */
    VehicleState getStateAfterCurrentControl();

    /**
     * Send controls to the control receiver (ROS node).
     * @param r
     * @param t
     */
    void sendControls(double r, double t);

    /**
     * Run MPC until the trajectory gets updated (tested through the trajectory number) or the current trajectory times
     * out. This is designed to be run in a new thread.
     * @param trajectory
     * @param trajectoryNumber
     */
    void runMpc(DubinsPlan trajectory, long trajectoryNumber);

    // Constants
    /**
     * Time interval for scoring against the trajectory (seconds).
     */
    static constexpr double c_ScoringTimeStep = 1;
    /**
     * Tolerance used to circumvent some rounding errors. Pretty arbitrary.
     */
    static constexpr double c_Tolerance = 1.0e-5;
    /**
     * Flag that lets me set things a little differently for the controller running on its own.
     * This is really a temporary solution but it nicely marks everything that needs to get changed.
     */
    static constexpr bool c_ControllerTestMode = false;
    /**
     * Time (seconds) to continue to use a reference trajectory after the planner has issued it, if no new trajectories
     * are received.
     */
    static constexpr double c_ReferenceTrajectoryExpirationTime = c_ControllerTestMode? 50000000 : 5;

};


#endif //SRC_CONTROLLER_H
