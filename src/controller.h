#ifndef SRC_CONTROLLER_H
#define SRC_CONTROLLER_H

#include "control_receiver.h"
#include "path_planner/State.h"
#include "VehicleState.h"
#include "CurrentEstimator.h"
#include <mutex>
#include <future>
#include <random>
#include "path_planner/Trajectory.h"
#include "path_planner/TrajectoryDisplayer.h"
#include "StateInterpolater.h"

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
     * Tell the MPC thread to stop running. This should be done when the controller should no longer control the boat.
     *
     */
    void terminate();

    /**
     * Do approximate MPC by scoring potential trajectories starting from startCopy based on the
     * reference trajectory until the current time reaches endTime. The best initial rudder and
     * throttle are assigned to r and t.
     *
     * This version of MPC uses limited branching piecewise constant controls to simulate potential trajectories.
     *
     * @param r
     * @param t
     * @param startCopy
     * @param referenceTrajectoryCopy
     * @param endTime
     * @param trajectoryNumber
     */
    void mpc(double& r, double& t, State startCopy, std::vector<State> referenceTrajectoryCopy, double endTime, long trajectoryNumber);
    State mpc2(double& r, double& t, State startCopy, std::vector<State> referenceTrajectoryCopy, double endTime, long trajectoryNumber);

    /**
     * Same as mpc but returns the state expected to be in 1s in the future
     * @param r
     * @param t
     * @param startCopy
     * @param referenceTrajectoryCopy
     * @param endTime
     * @param trajectoryNumber
     * @return
     */
    State initialMpc(double& r, double& t, State startCopy, const std::vector<State>& referenceTrajectoryCopy, double endTime);

    /**
     * Utility for getting the time. It's public for testing but it really doesn't matter much.
     * @return the current time in seconds
     */
    static double getTime();

    /**
     * Update the configuration of the controller.
     * @param rudders
     * @param throttles
     * @param distanceWeight
     * @param headingWeight
     * @param speedWeight
     */
    void updateConfig(int rudders, int throttles, double distanceWeight, double headingWeight, double speedWeight,
                      double achievableThreshold);

    /**
     * Set the trajectory number. Only public for testing.
     * @param trajectoryNumber
     */
    void setTrajectoryNumber(long trajectoryNumber);

private:

    ControlReceiver* m_ControlReceiver;

    std::future<void> m_LastMpc;

    bool m_Achievable = true;

    // configuration
    int m_Rudders = 21, m_Throttles = 5;
    double m_DistanceWeight{}, m_HeadingWeight{}, m_SpeedWeight{};

    std::mutex m_CurrentLocationMutex;
    State m_CurrentLocation;

    CurrentEstimator m_CurrentEstimator;
//    OtherCurrentEstimator m_CurrentEstimator;

    long m_TrajectoryNumber = 0;
    std::mutex m_TrajectoryNumberMutex;

    double m_LastRudder = 0, m_LastThrottle = 0; // should replace with some kind of collection with time stamps

    double compareStates(const State& s1, const VehicleState& s2) const;
    double compareStates(const State& s1, const State& s2) const;

    bool validTrajectoryNumber(long trajectoryNumber);

    VehicleState getStateAfterCurrentControl();

    void sendControls(double r, double t);

    /**
     * Run MPC until the trajectory gets updated (tested through the trajectory number) or the current trajectory times
     * out. This is designed to be run in a new thread.
     * @param trajectory
     * @param start
     * @param result
     * @param trajectoryNumber
     */
    void runMpc(std::vector<State> trajectory, State start, State result, long trajectoryNumber);

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
     * Time (seconds) for controller to think between issuing controls.
     */
    static constexpr double c_PlanningTime = 0.1;
    /**
     * Time (seconds) to continue to use a reference trajectory after the planner has issued it, if no new trajectories
     * are received.
     */
    static constexpr double c_ReferenceTrajectoryExpirationTime = 5; // if this is big it's for controller tests

    /**
     * Threshold below which the controller will tell the executive to simply assume the reference trajectory is
     * achievable. Should really be tuned with data somehow.
     */
    double m_AchievableScoreThreshold = 2;

    /**
     * Interpolate along the given trajectory to the desired time.
     * @param desiredTime
     * @param trajectory
     * @return the interpolated state
     */
    static State interpolateTo(double desiredTime, const std::vector<State>& trajectory);
};


#endif //SRC_CONTROLLER_H
