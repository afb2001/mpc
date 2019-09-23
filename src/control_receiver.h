#include <path_planner/State.h>

#ifndef SRC_TRAJECTORY_PUBLISHER_H
#define SRC_CONTROL_RECEIVER_H

/**
 * Interface for receiving controls.
 */
class ControlReceiver
{
public:
    virtual ~ControlReceiver() = default;
    /**
     * Receive a rudder and throttle. This is the point of this interface.
     * @param rudder a rudder command in the interval [-1, 1]
     * @param throttle a throttle command in the interval [0, 1]
     */
    virtual void receiveControl(double rudder, double throttle) = 0;
    virtual void displayTrajectory(const std::vector<State>& trajectory, bool plannerTrajectory) = 0;

    /**
     * Get the current time.
     * @return the current time in seconds
     */
    virtual double getTime() const = 0;
};


#endif //SRC_TRAJECTORY_PUBLISHER_H
