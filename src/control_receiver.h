#include <path_planner/State.h>

#ifndef SRC_TRAJECTORY_PUBLISHER_H
#define SRC_CONTROL_RECEIVER_H

class ControlReceiver
{
public:
    virtual ~ControlReceiver() = default;
    virtual void receiveControl(double rudder, double throttle) = 0;
    virtual void displayTrajectory(std::vector<State> trajectory, bool plannerTrajectory) = 0;
};


#endif //SRC_TRAJECTORY_PUBLISHER_H
