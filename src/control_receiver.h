#ifndef SRC_TRAJECTORY_PUBLISHER_H
#define SRC_CONTROL_RECEIVER_H

class ControlReceiver
{
public:
    virtual ~ControlReceiver() = default;
    virtual void receiveControl(double rudder, double throttle) = 0;
};


#endif //SRC_TRAJECTORY_PUBLISHER_H
