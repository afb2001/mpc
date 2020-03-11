#ifndef SRC_NODESTUB_H
#define SRC_NODESTUB_H


#include <vector>
#include "../src/controller.h"

class NodeStub : public ControlReceiver {
public:
    void receiveControl(double rudder, double throttle) override;

    void displayTrajectory(const std::vector<State>& trajectory, bool plannerTrajectory, bool achievable) override;

    double getTime() const override;
};


#endif //SRC_NODESTUB_H
