#include <iostream>
#include "NodeStub.h"

using std::cerr;
using std::endl;

void NodeStub::receiveControl(double rudder, double throttle) {
    cerr << "NodeStub received control " << rudder << ", " << throttle << endl;
}

void NodeStub::displayTrajectory(const std::vector<State>& trajectory, bool plannerTrajectory) {
    cerr << "NodeStub displayed" << (plannerTrajectory? " planner " : " controller ") <<  "trajectory: \n";
    for (auto s : trajectory) cerr << s.toString() << endl;
    cerr << endl;
}

double NodeStub::getTime() const {
    return Controller::getTime();
}
