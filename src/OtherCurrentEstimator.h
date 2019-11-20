#ifndef SRC_OTHERCURRENTESTIMATOR_H
#define SRC_OTHERCURRENTESTIMATOR_H


#include <list>
#include <vector>
#include "VehicleState.h"

class OtherCurrentEstimator {
public:
    void initialize(const VehicleState& start);

    std::pair<double, double> getCurrent() const;

    void update(const VehicleState& state);

    void addControl(double rudder, double throttle, double time);
private:
    class Control {
    public:
        Control(double r, double th, double t) : rudder(r), throttle(th), time(t){};
        double rudder, throttle, time;
    };
    std::vector<VehicleState> m_States;
    std::list<Control> m_Controls;

    std::pair<double, double> m_EstimatedCurrent;
    std::vector<std::pair<double, double>> m_EstimatedCurrentVector;
    unsigned long currentEstimateIteration = 0;

    static constexpr int c_BufferSize = 50;
};


#endif //SRC_OTHERCURRENTESTIMATOR_H
