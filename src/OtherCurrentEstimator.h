#ifndef SRC_OTHERCURRENTESTIMATOR_H
#define SRC_OTHERCURRENTESTIMATOR_H


#include <list>
#include <vector>
#include "VehicleState.h"

/**
 * Class to estimate the current. It keeps a circular array of past states, and a similar list of controls issued which
 * it uses the simulate past states up to the current time. The results of these simulations are compared to the current
 * position of the vessel to estimate the current.
 */
class OtherCurrentEstimator {
public:
    /**
     * Initialize the estimator.
     * @param start the current location
     */
    void initialize(const VehicleState& start);

    /**
     * Retrieve the estimate of the current.
     * @return
     */
    std::pair<double, double> getCurrent() const;

    /**
     * Update the estimate of the current, given the state we are in.
     * @param state
     */
    void update(const VehicleState& state);

    /**
     * Store a control issued in order to simulate them later.
     * @param rudder
     * @param throttle
     * @param time
     */
    void addControl(double rudder, double throttle, double time);

private:
    class Control {
    public:
        Control(double r, double th, double t) : rudder(r), throttle(th), time(t){};
        double rudder, throttle, time;
    };

    std::vector<VehicleState> m_States; // array of old states
    std::list<Control> m_Controls; // controls to simulate from old states

    std::pair<double, double> m_EstimatedCurrent; // average current
    std::vector<std::pair<double, double>> m_EstimatedCurrentVector; // array of estimates
    unsigned long currentEstimateIteration = 0; // index into estimate array

    static constexpr int c_BufferSize = 50;
};


#endif //SRC_OTHERCURRENTESTIMATOR_H
