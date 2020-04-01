#include <utility>
#include <mutex>
#include "VehicleState.h"
#include <list>

#ifndef SRC_CURRENTESTIMATOR_H
#define SRC_CURRENTESTIMATOR_H

/**
 * This class is responsible for estimating the current (or other outside forces acting on the vehicle).
 */
class CurrentEstimator {
public:
    /**
     * Construct a CurrentEstimator that assumes <0, 0> current to start off.
     */
    CurrentEstimator();

    void updateEstimate(const State& state, double rudder, double throttle);

    /**
     * Get an estimate of the forces acting on the vehicle.
     * @return a pair of doubles representing the m/s of displacement in <x, y>
     */
    std::pair<double, double> getCurrent(const State& actual) const;

    /**
     * Reset the estimate of the current to <0, 0> m/s.
     */
    void resetCurrentEstimate() {
        m_History.clear();
    }

    void enable() { m_Enabled = true; }

    void disable() { m_Enabled = false; }

private:

    struct StateControlPair {
    public:
        StateControlPair(VehicleState s, double r, double t) : state(s), rudder(r), throttle(t){}
        StateControlPair(State s, double r, double t) : state(s), rudder(r), throttle(t){}
        VehicleState state;
        double rudder, throttle;
    };
    std::list<StateControlPair> m_History; // history of state/control pairs
    bool m_Enabled = true;

    static constexpr int c_BufferSize = 600;
};

#endif //SRC_CURRENTESTIMATOR_H
