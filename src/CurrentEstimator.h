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

    /**
     * Update the history, allowing updated estimates.
     * @param state
     * @param rudder
     * @param throttle
     */
    void updateEstimate(const State& state, double rudder, double throttle);

    /**
     * Get an estimate of the forces acting on the vehicle.
     *
     * Simulates past controls assuming no current and compares the result to the provided actual state. The difference
     * is assumed to be the result of a constant force acting on the vehicle over the time period.
     *
     * @param actual the actual state of the vehicle
     * @return a pair of doubles representing the m/s of displacement in <x, y>
     */
    std::pair<double, double> getCurrent(const State& actual) const;

    /**
     * Reset the estimate of the current to <0, 0> m/s.
     */
    void resetEstimate() {
        m_History.clear();
    }

    /**
     * Enable the current estimator.
     */
    void enable() { m_Enabled = true; }

    /**
     * Disable the current estimator (returns <0, 0> when disabled).
     */
    void disable() { m_Enabled = false; }

private:

    /**
     * Object to hold states and controls together in a collection.
     */
    struct StateControlPair {
    public:
        StateControlPair(VehicleState s, double r, double t) : state(s), rudder(r), throttle(t){}
        StateControlPair(State s, double r, double t) : state(s), rudder(r), throttle(t){}
        VehicleState state;
        double rudder, throttle;
    };
    std::list<StateControlPair> m_History; // history of state/control pairs
    bool m_Enabled = true;

    /**
     * Number of states to keep before we start wrapping around. I don't know what the right number here is, but
     * the buffer gets updated each MPC iteration, so this number divided by 10 (for 10Hz) represents the number of
     * seconds of history that is kept.
     */
    static constexpr int c_BufferSize = 600;
};

#endif //SRC_CURRENTESTIMATOR_H
