#ifndef SRC_VEHICLESTATE_H
#define SRC_VEHICLESTATE_H

#include "path_planner/State.h"

//using namespace std;

/**
 * Class representing a state of the vehicle beyond position, heading, speed and time. In this implementation that
 * extra information is just the rpm, but the class also holds the model constants.
 */
class VehicleState
{
public:
    /**
     * Construct a VehicleState, deriving the rpm from the state's speed.
     * @param state the underlying state
     */
    VehicleState(State state) : State(state) {
        rpm = idle_rpm + (state.speed / max_speed) * (max_rpm - idle_rpm);
    }

    VehicleState simulate(double rudder, double throttle, double t) {
        return simulate(rudder, throttle, t, std::make_pair(0.0, 0.0));
    }

    VehicleState simulate(double rudder, double throttle, double t, std::pair<double, double> estimatedCurrent) {
        std::vector<VehicleState> simulated;
        return simulate(rudder, throttle, t, estimatedCurrent, simulated);
    }

    VehicleState simulate(double rudder, double throttle, double t, std::pair<double, double> estimatedCurrent,
            std::vector<VehicleState>& predictedStates) {
        if (rudder < -1) rudder = -1;
        else if (rudder > 1) rudder = 1;
        if (throttle < 0) throttle = 0;
        else if (throttle > 1) throttle = 1;
        VehicleState state(*this);
        predictedStates.push_back(state);
        while (t > 0) {
            double dTime = t < 0.1 ? t : 0.1;
            t -= 0.1;
            predictedStates.push_back(estimate(state, rudder, throttle, dTime, estimatedCurrent));
        }
        return state;
    }

    VehicleState simulate(double rudder, double throttle, double t, double currentDirection, double currentSpeed,
            std::vector<VehicleState>& predictedStates) {
        if (rudder < -1) rudder = -1;
        else if (rudder > 1) rudder = 1;
        if (throttle < 0) throttle = 0;
        else if (throttle > 1) throttle = 1;
        VehicleState state(*this);
        predictedStates.push_back(state);
        while (t > 0) {
            double dTime = t < 0.1 ? t : 0.1;
            t -= 0.1;
            predictedStates.push_back(estimate(state, rudder, throttle, dTime, currentDirection, currentSpeed));
        }
        return state;
    }

#pragma clang diagnostic push
#pragma ide diagnostic ignored "hicpp-explicit-conversions"
    /**
     * Intentional implicit conversion.
     * A vehicle state is essentially a regular state so I'm doing this in lieu of polymorphism
     * @return
     */
    operator State() const { // NOLINT(google-explicit-constructor)
        return State;
    }
#pragma clang diagnostic pop

    /**
     * The RPMs at the state.
     */
    double rpm;

    State State;

private:

    // TODO! -- should allow setting of boat characteristics; used to read from a file I think
    //model parameters
    static constexpr double idle_rpm = 0.0;
    static constexpr double max_rpm = 3200.0;
    static constexpr double max_rpm_change_rate = 1000.0;
    static constexpr double prop_ratio = 0.389105058;
    static constexpr double prop_pitch = 20.0;
    static constexpr double max_rudder_angle = M_PI / 6; // 30 degrees
    static constexpr double rudder_coefficient = 0.25;
    static constexpr double rudder_distance = 2.0;
    static constexpr double mass = 2000.0;
    static constexpr double max_power = 8948.4;
    static constexpr double max_speed = 2.75;
    // useful derived parameters
    static constexpr double max_prop_speed = (max_rpm * prop_ratio) / prop_pitch;
    static constexpr double max_force = max_power / max_speed;
    static constexpr double prop_coefficient = max_force / (max_prop_speed * max_prop_speed - max_speed * max_speed);
    static constexpr double drag_coefficient = max_force / (max_speed * max_speed * max_speed);

    static VehicleState estimate(VehicleState& other, double rudder, double throttle, double d_time,
                          double currentDirection, double currentSpeed) {
        double target_rpm = idle_rpm + throttle * (max_rpm - idle_rpm);
        double rcr = (target_rpm - other.rpm) / d_time;

        if (rcr > max_rpm_change_rate) rcr = max_rpm_change_rate;
        else if (rcr < -max_rpm_change_rate) rcr = -max_rpm_change_rate;

        other.rpm += rcr * d_time;

        double prop_rpm = prop_ratio * other.rpm;
        double prop_speed = prop_rpm / prop_pitch;
        double rudder_speed = fmax(sqrt(prop_speed), other.State.speed);
        double thrust = prop_coefficient * (prop_speed * prop_speed - other.State.speed * other.State.speed);
        double rudder_angle = rudder * max_rudder_angle;
        double thrust_fwd = thrust * cos(rudder_angle);
        double rudder_speed_yaw = rudder_speed * sin(rudder_angle);
        double yaw_rate = rudder_coefficient * rudder_speed_yaw / rudder_distance;

        other.State.heading += yaw_rate * d_time;
        other.State.heading = fmod(other.State.heading, M_PI * 2);
        if (other.State.heading < 0)
            other.State.heading += M_PI * 2;

        double drag = pow(other.State.speed, 3) * drag_coefficient;
        other.State.speed += ((thrust_fwd - drag) / mass) * d_time;
        double delta = other.State.speed * d_time;

        auto deltaEV = currentSpeed * d_time;

        other.State.x += delta * sin(other.State.heading) + deltaEV * sin(currentDirection);
        other.State.y += delta * cos(other.State.heading) + deltaEV * cos(currentDirection);

        other.State.time += d_time;
        return other;
    }

    /**
     * Generate the state achieved by applying the given rudder and throttle for the given duration to this state.
     * @param rudder desired rudder
     * @param throttle desired throttle
     * @param t duration of controls
     * @param estimatedCurrent estimate of the <x,y> effect of the current (in m/s)
     * @return the resulting state
     */
    static VehicleState estimate(VehicleState& state, double rudder, double throttle, double d_time, std::pair<double, double> estimatedCurrent) {
        double target_rpm = idle_rpm + throttle * (max_rpm - idle_rpm);
        double rcr = (target_rpm - state.rpm) / d_time;

        if (rcr > max_rpm_change_rate) rcr = max_rpm_change_rate;
        else if (rcr < -max_rpm_change_rate) rcr = -max_rpm_change_rate;

        state.rpm += rcr * d_time;

        double prop_rpm = prop_ratio * state.rpm;
        double prop_speed = prop_rpm / prop_pitch;
        double rudder_speed = fmax(sqrt(prop_speed), state.State.speed);
        double thrust = prop_coefficient * (prop_speed * prop_speed - state.State.speed * state.State.speed);
        double rudder_angle = rudder * max_rudder_angle;
        double thrust_fwd = thrust * cos(rudder_angle);
        double rudder_speed_yaw = rudder_speed * sin(rudder_angle);
        double yaw_rate = rudder_coefficient * rudder_speed_yaw / rudder_distance;

        state.State.heading += yaw_rate * d_time;
        state.State.heading = fmod(state.State.heading, M_PI * 2);
        if (state.State.heading < 0)
            state.State.heading += M_PI * 2;

        double drag = pow(state.State.speed, 3) * drag_coefficient;
        state.State.speed += ((thrust_fwd - drag) / mass) * d_time;
        double delta = state.State.speed * d_time;

        state.State.x += delta * sin(state.State.heading) + estimatedCurrent.first * d_time;
        state.State.y += delta * cos(state.State.heading) + estimatedCurrent.second * d_time;

        state.State.time += d_time;
        return state;
    }
};

#endif //SRC_VEHICLESTATE_H
