#ifndef SRC_VEHICLESTATE_H
#define SRC_VEHICLESTATE_H

#include "path_planner/State.h"

//using namespace std;

/**
 * Class representing a state of the vehicle beyond position, heading, speed and time. In this implementation that
 * extra information is just the rpm, but the class also holds the model constants.
 */
class VehicleState : public State
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

    /**
     * The RPMs at the state.
     */
    double rpm;

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

    VehicleState estimate(VehicleState& state, double rudder, double throttle, double d_time,
            double currentDirection, double currentSpeed) {
        double target_rpm = idle_rpm + throttle * (max_rpm - idle_rpm);
        double rcr = (target_rpm - state.rpm) / d_time;

        if (rcr > max_rpm_change_rate) rcr = max_rpm_change_rate;
        else if (rcr < -max_rpm_change_rate) rcr = -max_rpm_change_rate;

        state.rpm += rcr * d_time;

        double prop_rpm = prop_ratio * state.rpm;
        double prop_speed = prop_rpm / prop_pitch;
        double rudder_speed = fmax(sqrt(prop_speed), state.speed);
        double thrust = prop_coefficient * (prop_speed * prop_speed - state.speed * state.speed);
        double rudder_angle = rudder * max_rudder_angle;
        double thrust_fwd = thrust * cos(rudder_angle);
        double rudder_speed_yaw = rudder_speed * sin(rudder_angle);
        double yaw_rate = rudder_coefficient * rudder_speed_yaw / rudder_distance;

        state.heading += yaw_rate * d_time;
        state.heading = fmod(state.heading, M_PI * 2);
        if (state.heading < 0)
            state.heading += M_PI * 2;

        double drag = pow(state.speed, 3) * drag_coefficient;
        state.speed += ((thrust_fwd - drag) / mass) * d_time;
        double delta = state.speed * d_time;

        auto deltaEV = currentSpeed * d_time;

        state.x += delta * sin(state.heading) + deltaEV * sin(currentDirection);
        state.y += delta * cos(state.heading) + deltaEV * cos(currentDirection);

        state.time += d_time;
        return state;
    }

    /**
     * Generate the state achieved by applying the given rudder and throttle for the given duration to this state.
     * @param rudder desired rudder
     * @param throttle desired throttle
     * @param t duration of controls
     * @param estimatedCurrent estimate of the <x,y> effect of the current (in m/s)
     * @return the resulting state
     */
    VehicleState estimate(VehicleState& state, double rudder, double throttle, double d_time, std::pair<double, double> estimatedCurrent) const {
        double target_rpm = idle_rpm + throttle * (max_rpm - idle_rpm);
        double rcr = (target_rpm - state.rpm) / d_time;

        if (rcr > max_rpm_change_rate) rcr = max_rpm_change_rate;
        else if (rcr < -max_rpm_change_rate) rcr = -max_rpm_change_rate;

        state.rpm += rcr * d_time;

        double prop_rpm = prop_ratio * state.rpm;
        double prop_speed = prop_rpm / prop_pitch;
        double rudder_speed = fmax(sqrt(prop_speed), state.speed);
        double thrust = prop_coefficient * (prop_speed * prop_speed - state.speed * state.speed);
        double rudder_angle = rudder * max_rudder_angle;
        double thrust_fwd = thrust * cos(rudder_angle);
        double rudder_speed_yaw = rudder_speed * sin(rudder_angle);
        double yaw_rate = rudder_coefficient * rudder_speed_yaw / rudder_distance;

        state.heading += yaw_rate * d_time;
        state.heading = fmod(state.heading, M_PI * 2);
        if (state.heading < 0)
            state.heading += M_PI * 2;

        double drag = pow(state.speed, 3) * drag_coefficient;
        state.speed += ((thrust_fwd - drag) / mass) * d_time;
        double delta = state.speed * d_time;

        state.x += delta * sin(state.heading) + estimatedCurrent.first * d_time;
        state.y += delta * cos(state.heading) + estimatedCurrent.second * d_time;

        state.time += d_time;
        return state;
    }
};

#endif //SRC_VEHICLESTATE_H
