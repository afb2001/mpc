#ifndef SRC_VEHICLESTATE_H
#define SRC_VEHICLESTATE_H

#include "path_planner/State.h"
#include <vector>

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
    VehicleState(State state) : state(state) {
        rpm = idle_rpm + (state.speed() / max_speed) * (max_rpm - idle_rpm);
    }

    VehicleState simulate(double rudder, double throttle, double t) {
        return simulate(rudder, throttle, t, std::make_pair(0.0, 0.0));
    }

    VehicleState simulate(double rudder, double throttle, double t, std::pair<double, double> estimatedCurrent) {
        if (rudder < -1) rudder = -1;
        else if (rudder > 1) rudder = 1;
        if (throttle < 0) throttle = 0;
        else if (throttle > 1) throttle = 1;
        VehicleState vehicleState(*this);
        while (t > 0) {
            double dTime = t < 0.1 ? t : 0.1;
            t -= 0.1;
            estimate(vehicleState, rudder, throttle, dTime, estimatedCurrent);
        }
        return vehicleState;
    }

    VehicleState simulate(double rudder, double throttle, double t, std::pair<double, double> estimatedCurrent,
            std::vector<VehicleState>& predictedStates) {
        if (rudder < -1) rudder = -1;
        else if (rudder > 1) rudder = 1;
        if (throttle < 0) throttle = 0;
        else if (throttle > 1) throttle = 1;
        VehicleState vehicleState(*this);
        predictedStates.push_back(vehicleState);
        while (t > 0) {
            double dTime = t < 0.1 ? t : 0.1;
            t -= 0.1;
            predictedStates.push_back(estimate(vehicleState, rudder, throttle, dTime, estimatedCurrent));
        }
        return vehicleState;
    }

#pragma clang diagnostic push
#pragma ide diagnostic ignored "hicpp-explicit-conversions"
//    /**
//     * Intentional implicit conversion.
//     * A vehicle state is essentially a regular state so I'm doing this in lieu of polymorphism
//     * @return
//     */
//    operator State() const { // NOLINT(google-explicit-constructor)
//        return state;
//    }
#pragma clang diagnostic pop

    std::string toString() const {
        return state.toStringRad();
    }

    /**
     * The RPMs at the state.
     */
    double rpm;

    /**
     * Keep track of the course made good.
     */
    double courseMadeGood;

    State state;

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

    /**
     * Generate the state achieved by applying the given rudder and throttle for the given duration to this state.
     * @param rudder desired rudder
     * @param throttle desired throttle
     * @param t duration of controls
     * @param estimatedCurrent estimate of the <x,y> effect of the current (in m/s)
     * @return the resulting state
     */
    static VehicleState estimate(VehicleState& vehicleState, double rudder, double throttle, double d_time, std::pair<double, double> estimatedCurrent) {
        double target_rpm = idle_rpm + throttle * (max_rpm - idle_rpm);
        double rcr = (target_rpm - vehicleState.rpm) / d_time;

        if (rcr > max_rpm_change_rate) rcr = max_rpm_change_rate;
        else if (rcr < -max_rpm_change_rate) rcr = -max_rpm_change_rate;

        vehicleState.rpm += rcr * d_time;

        double prop_rpm = prop_ratio * vehicleState.rpm;
        double prop_speed = prop_rpm / prop_pitch;
        double rudder_speed = fmax(sqrt(prop_speed), vehicleState.state.speed());
        double thrust = prop_coefficient * (prop_speed * prop_speed - vehicleState.state.speed() * vehicleState.state.speed());
        double rudder_angle = rudder * max_rudder_angle;
        double thrust_fwd = thrust * cos(rudder_angle);
        double rudder_speed_yaw = rudder_speed * sin(rudder_angle);
        double yaw_rate = rudder_coefficient * rudder_speed_yaw / rudder_distance;

        vehicleState.state.heading() += yaw_rate * d_time;
        vehicleState.state.heading() = fmod(vehicleState.state.heading(), M_PI * 2);
        if (vehicleState.state.heading() < 0)
            vehicleState.state.heading() += M_PI * 2;

        double speed = vehicleState.state.speed();
        double drag = speed * speed * speed * drag_coefficient;
        vehicleState.state.speed() += ((thrust_fwd - drag) / mass) * d_time;
        double delta = vehicleState.state.speed() * d_time;

        auto dx = delta * sin(vehicleState.state.heading()) + estimatedCurrent.first * d_time;
        auto dy = delta * cos(vehicleState.state.heading()) + estimatedCurrent.second * d_time;
        vehicleState.state.x() += dx;
        vehicleState.state.y() += dy;
        vehicleState.courseMadeGood = M_PI_2 - atan2(dy, dx); // will be the same as heading if current is <0, 0>

        vehicleState.state.time() += d_time;
        return vehicleState;
    }
};

#endif //SRC_VEHICLESTATE_H
