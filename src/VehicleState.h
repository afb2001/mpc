#ifndef SRC_VEHICLESTATE_H
#define SRC_VEHICLESTATE_H

#include "State.h"

using namespace std;

class VehicleState : public State
{
public:
    VehicleState(State state) : State(state) {
        rpm = idle_rpm + (state.speed / max_speed) * (max_rpm - idle_rpm);
    }

    /**
     * Generate the state achieved by applying the given rudder and throttle for the given duration to this state.
     * @param rudder desired rudder
     * @param throttle desired throttle
     * @param d_time duration of controls
     * @param estimatedCurrent estimate of the <x,y> effect of the current (in m/s)
     * @return the resulting state
     */
    VehicleState estimate(double rudder, double throttle, double d_time, pair<double, double> estimatedCurrent) {
        VehicleState state(*this);
        double target_rpm = idle_rpm + throttle * (max_rpm - idle_rpm);
        double rcr = (target_rpm - state.rpm) / d_time;

        if (rcr > max_rpm_change_rate) rcr = max_rpm_change_rate;
        else if (rcr < -max_rpm_change_rate) rcr = -max_rpm_change_rate;

        state.rpm += rcr * d_time;

        double prop_speed = prop_ratio * state.rpm / prop_pitch;
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

        state.otime += d_time;

        return state;
    }

    double rpm;

private:

    // TODO! -- should allow setting of boat characteristics; used to read from a file I think
    //model parameters
    double idle_rpm = 0.0;
    double max_rpm = 3200.0;
    double max_rpm_change_rate = 1000.0;
    double prop_ratio = 0.389105058;
    double prop_pitch = 20.0;
    double max_rudder_angle = M_PI / 6; // 30 degrees
    double rudder_coefficient = 0.25;
    double rudder_distance = 2.0;
    double mass = 2000.0;
    double max_power = 8948.4;
    double max_speed = 2.75;
    // useful derived parameters
    double max_prop_speed = (max_rpm * prop_ratio) / prop_pitch;
    double max_force = max_power / max_speed;
    double prop_coefficient = max_force / (max_prop_speed * max_prop_speed - max_speed * max_speed);
    double drag_coefficient = max_force / (pow(max_speed, 3));
};

#endif //SRC_VEHICLESTATE_H
