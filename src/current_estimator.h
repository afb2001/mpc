#include <utility>
#include <mutex>
#include "VehicleState.h"

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
    CurrentEstimator(){ resetCurrentEstimate(); }

    /**
     * Update the estimate of the current with where we are and where we thought we would be.
     * @param currentState the updated state of the vehicle
     * @param predictedTrajectory the old predicted trajectory
     */
    void updateEstimate(const State &currentState, const std::vector<VehicleState>& predictedTrajectory) {
        if (previousTime != 0 && previousTime != currentState.time && predictedTrajectory.size() > 1) {
            int index = 1;
            for (; index < predictedTrajectory.size(); index++) {
                if (currentState.time < predictedTrajectory[index].time) {
                    break;
                }
            }
            if (predictedTrajectory[index].time <= 0) return; // if the states are invalid don't estimate current
            auto old = estimatedCurrentVector[currentEstimateIteration];
            estimatedCurrent.first -= old.first / estimatedCurrentVector.size();
            estimatedCurrent.second -= old.second / estimatedCurrentVector.size();
            auto interpolated = predictedTrajectory[index - 1].interpolate(predictedTrajectory[index], currentState.time);
            old.first += (currentState.x -interpolated.x);
            old.second += (currentState.y - interpolated.y);
            estimatedCurrent.first += old.first / estimatedCurrentVector.size();
            estimatedCurrent.second += old.second / estimatedCurrentVector.size();
            estimatedCurrentVector[currentEstimateIteration] = old;
            currentEstimateIteration = (currentEstimateIteration + 1) % estimatedCurrentVector.size();
        }
        // hack to cap current
        if (estimatedCurrent.first > 5) estimatedCurrent.first = 5;
        if (estimatedCurrent.first < -5) estimatedCurrent.first = -5;
        if (estimatedCurrent.second > 5) estimatedCurrent.second = 5;
        if (estimatedCurrent.second < -5) estimatedCurrent.second = -5;

        // if the trajectory is empty still update the old time
        previousTime = currentState.time;
//        std::cerr << "Estimated current: " << estimatedCurrent.first << ", " << estimatedCurrent.second << std::endl;
    }

    /**
     * Chao's current estimation update
     * @param startCopy
     * @param future
     */
    void updateEstimate2(const State& startCopy, const std::vector<VehicleState>& future) {
        if (ptime != startCopy.time && !future.empty()) {
            if (iteration < 50)
                ++iteration;
            ptime = startCopy.time;
            double cx = startCopy.x;
            double cy = startCopy.y;
            int index = 0;
            while (index < future.size() - 1 && future[index + 1].time < startCopy.time) index++;
//            for (int i = 1; i < future.size(); i++) {
//                if (ptime <= future[i].time) {
//                    index = (fabs(future[i].time - ptime) < fabs(future[i - 1].time - ptime)) ? i : i - 1;
//                    break;
//                }
//            }
//            double dtime = future[index].time - (future[0].time - 0.05);
            double dtime = future[index].time - startCopy.time;
            double diffx = (startCopy.x - future[index].x) / dtime;
            double diffy = (startCopy.y - future[index].y) / dtime;
            double deltax = estimate_effect_speed * sin(estimate_effect_direction);
            double deltay = estimate_effect_speed * cos(estimate_effect_direction);
            deltax += diffx / iteration;
            deltay += diffy / iteration;
            estimate_effect_direction = atan2(deltax, deltay);
            double cosd = cos(estimate_effect_direction);
            estimate_effect_speed = (cosd > 0.1) ? deltay / cosd : deltax / sin(estimate_effect_direction);
            if (estimate_effect_direction < 0)
                estimate_effect_direction = fmod(estimate_effect_direction + M_PI * 10000, M_PI * 2);
            else if (estimate_effect_direction > 2 * M_PI)
                estimate_effect_direction = fmod(estimate_effect_direction, M_PI * 2);
        }
        std::cerr << "Estimated current speed: " << estimate_effect_speed << ", direction: " << estimate_effect_direction << std::endl;
    }

    /**
     * Get an estimate of the forces acting on the vehicle.
     * @return a pair of doubles representing the m/s of displacement in <x, y>
     */
    std::pair<double, double> getCurrent() const {
//        return std::make_pair(0.0, 0.0);
        return estimatedCurrent;
    }


    // Chao's current estimation accessors
    double getCurrentSpeed() const {
        return estimate_effect_speed;
    }
    double getCurrentDirection() const {
        return estimate_effect_direction;
    }

    /**
     * Reset the estimate of the current to <0, 0> m/s.
     */
    void resetCurrentEstimate() { estimatedCurrentVector = std::vector<std::pair<double,double>>(5); }

private:
    std::pair<double, double> estimatedCurrent;
    std::vector<std::pair<double, double>> estimatedCurrentVector;
    unsigned long currentEstimateIteration = 0;
    double previousTime = 0;

    // Chao's variables for current estimation
    double estimate_effect_speed = 0, estimate_effect_direction = 0;
    double ptime = 0;
    int iteration = 0;
};

#endif //SRC_CURRENTESTIMATOR_H
