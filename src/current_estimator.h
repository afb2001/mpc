#include <utility>
#include <mutex>

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
        if (previousTime != 0 && previousTime != currentState.time && !predictedTrajectory.empty()) {
            int index = 0;
            for (int i = 1; i < predictedTrajectory.size(); i++) {
                if (currentState.time <= predictedTrajectory[i].time) {
                    index = (fabs(predictedTrajectory[i].time - currentState.time) <
                            fabs(predictedTrajectory[i - 1].time - currentState.time)) ?
                                    i : i - 1;
                    break;
                }
            }
            if (predictedTrajectory[index].time <= 0) return; // if the states are invalid don't estimate current
            auto old = estimatedCurrentVector[currentEstimateIteration];
            estimatedCurrent.first -= old.first / estimatedCurrentVector.size();
            estimatedCurrent.second -= old.second / estimatedCurrentVector.size();
            double dtime = predictedTrajectory[index].time - (predictedTrajectory[0].time - (currentState.time - previousTime));
            old.first += (currentState.x - predictedTrajectory[index].x) / dtime;
            old.second += (currentState.y - predictedTrajectory[index].y) / dtime;
            estimatedCurrent.first += old.first / estimatedCurrentVector.size();
            estimatedCurrent.second += old.second / estimatedCurrentVector.size();
            estimatedCurrentVector[currentEstimateIteration] = old;
            currentEstimateIteration = (currentEstimateIteration + 1) % estimatedCurrentVector.size();
        }
        // if the trajectory is empty still update the old time
        previousTime = currentState.time;
    }

    /**
     * Get an estimate of the forces acting on the vehicle.
     * @return a pair of doubles representing the m/s of displacement in <x, y>
     */
    std::pair<double, double> getCurrent() {
//        return std::make_pair(0.0, 0.0);
        return estimatedCurrent;
    }

    /**
     * Reset the estimate of the current to <0, 0> m/s.
     */
    void resetCurrentEstimate() { estimatedCurrentVector = std::vector<std::pair<double,double>>(50); }

private:
    std::pair<double, double> estimatedCurrent;
    std::vector<std::pair<double, double>> estimatedCurrentVector;
    int currentEstimateIteration = 0;
    double previousTime = 0;
};

#endif //SRC_CURRENTESTIMATOR_H
