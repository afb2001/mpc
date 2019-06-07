#include <utility>

#ifndef SRC_CURRENTESTIMATOR_H
#define SRC_CURRENTESTIMATOR_H

class CurrentEstimator {
public:
    CurrentEstimator(){ resetCurrentEstimate(); }
    void updateEstimate(const State &currentState) {
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

    pair<double, double> getCurrent() {
        return estimatedCurrent;
    }

    void updatePredictedTrajectory(vector<State> trajectory) {
        predictedTrajectory = std::move(trajectory);
    }

    void resetCurrentEstimate() { estimatedCurrentVector = vector<pair<double,double>>(50); }

private:
    pair<double, double> estimatedCurrent;
    vector<pair<double, double>> estimatedCurrentVector;
    int currentEstimateIteration = 0;
    double previousTime = 0;
    vector<State> predictedTrajectory;
};

#endif //SRC_CURRENTESTIMATOR_H
