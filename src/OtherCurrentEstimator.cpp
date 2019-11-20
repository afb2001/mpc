#include "OtherCurrentEstimator.h"

void OtherCurrentEstimator::initialize(const VehicleState& start) {
    m_Controls.clear();
    m_States = std::vector<VehicleState>(c_BufferSize, start);
    m_EstimatedCurrentVector = std::vector<std::pair<double, double>>(c_BufferSize, std::make_pair(0.0, 0.0));
    m_EstimatedCurrent = std::make_pair(0.0, 0.0);
    std::cerr << "Current estimator initialized with state " << start.toString() << std::endl;
}

std::pair<double, double> OtherCurrentEstimator::getCurrent() const {
    return m_EstimatedCurrent;
}

void OtherCurrentEstimator::update(const VehicleState& state) {
    if (m_States.empty() || m_EstimatedCurrentVector.empty()) return;
    VehicleState old = m_States[currentEstimateIteration];
    auto dTime = state.time - old.time;
    if (dTime <= 0) {
        std::cerr << "Can't update current estimate because starting state is newer than current state" << std::endl;
        return;
    }
    if (m_Controls.empty()) {
        std::cerr << "Can't update current estimate because there are no known controls" << std::endl;
        return;
    }
    Control control = m_Controls.front();
    // when we start the controls start later than the starting time
//    if (control.time > state.time) {
//        std::cerr << "Can't update current estimate because there earliest control is later than the current state" << std::endl;
//        std::cerr << "Current state time is " << state.time << " and earliest control is from " << control.time << std::endl;
//        return;
//    }
    m_States[currentEstimateIteration] = state;
    for (auto it = m_Controls.begin()++; it != m_Controls.end(); it++) {
        if (it->time > state.time) break;
        if (it->time < old.time) continue;
        old = old.simulate(control.rudder, control.throttle, it->time - control.time);
        control = *it;
    }
    old = old.simulate(control.rudder, control. throttle, state.time - control.time);
    double dx = (state.x - old.x) / dTime;
    double dy = (state.y - old.y) / dTime;
    auto oldEstimate = m_EstimatedCurrentVector[currentEstimateIteration];
    std::cerr << "Updating old current estimate of " << oldEstimate.first << ", " << oldEstimate.second
        << " to " << dx << ", " << dy << std::endl;
    m_EstimatedCurrent.first -= oldEstimate.first / c_BufferSize;
    m_EstimatedCurrent.second -= oldEstimate.second / c_BufferSize;
    m_EstimatedCurrent.first += dx / c_BufferSize;
    m_EstimatedCurrent.second += dy / c_BufferSize;
    m_EstimatedCurrentVector[currentEstimateIteration++] = std::make_pair(dx, dy);
    currentEstimateIteration %= c_BufferSize;
}

void OtherCurrentEstimator::addControl(double rudder, double throttle, double time) {
    m_Controls.emplace_back(rudder, throttle, time);
    if (m_Controls.back().time <= m_Controls.front().time) {
        std::cerr << "Ending controls time is less than or equal to the starting controls time" << std::endl;
    }
    if (m_Controls.size() > c_BufferSize) m_Controls.pop_front();
}
