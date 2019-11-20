#include "OtherCurrentEstimator.h"

void OtherCurrentEstimator::initialize(const VehicleState& start) {
    m_Controls.clear();
    m_States = std::vector<VehicleState>(c_BufferSize, start);

}

std::pair<double, double> OtherCurrentEstimator::getCurrent() const {
    return m_EstimatedCurrent;
}

void OtherCurrentEstimator::update(const VehicleState& state) {
    VehicleState old = m_States[currentEstimateIteration];
    auto dTime = state.time - old.time;
    if (dTime <= 0) return;
    if (m_Controls.empty()) return;
    Control control = m_Controls.front();
    if (control.time > state.time) return;
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
    m_EstimatedCurrent.first -= oldEstimate.first / c_BufferSize;
    m_EstimatedCurrent.second -= oldEstimate.second / c_BufferSize;
    m_EstimatedCurrent.first += dx / c_BufferSize;
    m_EstimatedCurrent.second += dy / c_BufferSize;
    m_EstimatedCurrentVector[currentEstimateIteration++] = std::make_pair(dx, dy);
    currentEstimateIteration %= c_BufferSize;
}

void OtherCurrentEstimator::addControl(double rudder, double throttle, double time) {
    m_Controls.emplace_back(rudder, throttle, time);
    if (m_Controls.size() > c_BufferSize) m_Controls.pop_front();
}
