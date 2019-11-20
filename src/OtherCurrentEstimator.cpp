#include "OtherCurrentEstimator.h"

void OtherCurrentEstimator::initialize(const VehicleState& start) {
    m_Controls.clear();
    // initialize an array with copies of the starting state
    m_States = std::vector<VehicleState>(c_BufferSize, start);
    // fill the estimates array with zeros
    m_EstimatedCurrentVector = std::vector<std::pair<double, double>>(c_BufferSize, std::make_pair(0.0, 0.0));
    // and set the average to zero
    m_EstimatedCurrent = std::make_pair(0.0, 0.0);
    // this is just for debugging
    std::cerr << "Current estimator initialized with state " << start.toString() << std::endl;
}

std::pair<double, double> OtherCurrentEstimator::getCurrent() const {
    return m_EstimatedCurrent;
}

void OtherCurrentEstimator::update(const VehicleState& state) {
    // if things aren't initialized somehow just return (TODO)
    if (m_States.empty() || m_EstimatedCurrentVector.empty()) return;
    // grab an old state we were in
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
    // TODO! -- when we start the controls start later than the starting time, so I commented this out
//    if (control.time > state.time) {
//        std::cerr << "Can't update current estimate because there earliest control is later than the current state" << std::endl;
//        std::cerr << "Current state time is " << state.time << " and earliest control is from " << control.time << std::endl;
//        return;
//    }
    // replace the old state with the current one (could be done later I think)
    m_States[currentEstimateIteration] = state;
    // simulate the old state using the controls we recorded up to the current time
    for (auto it = m_Controls.begin()++; it != m_Controls.end(); it++) {
        if (it->time > state.time) break;
        if (it->time < old.time) continue;
        old = old.simulate(control.rudder, control.throttle, it->time - control.time);
        control = *it;
    }
    old = old.simulate(control.rudder, control. throttle, state.time - control.time);
    // calculate the changes in x and y over time (m/s)
    double dx = (state.x - old.x) / dTime;
    double dy = (state.y - old.y) / dTime;
    // grab the old estimate that's in the slot
    auto oldEstimate = m_EstimatedCurrentVector[currentEstimateIteration];
    std::cerr << "Updating old current estimate of " << oldEstimate.first << ", " << oldEstimate.second
        << " to " << dx << ", " << dy << std::endl;
    // subtract the old estimate from the average
    m_EstimatedCurrent.first -= oldEstimate.first / c_BufferSize;
    m_EstimatedCurrent.second -= oldEstimate.second / c_BufferSize;
    // add the new estimate to the average
    m_EstimatedCurrent.first += dx / c_BufferSize;
    m_EstimatedCurrent.second += dy / c_BufferSize;
    // assign the new estimate to the slot in the buffer and increment the slot number
    m_EstimatedCurrentVector[currentEstimateIteration++] = std::make_pair(dx, dy);
    // and finally mod it by the buffer size
    currentEstimateIteration %= c_BufferSize;
}

void OtherCurrentEstimator::addControl(double rudder, double throttle, double time) {
    // record the new control that we presumably are about to issue
    m_Controls.emplace_back(rudder, throttle, time);
    if (m_Controls.back().time <= m_Controls.front().time) {
        std::cerr << "Ending controls time is less than or equal to the starting controls time" << std::endl;
    }
    // don't keep more controls than we have states
    if (m_Controls.size() > c_BufferSize) m_Controls.pop_front();
}
