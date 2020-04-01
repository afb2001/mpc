#include "CurrentEstimator.h"

CurrentEstimator::CurrentEstimator() { resetCurrentEstimate(); }

void CurrentEstimator::updateEstimate(const State& state, double rudder, double throttle) {
    m_History.emplace_back(state, rudder, throttle);
    if (m_History.size() > c_BufferSize) m_History.pop_front();
}

std::pair<double, double> CurrentEstimator::getCurrent(const State& actual) const {
    if (!m_Enabled) return std::make_pair(0.0, 0.0);
    if (m_History.empty()) return std::make_pair(0.0, 0.0); // no history yet
    VehicleState expected = m_History.front().state;
    auto timeDifference = actual.time() - expected.state.time();
    if (timeDifference <= 0) return std::make_pair(0.0, 0.0); // can't estimate unless there's time difference
    for (auto it = m_History.begin()++; it != m_History.end(); it++) {
        if (it->state.state.time() >= actual.time()) {
            expected = expected.simulate(it->rudder, it->throttle, actual.time() - expected.state.time());
            break;
        } else {
            expected = expected.simulate(it->rudder, it->throttle, it->state.state.time() - expected.state.time());
        }
    }
    if (expected.state.time() < actual.time()) {
        auto last = m_History.back();
        expected = expected.simulate(last.rudder, last.throttle, actual.time() - expected.state.time());
    }
    return std::make_pair((actual.x() - expected.state.x()) / timeDifference,
                          (actual.y() - expected.state.y()) / timeDifference);
}
