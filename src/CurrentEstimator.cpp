#include "CurrentEstimator.h"

CurrentEstimator::CurrentEstimator() { resetEstimate(); }

void CurrentEstimator::updateEstimate(const State& state, double rudder, double throttle) {
    std::unique_lock<std::mutex> lock(m_HistoryMutex);
    if (m_SkipsLeft > 0) {
        m_SkipsLeft--;
        return;
    }
    m_History.emplace_back(state, rudder, throttle);
    if (m_History.size() > c_BufferSize) m_History.pop_front();
}

std::pair<double, double> CurrentEstimator::getCurrent(const State& actual) const {
    std::unique_lock<std::mutex> lock(m_HistoryMutex);
    if (!m_Enabled) return std::make_pair(0.0, 0.0);
    if (m_History.empty() || m_History.size() < 10) return std::make_pair(0.0, 0.0); // not enough history yet
    VehicleState expected = m_History.front().state;
    auto timeDifference = actual.time() - expected.state.time();
    if (timeDifference <= 0) return std::make_pair(0.0, 0.0); // can't estimate before our history starts
    for (auto it = m_History.begin()++; it != m_History.end(); it++) {
        if (it->state.state.time() >= actual.time()) {
            // "history" extends past the query time, and this is the first state in history past the query time, so
            // only simulate up to the query time
            expected = expected.simulate(it->rudder, it->throttle, actual.time() - expected.state.time());
            break;
        } else {
            // simulate up until the next control, because the query time is past that (this should be the typical case)
            expected = expected.simulate(it->rudder, it->throttle, it->state.state.time() - expected.state.time());
        }
    }
    // if the query time is past all our history, apply the last control to make up the difference
    if (expected.state.time() < actual.time()) {
        auto last = m_History.back();
        expected = expected.simulate(last.rudder, last.throttle, actual.time() - expected.state.time());
    }
    // return the difference between the expected (no current) vs the actual (presumably some current)
    return std::make_pair((actual.x() - expected.state.x()) / timeDifference,
                          (actual.y() - expected.state.y()) / timeDifference);
}
