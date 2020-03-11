#ifndef SRC_STATEINTERPOLATER_H
#define SRC_STATEINTERPOLATER_H

/**
 * I made this its own class to facilitate testing without breaking encapsulation.
 * Why is it so hard to have an encapsulated and testable design?
 */
class StateInterpolater {
public:
    /**
     * Interpolate along the given trajectory to the desired time.
     * @param desiredTime
     * @param trajectory
     * @return the interpolated state
     */
    static State interpolateTo(double desiredTime, const std::vector<State>& trajectory) {
        // doesn't handle duplicates well, I suspect
        if (trajectory.size() < 2) throw std::logic_error("Cannot interpolate on a trajectory with fewer than 2 states");
        int i = 1;
        for (; i < trajectory.size() - 1 && trajectory[i].time() < desiredTime; i++) ;
        if (trajectory[i].time() < desiredTime) std::cerr << "Warning: extrapolating instead of interpolating" << std::endl;
        return trajectory[i - 1].interpolate(trajectory[i], desiredTime);
    }
};

#endif //SRC_STATEINTERPOLATER_H
