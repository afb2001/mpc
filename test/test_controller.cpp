#include <gtest/gtest.h>
//#include "controller.h"
#include "../src/controller.h"

TEST(ControllerUnitTests, goNowhere)
{
    Controller controller(nullptr);
    State start(0,0,0,0,Controller::getTime());
    vector<State> reference;
    reference.emplace_back(0,0,0,0,Controller::getTime() + 1);
    double r, t;
    controller.MPC(r, t, start, reference, Controller::getTime() + 0.05);
//    EXPECT_DOUBLE_EQ(r, 0.0);
    EXPECT_DOUBLE_EQ(t, 0.0);
}

TEST(ControllerUnitTests, goNorth)
{
    Controller controller(nullptr);
    State start(0,0,0,0,Controller::getTime());
    vector<State> reference;
    double r, t;
    reference.emplace_back(0,1,0,2.5,Controller::getTime() + 1);
    reference.emplace_back(0,3.5,0,2.5,Controller::getTime() + 2);
    controller.MPC(r, t, start, reference, Controller::getTime() + 0.05);
    EXPECT_DOUBLE_EQ(r, 0.0);
    EXPECT_DOUBLE_EQ(t, 1.0);
}

TEST(ControllerUnitTests, goEast)
{
    Controller controller(nullptr);
    State start(0,0,0,0,Controller::getTime());
    vector<State> reference;
    double r, t;
    reference.emplace_back(1,0,M_PI/2,2.5,Controller::getTime() + 1);
    reference.emplace_back(3.5,0,M_PI/2,2.5,Controller::getTime() + 2);
    controller.MPC(r, t, start, reference, Controller::getTime() + 0.05);
    EXPECT_DOUBLE_EQ(r, 1);
    EXPECT_DOUBLE_EQ(t, 1.0);
}

TEST(ControllerUnitTests, goWest)
{
    Controller controller(nullptr);
    State start(0,0,0,0,Controller::getTime());
    vector<State> reference;
    double r, t;
    reference.emplace_back(-1,0,-M_PI/2,2.5,Controller::getTime() + 1);
    reference.emplace_back(-3.5,0,-M_PI/2,2.5,Controller::getTime() + 2);
    controller.MPC(r, t, start, reference, Controller::getTime() + 0.05);
    EXPECT_DOUBLE_EQ(r, -1);
    EXPECT_DOUBLE_EQ(t, 1.0);
}

TEST(ControllerUnitTests, realStateTest1)
{
    Controller controller(nullptr);
    VehicleState start(State(0,0,2,2.3,7));
    vector<State> reference;
    double r, t;
    reference.push_back(start.estimate(0.806, 0.765, 5, pair<double,double>(0,0)));
    controller.MPC(r, t, start, reference, Controller::getTime() + 0.05);
    EXPECT_LT(fabs(r - 0.806), 0.001);
    EXPECT_LT(fabs(t - 0.765), 0.001);
}

TEST(CurrentEstimatorTests, currentEstimatorTest1)
{
    CurrentEstimator currentEstimator;
    vector<State> reference;
    currentEstimator.updateEstimate(State(0,0,0,0,1));
    reference.emplace_back(0,0,0,0,1.05);
    currentEstimator.updatePredictedTrajectory(reference);
    pair<double,double> p(0,0);
    EXPECT_LT(fabs(currentEstimator.getCurrent().second - p.second), 0.001);
    currentEstimator.updateEstimate(State(0,0.1,0,0,1.05));
    p.second = 0.1 / 50 / 0.05;
    // floating point math makes me do this instead of equality comparison
    EXPECT_LT(fabs(currentEstimator.getCurrent().second - p.second), 0.001);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
