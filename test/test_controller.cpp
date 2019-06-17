#include <gtest/gtest.h>
//#include "controller.h"
#include "../src/controller.h"
using std::vector;
using std::pair;
using std::cerr;
using std::endl;

TEST(ControllerUnitTests, goNowhere)
{
    Controller controller(nullptr, nullptr);
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
    Controller controller(nullptr, nullptr);
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
    Controller controller(nullptr, nullptr);
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
    Controller controller(nullptr, nullptr);
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
    Controller controller(nullptr, nullptr);
    VehicleState start(State(0,0,2,2.3,7));
    vector<State> reference;
    double r, t;
    reference.push_back(start.estimate(0.806, 0.765, 5, pair<double,double>(0,0)));
    controller.MPC(r, t, start, reference, Controller::getTime() + 0.05);
    EXPECT_LT(fabs(r - 0.806), 0.001);
    EXPECT_LT(fabs(t - 0.765), 0.001);
}

TEST(ControllerUnitTests, trajectoryDepthTest1)
{
    Controller controller(nullptr, nullptr);
    VehicleState start(State(0,0,2,2.3,7));
    vector<State> reference;
    double r, t;
    auto vs = start.estimate(0, 1, 0.499, pair<double,double>(0,0));
    vs.time -= 0.2;
    reference.push_back(vs);
    vs.time += 0.2;
    reference.push_back(vs);
    controller.MPC(r, t, start, reference, Controller::getTime() + 0.1);
    EXPECT_LT(fabs(r), 0.001);
    EXPECT_LT(fabs(t - 1), 0.001);
}

TEST(ControllerUnitTests, futureEstimateTest1)
{
    Controller controller(nullptr, nullptr);
    VehicleState start(State(0,0,0,1,7.5));
    vector<State> reference;
    double r, t;
    reference.emplace_back(0, -2, M_PI, 2, 8);
    reference.emplace_back(0, -3, M_PI, 2, 8.5);
    reference.emplace_back(0, -4, M_PI, 2, 9);
    reference.emplace_back(0, -5, M_PI, 2, 9.5);
    reference.emplace_back(0, -6, M_PI, 2, 10);
    reference.emplace_back(0, -7, M_PI, 2, 10.5);
    reference.emplace_back(0, -8, M_PI, 2, 11);
    reference.emplace_back(0, -9, M_PI, 2, 11.5);
    reference.emplace_back(0, -10, M_PI, 2, 12);
    controller.MPC(r, t, start, reference, Controller::getTime() + 0.1);
    cerr << controller.estimateStateInFuture(8).toString() << endl;
    cerr << controller.estimateStateInFuture(9).toString() << endl;
    cerr << controller.estimateStateInFuture(10).toString() << endl;
    cerr << controller.estimateStateInFuture(11).toString() << endl;
    cerr << controller.estimateStateInFuture(12).toString() << endl;
}

TEST(CurrentEstimatorTests, currentEstimatorTest1)
{
    CurrentEstimator currentEstimator;
    vector<State> reference;
    reference.emplace_back(0,0,0,0,1.05);
    currentEstimator.updateEstimate(State(0,0,0,0,1), reference);
    pair<double,double> p(0,0);
    EXPECT_LT(fabs(currentEstimator.getCurrent().second - p.second), 0.001);
    currentEstimator.updateEstimate(State(0,0.1,0,0,1.05), reference);
    p.second = 0.1 / 50 / 0.05;
    // floating point math makes me do this instead of equality comparison
    EXPECT_LT(fabs(currentEstimator.getCurrent().second - p.second), 0.001);
}

TEST(VehicleStateTests, estimateTest0)
{
    VehicleState s1(State(0,0,2,2.3,7));
    auto s2 = s1.estimate(0, 1, 3, pair<double,double>(0,0));
    auto s3 = s1.estimate(0, 1, 3, pair<double,double>(0,0));
    EXPECT_DOUBLE_EQ(s2.time, s3.time);
    EXPECT_DOUBLE_EQ(s2.x, s3.x);
    EXPECT_DOUBLE_EQ(s2.y, s3.y);
    EXPECT_DOUBLE_EQ(s2.heading, s3.heading);
    EXPECT_DOUBLE_EQ(s2.speed, s3.speed);
}

TEST(VehicleStateTests, estimateTest1)
{
    VehicleState s1(State(0,0,2,2.3,7));
    auto sFinal = s1.estimate(0, 1, 5, pair<double,double>(0,0));
    auto s2 = s1.estimate(0, 1, 2, pair<double,double>(0,0));
    auto s3 = s2.estimate(0, 1, 3, pair<double,double>(0,0));
    EXPECT_DOUBLE_EQ(sFinal.time, s3.time);
    EXPECT_DOUBLE_EQ(sFinal.x, s3.x);
    EXPECT_DOUBLE_EQ(sFinal.y, s3.y);
    EXPECT_DOUBLE_EQ(sFinal.heading, s3.heading);
    EXPECT_DOUBLE_EQ(sFinal.speed, s3.speed);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
