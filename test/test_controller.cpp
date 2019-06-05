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

TEST(CurrentEstimatorTests, currentEstimatorTest1)
{
    CurrentEstimator currentEstimator;
    vector<State> reference;
    // TODO
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
