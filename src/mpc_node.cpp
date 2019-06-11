#include "ros/ros.h"
#include "geographic_msgs/GeoPointStamped.h"
#include "geographic_msgs/GeoPath.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "marine_msgs/Helm.h"
#include "marine_msgs/NavEulerStamped.h"
#include <vector>
#include "project11/gz4d_geo.h"
#include "path_follower/path_followerAction.h"
#include "actionlib/server/simple_action_server.h"
#include "path_planner/Trajectory.h"
#include "mpc/EstimateState.h"
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <project11_transformations/LatLongToMap.h>
#include <thread>
#include <signal.h>
#include <geometry_msgs/PoseStamped.h>
#include <path_planner/TrajectoryDisplayer.h>
#include "geographic_visualization_msgs/GeoVizItem.h"
#include "geographic_visualization_msgs/GeoVizPointList.h"

//#include "control_receiver.h"
#include "controller.h"


class MPCNode: public TrajectoryDisplayer, public ControlReceiver
{
public:
    explicit MPCNode() : TrajectoryDisplayer()
    {
        m_helm_pub = m_node_handle.advertise<marine_msgs::Helm>("/helm",1);

        m_controller_msgs_sub = m_node_handle.subscribe("/controller_msgs", 10, &MPCNode::controllerMsgsCallback, this);
        m_reference_trajectory_sub = m_node_handle.subscribe("/reference_trajectory", 1, &MPCNode::referenceTrajectoryCallback, this);
        m_position_sub = m_node_handle.subscribe("/position_map", 10, &MPCNode::positionCallback, this);
        m_heading_sub = m_node_handle.subscribe("/heading", 10, &MPCNode::headingCallback, this);
        m_speed_sub = m_node_handle.subscribe("/sog", 10, &MPCNode::speedCallback, this);

        m_estimate_state_service = m_node_handle.advertiseService("/mpc/estimate_state", &MPCNode::estimateStateInFuture, this);

        m_Controller = new Controller(this);
    }

    ~MPCNode() final
    {
        m_Controller->terminate(); // make sure its thread can exit
        delete m_Controller;
    }

    void controllerMsgsCallback(const std_msgs::String::ConstPtr &inmsg)
    {
        string message = inmsg->data;
        std::cerr << "MPC node received message to: " << message << endl;
        if (message == "start running") {
            m_Controller->startRunning();
        } else if (message == "start sending controls") {
            m_Controller->startSendingControls();
        } else if (message == "terminate") {
            m_Controller->terminate();
        } else if (message == "stop sending controls") {
            m_Controller->stopSendingControls();
        }
    }

    void referenceTrajectoryCallback(const path_planner::Trajectory::ConstPtr &inmsg)
    {
        m_Controller->receiveRequest(inmsg);
    }

    void headingCallback(const marine_msgs::NavEulerStamped::ConstPtr& inmsg)
    {
        m_current_heading = inmsg->orientation.heading * M_PI / 180.0;
    }

    void speedCallback(const geometry_msgs::TwistStamped::ConstPtr& inmsg)
    {
        m_current_speed = inmsg->twist.linear.x; // this will change once /sog is a vector
    }

    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &inmsg)
    {
        m_Controller->updatePosition(State(
                inmsg->pose.position.x,
                inmsg->pose.position.y,
                m_current_speed,
                m_current_heading,
                inmsg->header.stamp.toNSec() / 1.0e9));
    }

    void receiveControl(double rudder, double throttle) final
    {
        marine_msgs::Helm helm;
        helm.rudder = rudder;
        helm.throttle = throttle;
        helm.header.stamp = ros::Time::now();
        m_helm_pub.publish(helm);
    }

    void displayTrajectory(vector<State> trajectory, bool plannerTrajectory) override
    {
        TrajectoryDisplayer::displayTrajectory(trajectory, plannerTrajectory);
        cerr << "Displayed controller trajectory" << endl;
    }

    bool estimateStateInFuture(mpc::EstimateState::Request &req, mpc::EstimateState::Response &res) {
//        cerr << "Received service call " << endl;
        auto s = m_Controller->estimateStateInFuture(req.desiredTime);
        res.state = (path_planner::StateMsg)s;
        return s.time != -1;
    }

private:
    double m_current_speed;
    double m_current_heading;

//    ros::NodeHandle m_node_handle;

    ros::Publisher m_helm_pub;
//    ros::Publisher m_display_pub;

    ros::Subscriber m_controller_msgs_sub;
    ros::Subscriber m_reference_trajectory_sub;
    ros::Subscriber m_position_sub;
    ros::Subscriber m_heading_sub;
    ros::Subscriber m_speed_sub;

    ros::ServiceServer m_estimate_state_service;
    Controller* m_Controller;
};

int main(int argc, char **argv)
{
    std::cerr << "Starting MPC node" << endl;
    ros::init(argc, argv, "mpc");
    MPCNode node;
    ros::spin();

    return 0;
}
