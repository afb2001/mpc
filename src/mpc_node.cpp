#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "marine_msgs/Helm.h"
#include "marine_msgs/NavEulerStamped.h"
#include <vector>
#include "project11/gz4d_geo.h"
#include "actionlib/server/simple_action_server.h"
#include "path_planner/Trajectory.h"
#include "mpc/EstimateState.h"
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <path_planner/TrajectoryDisplayer.h>
#include "controller.h"
#include <mpc/mpcConfig.h>
#include <dynamic_reconfigure/server.h>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "OCInconsistentNamingInspection"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"

/**
 * ROS node which manages a model-predictive controller.
 */
class MPCNode: public TrajectoryDisplayer, public ControlReceiver
{
public:
    /**
     * Construct a MPCNode, setting up all the ROS publishers, subscribers and services, and initialize a Controller
     * instance.
     */
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

        dynamic_reconfigure::Server<mpc::mpcConfig>::CallbackType f;
        f = boost::bind(&MPCNode::reconfigureCallback, this, _1, _2);

        m_Dynamic_Reconfigure_Server.setCallback(f);
    }

    /**
     * Tell the Controller instance to terminate before freeing it.
     */
    ~MPCNode() final
    {
        m_Controller->terminate(); // make sure its thread can exit
        delete m_Controller;
    }

    /**
     * Process a message for the controller.
     * @param inmsg the string message
     */
    void controllerMsgsCallback(const std_msgs::String::ConstPtr &inmsg)
    {
        std::string message = inmsg->data;
        std::cerr << "MPC node received message to: " << message << std::endl;
        if (message == "start running") {
            m_Controller->startRunning();
        } else if (message == "start sending controls") {
            m_Controller->startSendingControls();
        } else if (message == "terminate") {
            m_Controller->terminate();
        } else if (message == "stop sending controls") {
            m_Controller->stopSendingControls();
            TrajectoryDisplayer::displayTrajectory(std::vector<State>(), false);
        }
    }

    /**
     * Update the reference trajectory of the controller.
     * @param inmsg the new reference trajectory.
     */
    void referenceTrajectoryCallback(const path_planner::Trajectory::ConstPtr &inmsg)
    {
        std::vector<State> states;
        for (const auto &s : inmsg->states) {
            states.push_back(getState(s));
        }
//        m_TrajectoryNumber = inmsg->trajectoryNumber;
        m_Controller->receiveRequest(states, inmsg->trajectoryNumber);
    }

    /**
     * Update the controller's idea of the vehicle's heading.
     * @param inmsg a message containing the new heading
     */
    void headingCallback(const marine_msgs::NavEulerStamped::ConstPtr& inmsg)
    {
        m_current_heading = inmsg->orientation.heading * M_PI / 180.0;
    }

    /**
     * Update the controller's idea of the vehicle's speed.
     * @param inmsg a message containing the new speed
     */
    void speedCallback(const geometry_msgs::TwistStamped::ConstPtr& inmsg)
    {
        m_current_speed = inmsg->twist.linear.x; // this will change once /sog is a vector
    }

    /**
     * Update the controller's idea of the vehicle's position.
     * @param inmsg a message containing the new position
     */
    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &inmsg)
    {
        m_Controller->updatePosition(State(
                inmsg->pose.position.x,
                inmsg->pose.position.y,
                m_current_heading,
                m_current_speed,
                getTime()));
    }

    void reconfigureCallback(mpc::mpcConfig &config, uint32_t level)
    {
        m_Controller->updateConfig(config.mpc_type,
                config.weight_slope, config.weight_start,
                config.rudders, config.throttles,
                config.distance_weight, config.heading_weight, config.speed_weight);
    }

    /**
     * Publish a rudder and throttle to /helm.
     * @param rudder rudder command
     * @param throttle throttle command
     */
    void receiveControl(double rudder, double throttle) final
    {
        marine_msgs::Helm helm;
        helm.rudder = rudder;
        helm.throttle = throttle;
        helm.header.stamp = ros::Time::now();
        m_helm_pub.publish(helm);
    }

    void displayTrajectory(const std::vector<State>& trajectory, bool plannerTrajectory) final
    {
        TrajectoryDisplayer::displayTrajectory(trajectory, plannerTrajectory);
    }

    /**
     * Handle a service call requesting an estimate of a state in the future.
     * @param req the request (continaing a time)
     * @param res the response (containing a State)
     * @return whether the service call succeeded
     */
    bool estimateStateInFuture(mpc::EstimateState::Request &req, mpc::EstimateState::Response &res) {
//        cerr << "Received service call " << endl;
        auto s = m_Controller->estimateStateInFuture(req.desiredTime, res.trajectoryNumber);
        res.state = getStateMsg(s);
        return s.time != -1;
    }

    double getTime() const override {
        return TrajectoryDisplayer::getTime();
    }

private:
    double m_current_speed = 0; // marginally better than having it initialized with junk
    double m_current_heading = 0;

    ros::Publisher m_helm_pub;

    ros::Subscriber m_controller_msgs_sub;
    ros::Subscriber m_reference_trajectory_sub;
    ros::Subscriber m_position_sub;
    ros::Subscriber m_heading_sub;
    ros::Subscriber m_speed_sub;

    ros::ServiceServer m_estimate_state_service;

//    long m_TrajectoryNumber = 0;

    Controller* m_Controller;
    dynamic_reconfigure::Server<mpc::mpcConfig> m_Dynamic_Reconfigure_Server;
};

int main(int argc, char **argv)
{
    std::cerr << "Starting MPC node" << std::endl;
    ros::init(argc, argv, "mpc");
    MPCNode node;
    ros::spin();

    return 0;
}

#pragma clang diagnostic pop