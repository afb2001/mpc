#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "marine_msgs/Helm.h"
#include "marine_msgs/NavEulerStamped.h"
#include <vector>
#include "project11/gz4d_geo.h"
#include <path_planner_common/UpdateReferenceTrajectory.h>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include "controller.h"
#include <path_planner_common/DubinsPlan.h>
#include <mpc/mpcConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geographic_msgs/GeoPoint.h>
#include <path_planner_common/TrajectoryDisplayerHelper.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "OCInconsistentNamingInspection"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"

/**
 * ROS node which manages a model-predictive controller. The reference trajectory topic and the update reference
 * trajectory service should be used with mutual exclusion: both start new threads for MPC, the only difference being
 * that the service needs to return a state for the planner, and the topic is for other uses (none exist at this time).
 */
class MPCNode final: public ControlReceiver
{
public:
    /**
     * Construct a MPCNode, setting up all the ROS publishers, subscribers and services, and initialize a Controller
     * instance.
     */
    explicit MPCNode()
    {
        m_helm_pub = m_node_handle.advertise<marine_msgs::Helm>("/helm",1);
        m_display_pub = m_node_handle.advertise<geographic_visualization_msgs::GeoVizItem>("/project11/display",1);

        m_controller_msgs_sub = m_node_handle.subscribe("/controller_msgs", 10, &MPCNode::controllerMsgsCallback, this);
        m_position_sub = m_node_handle.subscribe("/position_map", 10, &MPCNode::positionCallback, this);
        m_heading_sub = m_node_handle.subscribe("/heading", 10, &MPCNode::headingCallback, this);
        m_speed_sub = m_node_handle.subscribe("/sog", 10, &MPCNode::speedCallback, this);
        m_piloting_mode_sub = m_node_handle.subscribe("/project11/piloting_mode", 10, &MPCNode::pilotingModeCallback, this);
        m_reference_trajectory_sub = m_node_handle.subscribe("/mpc/reference_trajectory", 10, &MPCNode::referenceTrajectoryCallback, this);
        m_disturbance_estimate_sub = m_node_handle.subscribe("/disturbance_estimate", 5, &MPCNode::disturbanceEstimateCallback, this);

        m_update_reference_trajectory_service = m_node_handle.advertiseService("/mpc/update_reference_trajectory", &MPCNode::updateReferenceTrajectory, this);

        m_Controller = new Controller(this);

        dynamic_reconfigure::Server<mpc::mpcConfig>::CallbackType f;
        f = boost::bind(&MPCNode::reconfigureCallback, this, _1, _2);

        m_Dynamic_Reconfigure_Server.setCallback(f);

        m_TrajectoryDisplayer = TrajectoryDisplayerHelper(m_node_handle, &m_display_pub);

        m_Output = &std::cerr;
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
        *m_Output << "MPC node received message to: " << message << std::endl;
        if (message == "start running") {
            /* Deprecated */
        } else if (message == "start sending controls") {
            /* Deprecated */
        } else if (message == "terminate") {
            m_Controller->terminate();
            m_DisturbanceEstimator.resetEstimate();
        } else if (message == "stop sending controls") {
            /* Deprecated */
        }
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
        // extract yaw from orientation
        tf::Quaternion q0(inmsg->pose.orientation.x,
                          inmsg->pose.orientation.y,
                          inmsg->pose.orientation.z,
                          inmsg->pose.orientation.w);
        tf::Matrix3x3 m0(q0);
        double roll, pitch, yaw;
        m0.getRPY(roll, pitch, yaw);
        auto heading = M_PI_2 - yaw; // convert yaw to heading

//        *m_Output << q0.x() << ", " << q0.y() << ", " << q0.z() << ", " << q0.w() << ", " << q0.length() << std::endl;

//        *m_Output << "Roll, pitch, yaw: " << roll << ", " << pitch << ", " << yaw << std::endl;

//        *m_Output << "Calculated heading is: " << heading << std::endl;

//        *m_Output << "Difference between heading and calculated heading is: " << heading - m_current_heading << std::endl;

        m_current_x = inmsg->pose.position.x;
        m_current_y = inmsg->pose.position.y;

        m_Controller->updatePosition(State(
                m_current_x,
                m_current_y,
                m_current_heading,
                m_current_speed,
                getTime()));
    }

    /**
     * Update controller parameters from dynamic reconfig.
     * @param config
     * @param level
     */
    void reconfigureCallback(mpc::mpcConfig &config, uint32_t level)
    {
        m_Controller->updateConfig(
                config.rudder_granularity, config.throttle_granularity,
                config.distance_weight, config.heading_weight, config.speed_weight,
                config.achievable_threshold, config.current_estimation);
        if (config.current_estimation) m_DisturbanceEstimator.enable();
        else m_DisturbanceEstimator.disable();
    }

    /**
     * Callback to update piloting mode.
     * @param inmsg
     */
    void pilotingModeCallback(const std_msgs::String::ConstPtr& inmsg) {
        // only enabled when autonomous
        m_Enabled = inmsg->data == "autonomous";
        if (!m_Enabled) m_Controller->terminate();
    }

    void referenceTrajectoryCallback(const path_planner_common::PlanConstPtr& inmsg) {
        // ignore state returned, just update
        m_Controller->updateReferenceTrajectory(convertPlanFromMessage(*inmsg), m_TrajectoryNumber++, false);
    }

    void disturbanceEstimateCallback(const geometry_msgs::Vector3::ConstPtr& inmsg) {
        m_DisturbanceEstimate = std::make_pair(inmsg->x, inmsg->y);
    }

    /**
     * Publish a rudder and throttle to /helm.
     * @param rudder rudder command
     * @param throttle throttle command
     */
    void receiveControl(double rudder, double throttle) final
    {
        if (!m_Enabled) return;
        marine_msgs::Helm helm;
        helm.rudder = rudder;
        helm.throttle = throttle;
        helm.header.stamp = ros::Time::now();
        m_helm_pub.publish(helm);
        // uncomment below (and comment out last line) for current estimation instead of true value
        State currentState(m_current_x, m_current_y, m_current_heading, m_current_speed, m_TrajectoryDisplayer.getTime());
        m_DisturbanceEstimator.updateEstimate(currentState, rudder, throttle);
        auto current = m_DisturbanceEstimator.getCurrent(currentState);
        m_Controller->updateDisturbanceEstimate(current.first, current.second);
//        m_Controller->updateDisturbanceEstimate(m_DisturbanceEstimate.first, m_DisturbanceEstimate.second);
    }

    /**
     * Display a predicted trajectory to /project11/display.
     * @param trajectory
     * @param plannerTrajectory
     */
    void displayTrajectory(const std::vector<State>& trajectory, bool plannerTrajectory, bool achievable) final
    {
        m_TrajectoryDisplayer.displayTrajectory(trajectory, plannerTrajectory, achievable);
    }

    /**
     * Fulful the service by calling MPC once and returning a state 1s into the future. The controller's reference
     * trajectory is updated (obviously) and MPC is set to run for some time.
     * @param req
     * @param res
     * @return
     */
    bool updateReferenceTrajectory(path_planner_common::UpdateReferenceTrajectory::Request &req, path_planner_common::UpdateReferenceTrajectory::Response &res) {
        if (!m_Enabled) return false;
        auto s = m_Controller->updateReferenceTrajectory(convertPlanFromMessage(req.plan), m_TrajectoryNumber++, true);
        res.state = m_TrajectoryDisplayer.convertToStateMsg(s);
        return s.time() != -1;
    }

    /**
     * Convert to internal DubinsPlan type from the ROS message type.
     * @param plan
     * @return
     */
    static DubinsPlan convertPlanFromMessage(path_planner_common::Plan plan) {
        DubinsPlan result;
        for (const auto& d : plan.paths) {
            DubinsWrapper wrapper;
            DubinsPath path;
            path.qi[0] = d.initial_x;
            path.qi[1] = d.initial_y;
            path.qi[2] = d.initial_yaw;
            path.param[0] = d.length0;
            path.param[1] = d.length1;
            path.param[2] = d.length2;
            path.rho = d.rho;
            path.type = (DubinsPathType)d.type;
            wrapper.fill(path, d.speed, d.start_time);
            if (wrapper.getEndTime() > plan.endtime){
                wrapper.updateEndTime(plan.endtime);
            }
            result.append(wrapper);
        }
        return result;
    }


    /**
     * @return the current time in seconds
     */
    double getTime() const override {
        return m_TrajectoryDisplayer.getTime();
    }

private:
    ros::NodeHandle m_node_handle;

    TrajectoryDisplayerHelper m_TrajectoryDisplayer;

    double m_current_x = 0;
    double m_current_y = 0;
    double m_current_speed = 0; // marginally better than having it initialized with junk
    double m_current_heading = 0;

    ros::Publisher m_helm_pub;
    ros::Publisher m_display_pub;

    ros::Subscriber m_controller_msgs_sub;
    ros::Subscriber m_position_sub;
    ros::Subscriber m_heading_sub;
    ros::Subscriber m_speed_sub;
    ros::Subscriber m_piloting_mode_sub;
    ros::Subscriber m_reference_trajectory_sub;
    ros::Subscriber m_disturbance_estimate_sub;

    ros::ServiceServer m_update_reference_trajectory_service;

    long m_TrajectoryNumber = 0;

    Controller* m_Controller;
    dynamic_reconfigure::Server<mpc::mpcConfig> m_Dynamic_Reconfigure_Server;

    std::atomic<bool> m_Enabled{}; // share visibility between ROS thread and MPC thread

    CurrentEstimator m_DisturbanceEstimator;
    std::pair<double, double> m_DisturbanceEstimate;

    std::ostream* m_Output;
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