/**
 * @file rgbd-slam-node.hpp
 * @brief Definition of the RgbdSlamNode Wrapper class.
 * @author Suchetan R S (rssuchetan@gmail.com)
 */

#ifndef RGB_SLAM_NODE_HPP_
#define RGB_SLAM_NODE_HPP_

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/pose_array.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <slam_msgs/msg/map_data.hpp>
#include <slam_msgs/srv/get_map.hpp>

#include "type_conversion.hpp"
#include "orb_slam3_interface.hpp"

namespace ORB_SLAM3_Wrapper
{
    class RgbSlamNode : public rclcpp::Node
    {
    public:
        RgbSlamNode(const std::string &strVocFile,
                     const std::string &strSettingsFile,
                     ORB_SLAM3::System::eSensor sensor);
        ~RgbSlamNode();

    private:

        // ROS 2 Callbacks.
        void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msgIMU);
        void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msgOdom);
        void RGBCallback(const sensor_msgs::msg::Image::SharedPtr msgRGB);

        /**
         * @brief Publishes map data. (Keyframes and all poses in the current active map.)
         * @param orb_atlas Pointer to the Atlas object.
         * @param last_init_kf_id ID of the last initialized keyframe.
         */
        void publishMapData();

        void publishMapPointCloud();

        /**
         * @brief Callback function for GetMap service.
         * @param request_header Request header.
         * @param request Request message.
         * @param response Response message.
         */
        void getMapServer(std::shared_ptr<rmw_request_id_t> request_header,
                          std::shared_ptr<slam_msgs::srv::GetMap::Request> request,
                          std::shared_ptr<slam_msgs::srv::GetMap::Response> response);

        /**
         * Member variables
         */
        // RGB
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_sub;
        // ROS Publishers and Subscribers
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Publisher<slam_msgs::msg::MapData>::SharedPtr map_data_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub;
        rclcpp::Service<slam_msgs::srv::GetMap>::SharedPtr get_map_data_service;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        // ROS Params
        std::string robot_base_frame_id_;
        std::string odom_frame_id_;
        std::string global_frame_;
        double robot_x_, robot_y_;
        bool rosViz_;
        ORB_SLAM3_Wrapper::WrapperTypeConversions conversions;
        std::shared_ptr<ORB_SLAM3_Wrapper::ORBSLAM3Interface> interface;
        geometry_msgs::msg::TransformStamped tfMapOdom;
    };
}
#endif
