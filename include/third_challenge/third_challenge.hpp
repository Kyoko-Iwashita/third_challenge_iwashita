#ifndef THIRD_CHALLENGE_HPP
#define THIRD_CHALLENGE_HPP

#include <rclcpp/rclcpp.hpp>
#include <functional>  // bind & placeholders用
#include <memory>      // SharedPtr用
#include <cmath> // 要らないかも
#include <image_geometry/pinhole_camera_model.h>
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "roomba_500driver_meiji/msg/roomba_ctrl.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include <tf2_ros/static_transform_broadcaster.h>

class thirdChallenge : public rclcpp::Node
{
    public:
        thirdChallenge();

        void box_callback(const std_msgs::msg::Float32MultiArray& msg);
        void camera_info_callback(const sensor_msgs::msg::CameraInfo& msg);
        void timer_callback();
        void broadcast_transform();

    private:
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_box_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
        rclcpp::Publisher<roomba_500driver_meiji::msg::RoombaCtrl>::SharedPtr cmd_vel_pub_;
        image_geometry::PinholeCameraModel camera_model_;
        roomba_500driver_meiji::msg::RoombaCtrl cmd_vel_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        geometry_msgs::msg::Quaternion odom_quat_;
        bool is_model_set = false;
        bool detected = false;

        int mode = 11;
        double frontal_threshold = 2.2;  // ?
        double target_yaw = 0.0;   // 目標角度を保持
        double base_omega;         // 角速度は0.5[rad/s]で固定
};

#endif  // THIRD_CHALLENGE_HPP