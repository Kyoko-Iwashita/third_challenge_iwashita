#include "third_challenge/third_challenge.hpp"

using namespace std::chrono_literals;

/*
    ・ros2_yoloから検出結果を受け取る
        ・topic: /ros2_yolo/box (Float32MultiArray)
            idx0: stamp.sec
            idx1: stamp.nanosec
            idx2~5: xmin, ymin, xmax, ymax (pixel 単位)
            idx6: reliability
            idx7: class
    ・検出結果を世界座標に変換する際は/color/camera_infoの情報を利用
        ・See: https://mem-archive.com/2018/10/13/post-682/
    ・検出結果をもとに人の方にルンバを旋回させる
        ・人の方位をルンバの旋回に依存しない座標系に変換し、保持
        ・タイマーを使って目標方位に旋回させる
*/
thirdChallenge::thirdChallenge() : Node("third_challenge")
{
    // グローバル変数を定義
    base_omega = this->declare_parameter<float>("base_omega", 0.5);

    // モータ制御用のパブリッシャ
    cmd_vel_pub_ = this->create_publisher<roomba_500driver_meiji::msg::RoombaCtrl>(
        "/roomba/control", rclcpp::QoS(10).reliable());

    // tf_buffer_ を初期化
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

    // tf_listener_ を tf_buffer_ に紐付けて初期化
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // tf_broadcaster_を初期化
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    // Tfをブロードキャスト
    broadcast_transform();

    // サブスクライバの登録
    sub_box_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/ros2_yolo/box", rclcpp::QoS(10).reliable(), std::bind(&thirdChallenge::box_callback, this, std::placeholders::_1));

    sub_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera/color/camera_info", rclcpp::QoS(10).reliable(), std::bind(&thirdChallenge::camera_info_callback, this, std::placeholders::_1));

    // タイマーの設定（50msごとにtimer_callbackを呼び出す）
    timer_ = this->create_wall_timer(50ms, std::bind(&thirdChallenge::timer_callback, this));

}

// 検出結果のバウンディングボックスから，odom座標系で表される目標角度を計算する
void thirdChallenge::box_callback(const std_msgs::msg::Float32MultiArray& msg)
{
    RCLCPP_INFO(this->get_logger(), "box_callback has been called!");  // box_callbackが呼ばれているか

    if (msg.data.size() < 8) {
       RCLCPP_ERROR(this->get_logger(), "Bounding box data is incomplete! Size: %zu", msg.data.size());
       detected = false;
       return;
    }

    detected = true;  // データサイズが８ならば、人を検出できたとみなす。

    // RCLCPP_INFO(this->get_logger(), "box_callback called. is_model_set = %d", is_model_set);

    if (!is_model_set) {
        RCLCPP_WARN(this->get_logger(), "Camera model is not set yet!");
        return;
    }

    // バウンディングボックスの中心座標 (ピクセル座標)
    float u = (msg.data[2] + msg.data[4]) / 2.0;  // xmin, xmax の平均
    float v = (msg.data[3] + msg.data[5]) / 2.0;  // ymin, ymax の平均

    // 深度 (Z値) の設定
    float Z = 1.0;  // 仮の値

    RCLCPP_INFO(this->get_logger(), "u = %f, v = %f, Z = %f/n", u, v, Z);

    // カメラ座標を計算
    geometry_msgs::msg::PointStamped camera_point;
    camera_point.header.frame_id = "camera_frame";
    camera_point.header.stamp = this->get_clock()->now();
    camera_point.point.x = (u - camera_model_.cx()) * Z / camera_model_.fx();
    camera_point.point.y = (v - camera_model_.cy()) * Z / camera_model_.fy();
    camera_point.point.z = Z;
    RCLCPP_INFO(this->get_logger(), "cmaera_point.point.x = %lf, camera_point.point.y = %lf, camera_point.point.z = %lf\n", camera_point.point.x, camera_point.point.y, camera_point.point.z);

    // Odom 座標に変換
    geometry_msgs::msg::PointStamped camera_point_stamped;
    camera_point_stamped.header.frame_id = "camera_link";  // カメラの座標系
    camera_point_stamped.header.stamp = this->now();
    camera_point_stamped.point.x = camera_point.point.x;
    camera_point_stamped.point.y = camera_point.point.y;
    camera_point_stamped.point.z = camera_point.point.z;

    RCLCPP_INFO(this->get_logger(), "Ready tf_buffer_\n");

    // たぶんrf_buffer_, tf_listener_は初期化された。
    if (!tf_buffer_) {
        RCLCPP_ERROR(this->get_logger(), "tf_buffer_ is nullptr before lookupTransform!");
    }else{
        RCLCPP_INFO(this->get_logger(), "tf_buffer_ and tf_listener_ initialized successfully!");
    }

    // `camera_link` から `odom` への変換を取得
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        // 現在の時刻を基に変換を取得
        transform_stamped = tf_buffer_->lookupTransform("odom", "camera_link", this->now());
        RCLCPP_INFO(this->get_logger(), "Transform received successfully!");

        // 変換を適用
        geometry_msgs::msg::PointStamped odom_point_stamped;
        tf2::doTransform(camera_point_stamped, odom_point_stamped, transform_stamped);
        RCLCPP_INFO(this->get_logger(), "odom_point_stamped.point.x = %.3lf, odom_point_stamped.point.y = %.3lf, odom_point_stamped.point.z = %.3lf\n", odom_point_stamped.point.x, odom_point_stamped.point.y, odom_point_stamped.point.z);

        // 目標角度を計算（atan2を使用）
        // target_yaw = std::atan2(odom_point_stamped.point.y, odom_point_stamped.point.x);
        target_yaw = atan2(odom_point_stamped.point.y, odom_point_stamped.point.x);


        RCLCPP_INFO(this->get_logger(), "目標角度: %f [rad]", target_yaw);

    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        return;
    }
}

// カメラパラメータを取得し，カメラモデルを生成する
void thirdChallenge::camera_info_callback(const sensor_msgs::msg::CameraInfo& msg)
{
    // sub_camera_info_ = *msg;
    if (!is_model_set) {
        // カメラ情報をセット
        camera_model_.fromCameraInfo(msg);
        is_model_set = true;  // モデル設定完了フラグを立てる
        RCLCPP_INFO(this->get_logger(), "カメラモデルを設定しました。");
    }

    RCLCPP_INFO(this->get_logger(), "fx: %f, fy: %f, cx: %f, cy: %f",
            camera_model_.fx(), camera_model_.fy(), camera_model_.cx(), camera_model_.cy());
}

void thirdChallenge::broadcast_transform()
{
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = this->get_clock()->now();
    transformStamped.header.frame_id = "odom";         // 親フレーム
    transformStamped.child_frame_id = "camera_link";  // 子フレーム

    // 仮の値でカメラの位置を設定（実際の値に変更してください）
    transformStamped.transform.translation.x = 0.0;  
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.5;  // 例えば、カメラが 0.5m 上にあると仮定

    tf2::Quaternion q;
    q.setRPY(M_PI / 2.0, 0.0, -M_PI / 2.0);
    q.normalize();  // クォータニオンの正規化

    // 回転なし（単位クォータニオン）
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    // TFをブロードキャスト
    tf_broadcaster_->sendTransform(transformStamped);
}

// 一定の周期で呼び出され，制御指令を生成する関数
// roombaを目標方位角の方に旋回させる
void thirdChallenge::timer_callback()
{
    RCLCPP_INFO(this->get_logger(), "timer_callback is running");
    RCLCPP_INFO(this->get_logger(), "detected = %d", detected);

    if (!detected) {
        RCLCPP_WARN(this->get_logger(), "No target detected. Stopping Roomba.");
        roomba_500driver_meiji::msg::RoombaCtrl stop_msg;
        stop_msg.cntl.linear.x = 0.0;
        stop_msg.cntl.angular.z = 0.0;
        cmd_vel_pub_->publish(stop_msg);
        return;
    }

    // 目標角度との差を計算（現在の向きは考慮せず）
    double yaw_error = target_yaw;
    while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
    RCLCPP_INFO(this->get_logger(), "yaw_error = %lf\n", yaw_error);

    // 角速度を base_omega に固定（0.5 rad/s）
    // double angular_speed = (0 < std::abs(yaw_error) <= 2.0) ? base_omega : -base_omega;
    // double angular_speed;  
    if (std::abs(yaw_error) < 2.8)
    {
        // angular_speed = base_omega;
        // 速度指令を発行
        if (yaw_error < 0.0)
        {
            cmd_vel_.mode = 11;
            cmd_vel_.cntl.linear.x = 0.0;
            cmd_vel_.cntl.angular.z = -base_omega;
            RCLCPP_INFO(this->get_logger(), "base_omega_1 = %lf\n", base_omega);
            cmd_vel_pub_->publish(cmd_vel_);
        }else if(0.0 <= yaw_error){
            cmd_vel_.mode = 11;
            cmd_vel_.cntl.linear.x = 0.0;
            cmd_vel_.cntl.angular.z = base_omega;
            RCLCPP_INFO(this->get_logger(), "base_omega_1 = %lf\n", base_omega);
            cmd_vel_pub_->publish(cmd_vel_);
        }
    }else if (2.8 <= std::abs(yaw_error))  // 目標角度が閾値範囲内ならば停止
    {
        double stop_omega = 0.0;
        RCLCPP_INFO(this->get_logger(), "Target aligned. Stopping rotation. stop_omeaga = %lf\n", stop_omega);
        // roomba_500driver_meiji::msg::RoombaCtrl stop_msg;
        cmd_vel_.mode = 11;
        cmd_vel_.cntl.linear.x = 0.0;
        cmd_vel_.cntl.angular.z = stop_omega;
        cmd_vel_pub_->publish(cmd_vel_);
    }   
}
