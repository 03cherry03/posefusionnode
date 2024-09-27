#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <deque>

class PoseTwistFusionNode : public rclcpp::Node {
public:
  PoseTwistFusionNode() : Node("pose_twist_fusion_node") {

    sub_pose_lidar_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/pose_with_covariance_lidar", 10,
      std::bind(&PoseTwistFusionNode::callbackPoseLidar, this, std::placeholders::_1));
    sub_pose_gnss_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/pose_with_covariance_gnss", 10,
      std::bind(&PoseTwistFusionNode::callbackPoseGnss, this, std::placeholders::_1));
    sub_twist_lidar_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/localization/pose_twist_fusion_filter/twist_with_covariance_lidar", 10,
      std::bind(&PoseTwistFusionNode::callbackTwistLidar, this, std::placeholders::_1));
    sub_twist_gnss_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/localization/pose_twist_fusion_filter/twist_with_covariance_gnss", 10,
      std::bind(&PoseTwistFusionNode::callbackTwistGnss, this, std::placeholders::_1));
    sub_kinematic_state_lidar_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/localization/pose_twist_fusion_filter/kinematic_state_lidar", 10,
      std::bind(&PoseTwistFusionNode::callbackKinematicStateLidar, this, std::placeholders::_1));
    sub_kinematic_state_gnss_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/localization/pose_twist_fusion_filter/kinematic_state_gnss", 10,
      std::bind(&PoseTwistFusionNode::callbackKinematicStateGnss, this, std::placeholders::_1));
    sub_biased_pose_lidar_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/pose_twist_fusion_filter/biased_pose_with_covariance_lidar", 10,
      std::bind(&PoseTwistFusionNode::callbackBiasedPoseLidar, this, std::placeholders::_1));
    sub_biased_pose_gnss_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/pose_twist_fusion_filter/biased_pose_with_covariance_gnss", 10,
      std::bind(&PoseTwistFusionNode::callbackBiasedPoseGnss, this, std::placeholders::_1));
    sub_pose_lidar_simple_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/localization/pose_twist_fusion_filter/pose_lidar", 10,
      std::bind(&PoseTwistFusionNode::callbackPoseLidarSimple, this, std::placeholders::_1));
    sub_pose_gnss_simple_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/localization/pose_twist_fusion_filter/pose_gnss", 10,
      std::bind(&PoseTwistFusionNode::callbackPoseGnssSimple, this, std::placeholders::_1));

    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/fused/localization/pose_with_covariance", 10);
    pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/localization/pose_twist_fusion_filter/twist_with_covariance", 10);
    pub_fused_kinematic_state_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/localization/pose_twist_fusion_filter/kinematic_state", 10);
    pub_fused_biased_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/pose_twist_fusion_filter/biased_pose_with_covariance", 10);
    pub_fused_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/localization/pose_twist_fusion_filter/pose", 10);

    tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    max_buffer_size_ = 15;
  }

private:
  // LiDAR & GNSS data buffer
  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> pose_lidar_buffer_;
  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> pose_gnss_buffer_;
  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> twist_lidar_buffer_;
  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> twist_gnss_buffer_;
  std::deque<nav_msgs::msg::Odometry::SharedPtr> kinematic_state_lidar_buffer_;
  std::deque<nav_msgs::msg::Odometry::SharedPtr> kinematic_state_gnss_buffer_;
  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> biased_pose_lidar_buffer_;
  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> biased_pose_gnss_buffer_;
  std::deque<geometry_msgs::msg::PoseStamped::SharedPtr> pose_lidar_simple_buffer_;
  std::deque<geometry_msgs::msg::PoseStamped::SharedPtr> pose_gnss_simple_buffer_;

  size_t max_buffer_size_;

  // callback
  void callbackPoseLidar(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    pose_lidar_buffer_.push_back(msg);
    if (pose_lidar_buffer_.size() > max_buffer_size_) {
      pose_lidar_buffer_.pop_front();
    }
    fusePose();
  }

  void callbackPoseGnss(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    pose_gnss_buffer_.push_back(msg);
    if (pose_gnss_buffer_.size() > max_buffer_size_) {
      pose_gnss_buffer_.pop_front();
    }
    fusePose();
  }

  void callbackTwistLidar(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    twist_lidar_buffer_.push_back(msg);
    if (twist_lidar_buffer_.size() > max_buffer_size_) {
      twist_lidar_buffer_.pop_front();
    }
    fuseTwist();
  }

  void callbackTwistGnss(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    twist_gnss_buffer_.push_back(msg);
    if (twist_gnss_buffer_.size() > max_buffer_size_) {
      twist_gnss_buffer_.pop_front();
    }
    fuseTwist();
  }

  void callbackKinematicStateLidar(const nav_msgs::msg::Odometry::SharedPtr msg) {
    kinematic_state_lidar_buffer_.push_back(msg);
    if (kinematic_state_lidar_buffer_.size() > max_buffer_size_) {
      kinematic_state_lidar_buffer_.pop_front();
    }
    fuseOdometry();
  }

  void callbackKinematicStateGnss(const nav_msgs::msg::Odometry::SharedPtr msg) {
    kinematic_state_gnss_buffer_.push_back(msg);
    if (kinematic_state_gnss_buffer_.size() > max_buffer_size_) {
      kinematic_state_gnss_buffer_.pop_front();
    }
    fuseOdometry();
  }

  void callbackBiasedPoseLidar(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    biased_pose_lidar_buffer_.push_back(msg);
    if (biased_pose_lidar_buffer_.size() > max_buffer_size_) {
      biased_pose_lidar_buffer_.pop_front();
    }
    fuseBiasedPose();
  }

  void callbackBiasedPoseGnss(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    biased_pose_gnss_buffer_.push_back(msg);
    if (biased_pose_gnss_buffer_.size() > max_buffer_size_) {
      biased_pose_gnss_buffer_.pop_front();
    }
    fuseBiasedPose();
  }

  void callbackPoseLidarSimple(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    pose_lidar_simple_buffer_.push_back(msg);
    if (pose_lidar_simple_buffer_.size() > max_buffer_size_) {
      pose_lidar_simple_buffer_.pop_front();
    }
    fuseSimplePose();
  }

  void callbackPoseGnssSimple(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    pose_gnss_simple_buffer_.push_back(msg);
    if (pose_gnss_simple_buffer_.size() > max_buffer_size_) {
      pose_gnss_simple_buffer_.pop_front();
    }
    fuseSimplePose();
  }

  //fuse data
  void fusePose() {
    if (pose_lidar_buffer_.empty() || pose_gnss_buffer_.empty()) return;

    auto latest_lidar_pose = pose_lidar_buffer_.back();
    auto closest_gnss_pose = findClosestPose(pose_gnss_buffer_, latest_lidar_pose->header.stamp);
    if (!closest_gnss_pose) return;

    geometry_msgs::msg::PoseWithCovarianceStamped fused_pose;
    fused_pose.header.stamp = this->now();
    fused_pose.header.frame_id = "map";

    double weight_lidar = 0.5;
    double weight_gnss = 0.5;

    // 위치 융합
    fused_pose.pose.pose.position.x = latest_lidar_pose->pose.pose.position.x * weight_lidar +
                                      closest_gnss_pose->pose.pose.position.x * weight_gnss;
    fused_pose.pose.pose.position.y = latest_lidar_pose->pose.pose.position.y * weight_lidar +
                                      closest_gnss_pose->pose.pose.position.y * weight_gnss;
    fused_pose.pose.pose.position.z = latest_lidar_pose->pose.pose.position.z * weight_lidar +
                                      closest_gnss_pose->pose.pose.position.z * weight_gnss;

    // 방향(쿼터니언) 융합
    fused_pose.pose.pose.orientation = fuseOrientation(
      latest_lidar_pose->pose.pose.orientation, closest_gnss_pose->pose.pose.orientation, weight_lidar, weight_gnss);

    // 공분산 융합 (여기서 fuseCovariance 사용)
    fused_pose.pose.covariance = fuseCovariance(
      latest_lidar_pose->pose.covariance, closest_gnss_pose->pose.covariance, weight_lidar, weight_gnss);

    pub_pose_->publish(fused_pose);
    broadcastTransform(fused_pose);
  }

  void fuseTwist() {
    if (twist_lidar_buffer_.empty() || twist_gnss_buffer_.empty()) return;

    auto latest_lidar_twist = twist_lidar_buffer_.back();
    auto closest_gnss_twist = findClosestTwist(twist_gnss_buffer_, latest_lidar_twist->header.stamp);
    if (!closest_gnss_twist) return;

    geometry_msgs::msg::TwistWithCovarianceStamped fused_twist;
    fused_twist.header.stamp = this->now();
    fused_twist.header.frame_id = "base_link";

    double weight_lidar = 0.5;
    double weight_gnss = 0.5;

    // 속도 융합
    fused_twist.twist.twist.linear.x = latest_lidar_twist->twist.twist.linear.x * weight_lidar +
                                      closest_gnss_twist->twist.twist.linear.x * weight_gnss;
    fused_twist.twist.twist.angular.z = latest_lidar_twist->twist.twist.angular.z * weight_lidar +
                                        closest_gnss_twist->twist.twist.angular.z * weight_gnss;

    // 공분산 융합 (여기서 fuseCovariance 사용)
    fused_twist.twist.covariance = fuseCovariance(
      latest_lidar_twist->twist.covariance, closest_gnss_twist->twist.covariance, weight_lidar, weight_gnss);

    pub_twist_->publish(fused_twist);
  }

  void fuseOdometry() {
    if (kinematic_state_lidar_buffer_.empty() || kinematic_state_gnss_buffer_.empty()) return;

    auto latest_lidar_odom = kinematic_state_lidar_buffer_.back();
    auto closest_gnss_odom = findClosestOdometry(kinematic_state_gnss_buffer_, latest_lidar_odom->header.stamp);
    if (!closest_gnss_odom) return;

    nav_msgs::msg::Odometry fused_odom;
    fused_odom.header.stamp = this->now();
    fused_odom.header.frame_id = "map";

    double weight_lidar = 0.5;
    double weight_gnss = 0.5;

    fused_odom.pose.pose.position.x = latest_lidar_odom->pose.pose.position.x * weight_lidar +
                                      closest_gnss_odom->pose.pose.position.x * weight_gnss;
    fused_odom.twist.twist.linear.x = latest_lidar_odom->twist.twist.linear.x * weight_lidar +
                                      closest_gnss_odom->twist.twist.linear.x * weight_gnss;

    pub_fused_kinematic_state_->publish(fused_odom);
  }

  void fuseBiasedPose() {
    if (biased_pose_lidar_buffer_.empty() || biased_pose_gnss_buffer_.empty()) {
      return;
    }

    auto latest_lidar_pose = biased_pose_lidar_buffer_.back();
    auto closest_gnss_pose = findClosestPose(biased_pose_gnss_buffer_, latest_lidar_pose->header.stamp);

    if (!closest_gnss_pose) {
      RCLCPP_WARN(this->get_logger(), "No matching GNSS biased pose found.");
      return;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped fused_biased_pose;
    fused_biased_pose.header.stamp = this->now();
    fused_biased_pose.header.frame_id = "map";

    // LiDAR weight & GNSS weight
    double weight_lidar = 0.5;
    double weight_gnss = 0.5;

    fused_biased_pose.pose.pose.position.x = latest_lidar_pose->pose.pose.position.x * weight_lidar +
                                             closest_gnss_pose->pose.pose.position.x * weight_gnss;
    fused_biased_pose.pose.pose.position.y = latest_lidar_pose->pose.pose.position.y * weight_lidar +
                                             closest_gnss_pose->pose.pose.position.y * weight_gnss;
    fused_biased_pose.pose.pose.position.z = latest_lidar_pose->pose.pose.position.z * weight_lidar +
                                             closest_gnss_pose->pose.pose.position.z * weight_gnss;

    fused_biased_pose.pose.pose.orientation = fuseOrientation(
      latest_lidar_pose->pose.pose.orientation, closest_gnss_pose->pose.pose.orientation, weight_lidar, weight_gnss);

    fused_biased_pose.pose.covariance = fuseCovariance(
      latest_lidar_pose->pose.covariance, closest_gnss_pose->pose.covariance, weight_lidar, weight_gnss);

    pub_fused_biased_pose_->publish(fused_biased_pose);
  }

  void fuseSimplePose() {
    if (pose_lidar_simple_buffer_.empty() || pose_gnss_simple_buffer_.empty()) {
      return;
    }

    auto latest_lidar_pose = pose_lidar_simple_buffer_.back();
    auto closest_gnss_pose = findClosestPoseSimple(pose_gnss_simple_buffer_, latest_lidar_pose->header.stamp);

    if (!closest_gnss_pose) {
      RCLCPP_WARN(this->get_logger(), "No matching GNSS simple pose found.");
      return;
    }

    geometry_msgs::msg::PoseStamped fused_pose;
    fused_pose.header.stamp = this->now();
    fused_pose.header.frame_id = "map";

    // LiDAR weight & GNSS weight
    double weight_lidar = 0.5;
    double weight_gnss = 0.5;

    fused_pose.pose.position.x = latest_lidar_pose->pose.position.x * weight_lidar +
                                closest_gnss_pose->pose.position.x * weight_gnss;
    fused_pose.pose.position.y = latest_lidar_pose->pose.position.y * weight_lidar +
                                closest_gnss_pose->pose.position.y * weight_gnss;
    fused_pose.pose.position.z = latest_lidar_pose->pose.position.z * weight_lidar +
                                closest_gnss_pose->pose.position.z * weight_gnss;

    fused_pose.pose.orientation = fuseOrientation(
      latest_lidar_pose->pose.orientation, closest_gnss_pose->pose.orientation, weight_lidar, weight_gnss);

    // Publish the fused PoseStamped message
    pub_fused_pose_->publish(fused_pose);
  }

  // Utility methods
  geometry_msgs::msg::Quaternion fuseOrientation(const geometry_msgs::msg::Quaternion& q1, const geometry_msgs::msg::Quaternion& q2, double w1, double w2) {
    tf2::Quaternion quat1, quat2;
    tf2::fromMsg(q1, quat1);
    tf2::fromMsg(q2, quat2);
    tf2::Quaternion fused_quat = quat1 * w1 + quat2 * w2;
    fused_quat.normalize();
    return tf2::toMsg(fused_quat);
  }

  std::array<double, 36> fuseCovariance(const std::array<double, 36>& cov1, const std::array<double, 36>& cov2, double w1, double w2) {
    std::array<double, 36> fused_cov;
    for (size_t i = 0; i < 36; ++i) {
      fused_cov[i] = cov1[i] * w1 + cov2[i] * w2;
    }
    return fused_cov;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr findClosestPose(
    const std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr>& buffer,
    const builtin_interfaces::msg::Time& target_time) {
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr closest_msg = nullptr;
    double min_time_diff = std::numeric_limits<double>::max();

    rclcpp::Time target_time_rclcpp(target_time);
    for (const auto& msg : buffer) {
      rclcpp::Time msg_time(msg->header.stamp);
      double time_diff = std::abs((msg_time - target_time_rclcpp).seconds());
      if (time_diff < min_time_diff) {
        min_time_diff = time_diff;
        closest_msg = msg;
      }
    }
    return closest_msg;
  }

  geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr findClosestTwist(
    const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr>& buffer,
    const builtin_interfaces::msg::Time& target_time) {
    geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr closest_msg = nullptr;
    double min_time_diff = std::numeric_limits<double>::max();

    rclcpp::Time target_time_rclcpp(target_time);
    for (const auto& msg : buffer) {
      rclcpp::Time msg_time(msg->header.stamp);
      double time_diff = std::abs((msg_time - target_time_rclcpp).seconds());
      if (time_diff < min_time_diff) {
        min_time_diff = time_diff;
        closest_msg = msg;
      }
    }
    return closest_msg;
  }

  nav_msgs::msg::Odometry::SharedPtr findClosestOdometry(
    const std::deque<nav_msgs::msg::Odometry::SharedPtr>& buffer,
    const builtin_interfaces::msg::Time& target_time) {
    nav_msgs::msg::Odometry::SharedPtr closest_msg = nullptr;
    double min_time_diff = std::numeric_limits<double>::max();

    rclcpp::Time target_time_rclcpp(target_time);
    for (const auto& msg : buffer) {
      rclcpp::Time msg_time(msg->header.stamp);
      double time_diff = std::abs((msg_time - target_time_rclcpp).seconds());
      if (time_diff < min_time_diff) {
        min_time_diff = time_diff;
        closest_msg = msg;
      }
    }
    return closest_msg;
  }

  geometry_msgs::msg::PoseStamped::SharedPtr findClosestPoseSimple(
    const std::deque<geometry_msgs::msg::PoseStamped::SharedPtr>& buffer,
    const builtin_interfaces::msg::Time& target_time) {
    geometry_msgs::msg::PoseStamped::SharedPtr closest_msg = nullptr;
    double min_time_diff = std::numeric_limits<double>::max();

    rclcpp::Time target_time_rclcpp(target_time);

    for (const auto& msg : buffer) {
      rclcpp::Time msg_time(msg->header.stamp);
      double time_diff = std::abs((msg_time - target_time_rclcpp).seconds());
      if (time_diff < min_time_diff) {
        min_time_diff = time_diff;
        closest_msg = msg;
      }
    }
    return closest_msg;
  }

  void broadcastTransform(const geometry_msgs::msg::PoseWithCovarianceStamped& pose) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = pose.header.stamp;
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "base_link";
    transform_stamped.transform.translation.x = pose.pose.pose.position.x;
    transform_stamped.transform.translation.y = pose.pose.pose.position.y;
    transform_stamped.transform.translation.z = pose.pose.pose.position.z;
    transform_stamped.transform.rotation = pose.pose.pose.orientation;
    tf_br_->sendTransform(transform_stamped);
  }

  // sub
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_lidar_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_gnss_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_twist_lidar_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_twist_gnss_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_kinematic_state_lidar_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_kinematic_state_gnss_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_biased_pose_lidar_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_biased_pose_gnss_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_lidar_simple_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_gnss_simple_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_fused_kinematic_state_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_fused_biased_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_fused_pose_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseTwistFusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
