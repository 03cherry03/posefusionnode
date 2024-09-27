#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
      "/localization/twist_with_covariance_lidar", 10,
      std::bind(&PoseTwistFusionNode::callbackTwistLidar, this, std::placeholders::_1));

    sub_twist_gnss_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/localization/twist_with_covariance_gnss", 10,
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

    sub_pose_stamped_lidar_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/localization/pose_twist_fusion_filter/pose_lidar", 10,
      std::bind(&PoseTwistFusionNode::callbackPoseStampedLidar, this, std::placeholders::_1));

    sub_pose_stamped_gnss_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/localization/pose_twist_fusion_filter/pose_gnss", 10,
      std::bind(&PoseTwistFusionNode::callbackPoseStampedGnss, this, std::placeholders::_1));

    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/pose_with_covariance", 10);
    pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/localization/twist_with_covariance", 10);
    pub_kinematic_state_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/localization/pose_twist_fusion_filter/kinematic_state", 10);
    pub_biased_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/pose_twist_fusion_filter/biased_pose_with_covariance", 10);
    pub_pose_stamped_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/localization/pose_twist_fusion_filter/pose", 10);

    tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

private:
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_lidar_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_gnss_;
  geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr twist_lidar_;
  geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr twist_gnss_;

  nav_msgs::msg::Odometry::SharedPtr kinematic_state_lidar_;
  nav_msgs::msg::Odometry::SharedPtr kinematic_state_gnss_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr biased_pose_lidar_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr biased_pose_gnss_;
  geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped_lidar_;
  geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped_gnss_;

  void callbackPoseLidar(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    pose_lidar_ = msg;
    fusePoseTwist();
  }

  void callbackPoseGnss(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    pose_gnss_ = msg;
    fusePoseTwist();
  }

  void callbackTwistLidar(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    twist_lidar_ = msg;
    fusePoseTwist();
  }

  void callbackTwistGnss(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    twist_gnss_ = msg;
    fusePoseTwist();
  }

  void callbackKinematicStateLidar(const nav_msgs::msg::Odometry::SharedPtr msg) {
    kinematic_state_lidar_ = msg;
    fuseKinematicState();
  }

  void callbackKinematicStateGnss(const nav_msgs::msg::Odometry::SharedPtr msg) {
    kinematic_state_gnss_ = msg;
    fuseKinematicState();
  }

  void callbackBiasedPoseLidar(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    biased_pose_lidar_ = msg;
    fuseBiasedPose();
  }

  void callbackBiasedPoseGnss(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    biased_pose_gnss_ = msg;
    fuseBiasedPose();
  }

  void callbackPoseStampedLidar(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    pose_stamped_lidar_ = msg;
    fusePoseStamped();
  }

  void callbackPoseStampedGnss(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    pose_stamped_gnss_ = msg;
    fusePoseStamped();
  }

  void fusePoseTwist() {
    if (!pose_lidar_ || !pose_gnss_ || !twist_lidar_ || !twist_gnss_) {
      return;
    }

    // Pose와 Twist 데이터를 시간 기반 가중치를 사용하여 융합
    // 융합된 결과를 퍼블리시
    pub_pose_->publish(*pose_lidar_);  
    pub_twist_->publish(*twist_lidar_); 
  }

  void fuseKinematicState() {
    if (!kinematic_state_lidar_ || !kinematic_state_gnss_) {
      return;
    }

    nav_msgs::msg::Odometry fused_kinematic_state;
    fused_kinematic_state.header.stamp = this->now();
    fused_kinematic_state.header.frame_id = "map";
    fused_kinematic_state.pose = kinematic_state_lidar_->pose;
    fused_kinematic_state.twist = kinematic_state_lidar_->twist;

    pub_kinematic_state_->publish(fused_kinematic_state); 

  void fuseBiasedPose() {
    if (!biased_pose_lidar_ || !biased_pose_gnss_) {
      return;
    }

    pub_biased_pose_->publish(*biased_pose_lidar_); 
  }

  void fusePoseStamped() {
    if (!pose_stamped_lidar_ || !pose_stamped_gnss_) {
      return;
    }

    pub_pose_stamped_->publish(*pose_stamped_lidar_);  
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_lidar_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_gnss_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_twist_lidar_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_twist_gnss_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_kinematic_state_lidar_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_kinematic_state_gnss_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_biased_pose_lidar_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_biased_pose_gnss_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_stamped_lidar_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_stamped_gnss_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_kinematic_state_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_biased_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_stamped_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseTwistFusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
