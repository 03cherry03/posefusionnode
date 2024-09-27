// // // #include <rclcpp/rclcpp.hpp>
// // // #include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
// // // #include <geometry_msgs/msg/twist_stamped.hpp>
// // // #include <tf2_ros/transform_broadcaster.h>
// // // #include <tf2/LinearMath/Quaternion.h>
// // // #include <Eigen/Dense>
// // // #include <cmath>
// // // #include <string>
// // // #include <sstream>
// // // #include <iomanip>

// // // class PoseFusionNode : public rclcpp::Node
// // // {
// // // public:
// // //     PoseFusionNode()
// // //         : Node("pose_fusion_node")
// // //     {
// // //         // Subscribers for LiDAR and GNSS pose
// // //         lidar_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
// // //             "/localization/pose_with_covariance", 10,
// // //             std::bind(&PoseFusionNode::lidarPoseCallback, this, std::placeholders::_1));

// // //         // gnss_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
// // //         //     "/fix_pose", 10,
// // //         //     std::bind(&PoseFusionNode::gnssPoseCallback, this, std::placeholders::_1));

// // //         gnss_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
// // //             "/mgrs_pose", 10,
// // //             std::bind(&PoseFusionNode::gnssPoseCallback, this, std::placeholders::_1));

// // //         // Subscribers for EKF and Filter twist
// // //         ekf_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
// // //             "/localization/pose_twist_fusion_filter/twist", 10,
// // //             std::bind(&PoseFusionNode::ekfTwistCallback, this, std::placeholders::_1));

// // //         filter_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
// // //             "/filter/twist", 10,
// // //             std::bind(&PoseFusionNode::filterTwistCallback, this, std::placeholders::_1));

// // //         // Publisher for final fused pose
// // //         final_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/fused/pose_with_covariance", 10);
// // //         fused_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/fused/twist_with_covariance", 10);

// // //         // Initialize the transform broadcaster
// // //         tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
// // //     }

// // // private:
// // //     void lidarPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr lidar_msg)
// // //     {
// // //         last_lidar_msg_ = lidar_msg;

// // //         if (last_gnss_msg_)
// // //         {
// // //             fusePoses();
// // //         }
// // //     }

// // //     void gnssPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr gnss_msg)
// // //     {
// // //         last_gnss_msg_ = gnss_msg;

// // //         if (last_lidar_msg_)
// // //         {
// // //             fusePoses();
// // //         }
// // //     }

// // //     void ekfTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr ekf_twist_msg)
// // //     {
// // //         last_ekf_twist_msg_ = ekf_twist_msg;

// // //         if (last_filter_twist_msg_)
// // //         {
// // //             fuseTwists();
// // //         }
// // //     }

// // //     void filterTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr filter_twist_msg)
// // //     {
// // //         last_filter_twist_msg_ = filter_twist_msg;

// // //         if (last_ekf_twist_msg_)
// // //         {
// // //             fuseTwists();
// // //         }
// // //     }

// // //     void fusePoses()
// // //     {
// // //         Eigen::Vector3d lidar_pos(last_lidar_msg_->pose.pose.position.x, last_lidar_msg_->pose.pose.position.y, last_lidar_msg_->pose.pose.position.z);
// // //         Eigen::Vector3d gnss_pos(last_gnss_msg_->pose.pose.position.x, last_gnss_msg_->pose.pose.position.y, last_gnss_msg_->pose.pose.position.z);

// // //         geometry_msgs::msg::PoseWithCovarianceStamped fused_pose;
// // //         fused_pose.header.stamp = this->now();
// // //         fused_pose.header.frame_id = "base_link";

// // //         fused_pose.pose.pose.position.x = lidar_weight_ * lidar_pos.x() + gnss_weight_ * gnss_pos.x();
// // //         fused_pose.pose.pose.position.y = lidar_weight_ * lidar_pos.y() + gnss_weight_ * gnss_pos.y();
// // //         fused_pose.pose.pose.position.z = lidar_weight_ * lidar_pos.z() + gnss_weight_ * gnss_pos.z();

// // //         fused_pose.pose.pose.orientation = last_lidar_msg_->pose.pose.orientation;

// // //         for (size_t i = 0; i < 36; ++i)
// // //         {
// // //             fused_pose.pose.covariance[i] = lidar_weight_ * last_lidar_msg_->pose.covariance[i] +
// // //                                             gnss_weight_ * last_gnss_msg_->pose.covariance[i];
// // //         }

// // //         final_pose_pub_->publish(fused_pose);

// // //         // Broadcast the transform
// // //         broadcastTransform(fused_pose);
// // //     }

// // //     void fuseTwists()
// // //     {
// // //         geometry_msgs::msg::TwistStamped fused_twist;
// // //         fused_twist.header.stamp = this->now();
// // //         fused_twist.header.frame_id = "base_link";  // Adjust frame_id as needed

// // //         fused_twist.twist.linear.x = 0.0;
// // //         fused_twist.twist.linear.y = 0.0;
// // //         fused_twist.twist.linear.z = 0.0;

// // //         fused_twist.twist.angular.x = 0.0;
// // //         fused_twist.twist.angular.y = 0.0;
// // //         fused_twist.twist.angular.z = ekf_twist_weight_ * last_ekf_twist_msg_->twist.angular.z + filter_twist_weight_ * last_filter_twist_msg_->twist.angular.z;

// // //         fused_twist_pub_->publish(fused_twist);
// // //     }

// // //     void broadcastTransform(const geometry_msgs::msg::PoseWithCovarianceStamped &fused_pose)
// // //     {
// // //         geometry_msgs::msg::TransformStamped transformStamped;

// // //         transformStamped.header.stamp = fused_pose.header.stamp;
// // //         transformStamped.header.frame_id = "map";  // Adjust this to the correct reference frame as needed
// // //         transformStamped.child_frame_id = "base_link";

// // //         transformStamped.transform.translation.x = fused_pose.pose.pose.position.x;
// // //         transformStamped.transform.translation.y = fused_pose.pose.pose.position.y;
// // //         transformStamped.transform.translation.z = fused_pose.pose.pose.position.z;

// // //         transformStamped.transform.rotation.x = fused_pose.pose.pose.orientation.x;
// // //         transformStamped.transform.rotation.y = fused_pose.pose.pose.orientation.y;
// // //         transformStamped.transform.rotation.z = fused_pose.pose.pose.orientation.z;
// // //         transformStamped.transform.rotation.w = fused_pose.pose.pose.orientation.w;

// // //         // Broadcast the transform
// // //         tf_broadcaster_->sendTransform(transformStamped);
// // //     }

// // //     rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr lidar_pose_sub_;
// // //     rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_pose_sub_;
// // //     rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr ekf_twist_sub_;
// // //     rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr filter_twist_sub_;
// // //     rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr final_pose_pub_;
// // //     rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr fused_twist_pub_;

// // //     std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

// // //     geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_lidar_msg_;
// // //     geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_gnss_msg_;
// // //     geometry_msgs::msg::TwistStamped::SharedPtr last_ekf_twist_msg_;
// // //     geometry_msgs::msg::TwistStamped::SharedPtr last_filter_twist_msg_;

// // //     double lidar_weight_ = 0.5; // Weight for LiDAR data
// // //     double gnss_weight_ = 0.5;  // Weight for GNSS data
// // //     double ekf_twist_weight_ = 0.5; // Weight for EKF twist data
// // //     double filter_twist_weight_ = 0.5; // Weight for Filter twist data
// // // };

// // // int main(int argc, char *argv[])
// // // {
// // //     rclcpp::init(argc, argv);
// // //     rclcpp::spin(std::make_shared<PoseFusionNode>());
// // //     rclcpp::shutdown();
// // //     return 0;
// // // }

// // /*
// //  * timerCallback
// //  */
// // void EKFLocalizer::timerCallback()
// // {
// //   if (!is_activated_) {
// //     warning_.warnThrottle(
// //       "The node is not activated. Provide initial pose to pose_initializer", 2000);
// //     publishDiagnostics();
// //     return;
// //   }

// //   DEBUG_INFO(get_logger(), "========================= timer called =========================");

// //   /* update predict frequency with measured timer rate */
// //   updatePredictFrequency();

// //   /* predict model in EKF */
// //   stop_watch_.tic();
// //   DEBUG_INFO(get_logger(), "------------------------- start prediction -------------------------");

// //   const Eigen::MatrixXd X_curr = ekf_.getLatestX();
// //   DEBUG_PRINT_MAT(X_curr.transpose());

// //   const Eigen::MatrixXd P_curr = ekf_.getLatestP();

// //   const double dt = ekf_dt_;

// //   const Vector6d X_next = predictNextState(X_curr, dt);
// //   const Matrix6d A = createStateTransitionMatrix(X_curr, dt);
// //   const Matrix6d Q = processNoiseCovariance(proc_cov_yaw_d_, proc_cov_vx_d_, proc_cov_wz_d_);

// //   ekf_.predictWithDelay(X_next, A, Q);

// //   // debug
// //   const Eigen::MatrixXd X_result = ekf_.getLatestX();
// //   DEBUG_PRINT_MAT(X_result.transpose());
// //   DEBUG_PRINT_MAT((X_result - X_curr).transpose());
// //   DEBUG_INFO(get_logger(), "[EKF] predictKinematicsModel calc time = %f [ms]", stop_watch_.toc());
// //   DEBUG_INFO(get_logger(), "------------------------- end prediction -------------------------\n");

// //   /* pose measurement fusion update */

// //   pose_queue_size_ = pose_queue_.size();
// //   pose_is_passed_delay_gate_ = true;
// //   pose_delay_time_ = 0.0;
// //   pose_delay_time_threshold_ = 0.0;
// //   pose_is_passed_mahalanobis_gate_ = true;
// //   pose_mahalanobis_distance_ = 0.0;

// //   bool pose_is_updated = false;

// //   if (!pose_queue_.empty()) {
// //     DEBUG_INFO(get_logger(), "------------------------- start Pose Fusion -------------------------");
// //     stop_watch_.tic();

// //     // Retrieve and fuse pose data from LIDAR and GNSS
// //     geometry_msgs::msg::PoseWithCovarianceStamped fused_pose;
// //     if (pose_lidar_ && pose_gnss_) {
// //       fused_pose = fusePoseData(*pose_lidar_, *pose_gnss_);
// //     }

// //     pub_pose_with_covariance_->publish(fused_pose);

// //     DEBUG_INFO(get_logger(), "[EKF] measurementUpdatePose fusion calc time = %f [ms]", stop_watch_.toc());
// //     DEBUG_INFO(get_logger(), "------------------------- end Pose Fusion -------------------------\n");
// //   }
// //   pose_no_update_count_ = pose_is_updated ? 0 : (pose_no_update_count_ + 1);

// //   /* twist measurement fusion update */

// //   twist_queue_size_ = twist_queue_.size();
// //   twist_is_passed_delay_gate_ = true;
// //   twist_delay_time_ = 0.0;
// //   twist_delay_time_threshold_ = 0.0;
// //   twist_is_passed_mahalanobis_gate_ = true;
// //   twist_mahalanobis_distance_ = 0.0;

// //   bool twist_is_updated = false;

// //   if (!twist_queue_.empty()) {
// //     DEBUG_INFO(get_logger(), "------------------------- start Twist Fusion -------------------------");
// //     stop_watch_.tic();

// //     // Retrieve and fuse twist data from LIDAR and GNSS
// //     geometry_msgs::msg::TwistWithCovarianceStamped fused_twist;
// //     if (twist_lidar_ && twist_gnss_) {
// //       fused_twist = fuseTwistData(*twist_lidar_, *twist_gnss_);
// //     }

// //     pub_twist_with_covariance_->publish(fused_twist);

// //     DEBUG_INFO(get_logger(), "[EKF] measurementUpdateTwist fusion calc time = %f [ms]", stop_watch_.toc());
// //     DEBUG_INFO(get_logger(), "------------------------- end Twist Fusion -------------------------\n");
// //   }
// //   twist_no_update_count_ = twist_is_updated ? 0 : (twist_no_update_count_ + 1);

// //   /* publish ekf result */
// //   publishEstimateResult();
// //   publishDiagnostics();
// // }

// // /* Function to fuse pose data */
// // geometry_msgs::msg::PoseWithCovarianceStamped EKFLocalizer::fusePoseData(
// //   const geometry_msgs::msg::PoseWithCovarianceStamped &lidar_pose,
// //   const geometry_msgs::msg::PoseWithCovarianceStamped &gnss_pose)
// // {
// //   geometry_msgs::msg::PoseWithCovarianceStamped fused_pose;

// //   // Example fusion logic: simple averaging of positions
// //   fused_pose.pose.pose.position.x = (lidar_pose.pose.pose.position.x + gnss_pose.pose.pose.position.x) / 2.0;
// //   fused_pose.pose.pose.position.y = (lidar_pose.pose.pose.position.y + gnss_pose.pose.pose.position.y) / 2.0;
// //   fused_pose.pose.pose.position.z = (lidar_pose.pose.pose.position.z + gnss_pose.pose.pose.position.z) / 2.0;

// //   // Quaternion blending or other methods for orientation can be applied here
// //   // Covariance fusion (for now, just copying lidar covariance as an example)
// //   fused_pose.pose.covariance = lidar_pose.pose.covariance;

// //   return fused_pose;
// // }

// // /* Function to fuse twist data */
// // geometry_msgs::msg::TwistWithCovarianceStamped EKFLocalizer::fuseTwistData(
// //   const geometry_msgs::msg::TwistWithCovarianceStamped &lidar_twist,
// //   const geometry_msgs::msg::TwistWithCovarianceStamped &gnss_twist)
// // {
// //   geometry_msgs::msg::TwistWithCovarianceStamped fused_twist;

// //   // Example fusion logic: simple averaging of velocities
// //   fused_twist.twist.twist.linear.x = (lidar_twist.twist.twist.linear.x + gnss_twist.twist.twist.linear.x) / 2.0;
// //   fused_twist.twist.twist.angular.z = (lidar_twist.twist.twist.angular.z + gnss_twist.twist.twist.angular.z) / 2.0;

// //   // Covariance fusion (for now, just copying lidar covariance as an example)
// //   fused_twist.twist.covariance = lidar_twist.twist.covariance;

// //   return fused_twist;
// // }


// #include "ekf_localizer/ekf_localizer.hpp"

// // Additional headers for subscribers
// #include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
// #include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

// // Declare member variables for fusion
// geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_lidar_;
// geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_gnss_;
// geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr twist_lidar_;
// geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr twist_gnss_;

// // Fusion weights (can be adjusted or calculated dynamically)
// double lidar_weight_ = 0.5;
// double gnss_weight_ = 0.5;

// EKFLocalizer::EKFLocalizer(const std::string & node_name, const rclcpp::NodeOptions & node_options)
// : rclcpp::Node(node_name, node_options),
//   warning_(this),
//   params_(this),
//   ekf_rate_(params_.ekf_rate),
//   ekf_dt_(params_.ekf_dt),
//   dim_x_(6 /* x, y, yaw, yaw_bias, vx, wz */),
//   pose_queue_(params_.pose_smoothing_steps),
//   twist_queue_(params_.twist_smoothing_steps)
// {
//   // Other initialization code...

//   // Create new subscribers for LIDAR and GNSS
//   sub_pose_with_cov_lidar_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
//     "/localization/pose_with_covariance_lidar", 1, 
//     std::bind(&EKFLocalizer::callbackPoseLidar, this, _1));

//   sub_pose_with_cov_gnss_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
//     "/localization/pose_with_covariance_gnss", 1, 
//     std::bind(&EKFLocalizer::callbackPoseGnss, this, _1));

//   sub_twist_with_cov_lidar_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
//     "/localization/twist_with_covariance_lidar", 1, 
//     std::bind(&EKFLocalizer::callbackTwistLidar, this, _1));

//   sub_twist_with_cov_gnss_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
//     "/localization/twist_with_covariance_gnss", 1, 
//     std::bind(&EKFLocalizer::callbackTwistGnss, this, _1));

//   // Other publishers for fused output
//   pub_fused_pose_cov_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
//     "/localization/pose_with_covariance", 1);
    
//   pub_fused_twist_cov_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
//     "/localization/twist_with_covariance", 1);
// }

// // Callback for LIDAR pose
// void EKFLocalizer::callbackPoseLidar(
//   const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
// {
//   pose_lidar_ = msg;
//   fusePose();
// }

// // Callback for GNSS pose
// void EKFLocalizer::callbackPoseGnss(
//   const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
// {
//   pose_gnss_ = msg;
//   fusePose();
// }

// // Callback for LIDAR twist
// void EKFLocalizer::callbackTwistLidar(
//   const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
// {
//   twist_lidar_ = msg;
//   fuseTwist();
// }

// // Callback for GNSS twist
// void EKFLocalizer::callbackTwistGnss(
//   const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
// {
//   twist_gnss_ = msg;
//   fuseTwist();
// }


// /* EKF 타이머 콜백 함수 */
// void EKFLocalizer::timerCallback()
// {
//   if (!is_activated_) {
//     warning_.warnThrottle(
//       "The node is not activated. Provide initial pose to pose_initializer", 2000);
//     publishDiagnostics();
//     return;
//   }

//   DEBUG_INFO(get_logger(), "========================= timer called =========================");

//   /* 예측 주기 업데이트 */
//   updatePredictFrequency();

//   /* EKF 예측 모델 실행 */
//   stop_watch_.tic();
//   DEBUG_INFO(get_logger(), "------------------------- start prediction -------------------------");

//   const Eigen::MatrixXd X_curr = ekf_.getLatestX();
//   DEBUG_PRINT_MAT(X_curr.transpose());

//   const Eigen::MatrixXd P_curr = ekf_.getLatestP();
//   const double dt = ekf_dt_;

//   const Vector6d X_next = predictNextState(X_curr, dt);
//   const Matrix6d A = createStateTransitionMatrix(X_curr, dt);
//   const Matrix6d Q = processNoiseCovariance(proc_cov_yaw_d_, proc_cov_vx_d_, proc_cov_wz_d_);

//   ekf_.predictWithDelay(X_next, A, Q);

//   // 포즈와 트위스트 데이터 융합을 EKF 주기와 맞춰 실행
//   fusePose();
//   fuseTwist();

//   DEBUG_INFO(get_logger(), "------------------------- end prediction -------------------------\n");

//   /* EKF 결과 퍼블리시 */
//   publishEstimateResult();
//   publishDiagnostics();
// }

// // Pose fusion logic
// void EKFLocalizer::fusePose()
// {
//   if (!pose_lidar_ || !pose_gnss_) return;

//   geometry_msgs::msg::PoseWithCovarianceStamped fused_pose;
//   fused_pose.header.stamp = this->now();
//   fused_pose.header.frame_id = "base_link";  // Adjust frame accordingly

//   // Fuse position
//   fused_pose.pose.pose.position.x = lidar_weight_ * pose_lidar_->pose.pose.position.x + gnss_weight_ * pose_gnss_->pose.pose.position.x;
//   fused_pose.pose.pose.position.y = lidar_weight_ * pose_lidar_->pose.pose.position.y + gnss_weight_ * pose_gnss_->pose.pose.position.y;
//   fused_pose.pose.pose.position.z = lidar_weight_ * pose_lidar_->pose.pose.position.z + gnss_weight_ * pose_gnss_->pose.pose.position.z;

//   // Fuse orientation (simple average, can be improved with quaternions)
//   fused_pose.pose.pose.orientation.x = lidar_weight_ * pose_lidar_->pose.pose.orientation.x + gnss_weight_ * pose_gnss_->pose.pose.orientation.x;
//   fused_pose.pose.pose.orientation.y = lidar_weight_ * pose_lidar_->pose.pose.orientation.y + gnss_weight_ * pose_gnss_->pose.pose.orientation.y;
//   fused_pose.pose.pose.orientation.z = lidar_weight_ * pose_lidar_->pose.pose.orientation.z + gnss_weight_ * pose_gnss_->pose.pose.orientation.z;
//   fused_pose.pose.pose.orientation.w = lidar_weight_ * pose_lidar_->pose.pose.orientation.w + gnss_weight_ * pose_gnss_->pose.pose.orientation.w;

//   // Fuse covariance (weighted average)
//   for (size_t i = 0; i < 36; ++i) {
//     fused_pose.pose.covariance[i] = lidar_weight_ * pose_lidar_->pose.covariance[i] + gnss_weight_ * pose_gnss_->pose.covariance[i];
//   }

//   // Publish fused pose
//   pub_fused_pose_cov_->publish(fused_pose);
// }

// // Twist fusion logic
// void EKFLocalizer::fuseTwist()
// {
//   if (!twist_lidar_ || !twist_gnss_) return;

//   geometry_msgs::msg::TwistWithCovarianceStamped fused_twist;
//   fused_twist.header.stamp = this->now();
//   fused_twist.header.frame_id = "base_link";  // Adjust frame accordingly

//   // Fuse linear velocity
//   fused_twist.twist.twist.linear.x = lidar_weight_ * twist_lidar_->twist.twist.linear.x + gnss_weight_ * twist_gnss_->twist.twist.linear.x;
//   fused_twist.twist.twist.linear.y = lidar_weight_ * twist_lidar_->twist.twist.linear.y + gnss_weight_ * twist_gnss_->twist.twist.linear.y;
//   fused_twist.twist.twist.linear.z = lidar_weight_ * twist_lidar_->twist.twist.linear.z + gnss_weight_ * twist_gnss_->twist.twist.linear.z;

//   // Fuse angular velocity
//   fused_twist.twist.twist.angular.x = lidar_weight_ * twist_lidar_->twist.twist.angular.x + gnss_weight_ * twist_gnss_->twist.twist.angular.x;
//   fused_twist.twist.twist.angular.y = lidar_weight_ * twist_lidar_->twist.twist.angular.y + gnss_weight_ * twist_gnss_->twist.twist.angular.y;
//   fused_twist.twist.twist.angular.z = lidar_weight_ * twist_lidar_->twist.twist.angular.z + gnss_weight_ * twist_gnss_->twist.twist.angular.z;

//   // Fuse covariance (weighted average)
//   for (size_t i = 0; i < 36; ++i) {
//     fused_twist.twist.covariance[i] = lidar_weight_ * twist_lidar_->twist.covariance[i] + gnss_weight_ * twist_gnss_->twist.covariance[i];
//   }

//   // Publish fused twist
//   pub_fused_twist_cov_->publish(fused_twist);
// }




// #include "pose_fusion_node.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>
#include <tier4_autoware_utils/ros/msg_covariance.hpp>


geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_lidar_;
geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_gnss_;
geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr twist_lidar_;
geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr twist_gnss_;

double lidar_weight_ = 0.5;
double gnss_weight_ = 0.5;

PoseFusionNode::PoseFusionNode(const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options),
  warning_(this),
  params_(this),
  ekf_rate_(params_.ekf_rate),
  ekf_dt_(params_.ekf_dt),
  dim_x_(6 /* x, y, yaw, yaw_bias, vx, wz */),
  pose_queue_(params_.pose_smoothing_steps),
  twist_queue_(params_.twist_smoothing_steps)
{
  sub_pose_with_cov_lidar_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    // "/localization/pose_with_covariance_lidar", 1, 
    "/localization/pose_with_covariance", 1, 
    std::bind(&PoseFusionNode::callbackPoseLidar, this, _1));

  sub_pose_with_cov_gnss_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    // "/localization/pose_with_covariance_gnss", 1, 
    "/mgrs/pose", 1, 
    std::bind(&PoseFusionNode::callbackPoseGnss, this, _1));

  sub_twist_with_cov_lidar_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "/localization/twist_with_covariance_lidar", 1, 
    std::bind(&PoseFusionNode::callbackTwistLidar, this, _1));

  sub_twist_with_cov_gnss_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "/localization/twist_with_covariance_gnss", 1, 
    std::bind(&PoseFusionNode::callbackTwistGnss, this, _1));

  pub_fused_pose_cov_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/fused/localization/pose_with_covariance", 1);
    
  pub_fused_twist_cov_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "/localization/twist_with_covariance", 1);

  initFusion();
}

void PoseFusionNode::callbackPoseLidar(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  pose_lidar_ = msg;
  fusePose();
}

void PoseFusionNode::callbackPoseGnss(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  pose_gnss_ = msg;
  fusePose();
}

void PoseFusionNode::callbackTwistLidar(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  twist_lidar_ = msg;
  fuseTwist();
}

void PoseFusionNode::callbackTwistGnss(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  twist_gnss_ = msg;
  fuseTwist();
}

void PoseFusionNode::timerCallback()
{
  if (!is_activated_) {
    warning_.warnThrottle(
      "The node is not activated. Provide initial pose to pose_initializer", 2000);
    publishDiagnostics();
    return;
  }

  updatePredictFrequency();

  stop_watch_.tic();

  const Eigen::MatrixXd X_curr = ekf_.getLatestX();

  const Eigen::MatrixXd P_curr = ekf_.getLatestP();
  const double dt = ekf_dt_;

  const Vector6d X_next = predictNextState(X_curr, dt);
  const Matrix6d A = createStateTransitionMatrix(X_curr, dt);
  const Matrix6d Q = processNoiseCovariance(proc_cov_yaw_d_, proc_cov_vx_d_, proc_cov_wz_d_);

  ekf_.predictWithDelay(X_next, A, Q);

  fusePose();
  fuseTwist();

  publishEstimateResult();
  publishDiagnostics();
}

void PoseFusionNode::fusePose()
{
  if (!pose_lidar_ || !pose_gnss_) return;

  geometry_msgs::msg::PoseWithCovarianceStamped fused_pose;
  fused_pose.header.stamp = this->now();
  fused_pose.header.frame_id = "base_link";

  fused_pose.pose.pose.position.x = lidar_weight_ * pose_lidar_->pose.pose.position.x +
                                    gnss_weight_ * pose_gnss_->pose.pose.position.x;
  fused_pose.pose.pose.position.y = lidar_weight_ * pose_lidar_->pose.pose.position.y +
                                    gnss_weight_ * pose_gnss_->pose.pose.position.y;
  fused_pose.pose.pose.position.z = lidar_weight_ * pose_lidar_->pose.pose.position.z +
                                    gnss_weight_ * pose_gnss_->pose.pose.position.z;

  fused_pose.pose.pose.orientation.x = lidar_weight_ * pose_lidar_->pose.pose.orientation.x +
                                       gnss_weight_ * pose_gnss_->pose.pose.orientation.x;
  fused_pose.pose.pose.orientation.y = lidar_weight_ * pose_lidar_->pose.pose.orientation.y +
                                       gnss_weight_ * pose_gnss_->pose.pose.orientation.y;
  fused_pose.pose.pose.orientation.z = lidar_weight_ * pose_lidar_->pose.pose.orientation.z +
                                       gnss_weight_ * pose_gnss_->pose.pose.orientation.z;
  fused_pose.pose.pose.orientation.w = lidar_weight_ * pose_lidar_->pose.pose.orientation.w +
                                       gnss_weight_ * pose_gnss_->pose.pose.orientation.w;

  for (size_t i = 0; i < 36; ++i) {
    fused_pose.pose.covariance[i] = lidar_weight_ * pose_lidar_->pose.covariance[i] +
                                    gnss_weight_ * pose_gnss_->pose.covariance[i];
  }

  pub_fused_pose_cov_->publish(fused_pose);
}

void PoseFusionNode::fuseTwist()
{
  if (!twist_lidar_ || !twist_gnss_) return;

  geometry_msgs::msg::TwistWithCovarianceStamped fused_twist;
  fused_twist.header.stamp = this->now();
  fused_twist.header.frame_id = "base_link";

  fused_twist.twist.twist.linear.x = lidar_weight_ * twist_lidar_->twist.twist.linear.x +
                                     gnss_weight_ * twist_gnss_->twist.twist.linear.x;
  fused_twist.twist.twist.linear.y = lidar_weight_ * twist_lidar_->twist.twist.linear.y +
                                     gnss_weight_ * twist_gnss_->twist.twist.linear.y;
  fused_twist.twist.twist.linear.z = lidar_weight_ * twist_lidar_->twist.twist.linear.z +
                                     gnss_weight_ * twist_gnss_->twist.twist.linear.z;

  fused_twist.twist.twist.angular.x = lidar_weight_ * twist_lidar_->twist.twist.angular.x +
                                      gnss_weight_ * twist_gnss_->twist.twist.angular.x;
  fused_twist.twist.twist.angular.y = lidar_weight_ * twist_lidar_->twist.twist.angular.y +
                                      gnss_weight_ * twist_gnss_->twist.twist.angular.y;
  fused_twist.twist.twist.angular.z = lidar_weight_ * twist_lidar_->twist.twist.angular.z +
                                      gnss_weight_ * twist_gnss_->twist.twist.angular.z;

  for (size_t i = 0; i < 36; ++i) {
    fused_twist.twist.covariance[i] = lidar_weight_ * twist_lidar_->twist.covariance[i] +
                                      gnss_weight_ * twist_gnss_->twist.covariance[i];
  }

  pub_fused_twist_cov_->publish(fused_twist);
}
