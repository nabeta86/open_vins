/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * (ov360 fork) ROS1 scale-correction interface. See DERIVED_FROM.md.
 */

#ifndef OV_MSCKF_ROS1_SCALECALIBRATOR_H
#define OV_MSCKF_ROS1_SCALECALIBRATOR_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <topic_tools/shape_shifter.h>

#include <Eigen/Eigen>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "utils/ScaleUtils.h"

namespace ov_msckf {

class VioManager;

/**
 * @brief (ov360) ROS1 interface for correcting the scale of the estimator.
 *
 * The VIO's scale can be off (bad initialisation, spurious zero-velocity updates, accelerometer scale).
 * Three ways to correct it are offered; all of them end in VioManager::request_scale(), i.e. a similarity
 * rescale of the whole filter state (positions, velocity, landmarks, covariance), so that every output of
 * the node is in the corrected units from then on. Defaults leave the estimator untouched.
 *
 *   manual         ~scale_cmd (std_msgs/Float64): multiply by the given factor
 *   known distance ~scale_mark (std_msgs/Empty) remembers the current position; ~scale_distance (Float64,
 *                  metres actually travelled in a straight line since the mark) applies d / |p_now − p_mark|
 *   reference      ~scale_ref_topic (nav_msgs/Odometry, geometry_msgs/PoseStamped or
 *                  PoseWithCovarianceStamped, stamped in the same IMU clock as poseimu): pairs of (VIO,
 *                  reference) positions are collected and the Umeyama similarity scale is published on
 *                  ~scale_est as the absolute factor w.r.t. the raw estimator once the pairs are spread
 *                  over ~scale_min_span metres. With ~scale_auto:=true it is applied automatically whenever
 *                  it differs from the current scale by more than ~scale_auto_tol.
 *
 * Status is a JSON string on ~scale_status (latched): the total factor applied, the estimate and its
 * statistics, the mark, etc. ~scale_inject_at / ~scale_inject apply a known factor once at a given time
 * after the first published state (test hook for the simulation).
 */
class ScaleCalibrator {

public:
  ScaleCalibrator(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<VioManager> app);

  /// Called by the visualizer for every published state (IMU-clock timestamp, p_IinG after any correction)
  void on_state(double timestamp, const Eigen::Vector3d &p_IinG);

private:
  void cb_cmd(const std_msgs::Float64::ConstPtr &msg);
  void cb_mark(const std_msgs::Empty::ConstPtr &msg);
  void cb_distance(const std_msgs::Float64::ConstPtr &msg);
  void cb_ref(const topic_tools::ShapeShifter::ConstPtr &msg);

  /// Forward a factor to the estimator (validated there). mtx_ must be held.
  void request(double s, const std::string &why);
  /// Rescale the buffered VIO positions when the estimator has applied a factor. mtx_ must be held.
  void sync_with_app();
  /// Publish the JSON status (throttled unless force). mtx_ must be held.
  void publish_status(bool force);

  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<VioManager> app_;
  ros::Subscriber sub_cmd_, sub_mark_, sub_dist_, sub_ref_;
  ros::Publisher pub_est_, pub_status_;

  std::string ref_topic_;
  bool auto_apply_ = false;
  double auto_tol_ = 0.02;
  double match_tol_ = 0.05;
  double inject_at_ = -1.0, inject_ = 1.0;
  bool injected_ = false;
  double first_t_ = -1.0;

  std::mutex mtx_;
  std::deque<std::pair<double, Eigen::Vector3d>> hist_; // published states (t, p), kept consistent with the applied scale
  std::deque<std::pair<double, Eigen::Vector3d>> refs_; // reference positions waiting to be matched by time
  ScaleEstimator est_;
  double seen_total_ = 1.0;   // app_->scale_applied() the buffers are consistent with
  bool await_apply_ = false;  // a request is pending in the estimator
  bool mark_valid_ = false;
  Eigen::Vector3d mark_p_ = Eigen::Vector3d::Zero();
  bool have_est_ = false;
  double est_rel_ = 1.0, est_abs_ = 1.0, est_span_ = 0.0, est_rms_ = 0.0;
  int est_n_ = 0;
  std::string last_why_;
  double last_status_t_ = -1.0;
  int warned_type_ = 0;
};

} // namespace ov_msckf

#endif // OV_MSCKF_ROS1_SCALECALIBRATOR_H
