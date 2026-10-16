/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * (ov360 fork) ROS1 scale-correction interface. See DERIVED_FROM.md.
 */

#include "ScaleCalibrator.h"

#include "core/VioManager.h"
#include "utils/colors.h"
#include "utils/print.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include <cmath>
#include <cstdio>

using namespace ov_msckf;

ScaleCalibrator::ScaleCalibrator(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<VioManager> app) : nh_(nh), app_(app) {

  double min_span = 1.0, min_step = 0.02;
  int min_pairs = 30, max_pairs = 600;
  nh_->param<std::string>("scale_ref_topic", ref_topic_, "");
  nh_->param<bool>("scale_auto", auto_apply_, false);
  nh_->param<double>("scale_auto_tol", auto_tol_, 0.02);
  nh_->param<double>("scale_min_span", min_span, 1.0);
  nh_->param<int>("scale_min_pairs", min_pairs, 30);
  nh_->param<int>("scale_max_pairs", max_pairs, 600);
  nh_->param<double>("scale_min_step", min_step, 0.02);
  nh_->param<double>("scale_match_tol", match_tol_, 0.05);
  nh_->param<double>("scale_inject_at", inject_at_, -1.0);
  nh_->param<double>("scale_inject", inject_, 1.0);
  est_ = ScaleEstimator(min_span, min_pairs, max_pairs, min_step);

  pub_est_ = nh_->advertise<std_msgs::Float64>("scale_est", 2);
  pub_status_ = nh_->advertise<std_msgs::String>("scale_status", 2, true);
  sub_cmd_ = nh_->subscribe("scale_cmd", 5, &ScaleCalibrator::cb_cmd, this);
  sub_mark_ = nh_->subscribe("scale_mark", 5, &ScaleCalibrator::cb_mark, this);
  sub_dist_ = nh_->subscribe("scale_distance", 5, &ScaleCalibrator::cb_distance, this);
  if (!ref_topic_.empty()) {
    sub_ref_ = nh_->subscribe<topic_tools::ShapeShifter>(ref_topic_, 50, &ScaleCalibrator::cb_ref, this);
  }
  PRINT_INFO("[SCALE]: commands %s %s %s, reference '%s', auto=%d (tol %.3f), min_span=%.2f m, min_pairs=%d, inject=%.3f@%.1fs\n",
             sub_cmd_.getTopic().c_str(), sub_mark_.getTopic().c_str(), sub_dist_.getTopic().c_str(), ref_topic_.c_str(), (int)auto_apply_,
             auto_tol_, min_span, min_pairs, inject_, inject_at_);
  std::lock_guard<std::mutex> lock(mtx_);
  publish_status(true);
}

void ScaleCalibrator::cb_cmd(const std_msgs::Float64::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  request(msg->data, "cmd");
}

void ScaleCalibrator::cb_mark(const std_msgs::Empty::ConstPtr &msg) {
  (void)msg;
  std::lock_guard<std::mutex> lock(mtx_);
  if (hist_.empty()) {
    PRINT_WARNING(YELLOW "[SCALE]: mark ignored, no state published yet\n" RESET);
    return;
  }
  mark_p_ = hist_.back().second;
  mark_valid_ = true;
  PRINT_INFO("[SCALE]: mark at (%.3f, %.3f, %.3f)\n", mark_p_(0), mark_p_(1), mark_p_(2));
  publish_status(true);
}

void ScaleCalibrator::cb_distance(const std_msgs::Float64::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  double d = msg->data;
  if (!mark_valid_ || hist_.empty()) {
    PRINT_WARNING(YELLOW "[SCALE]: distance ignored, set a mark first (scale_mark)\n" RESET);
    return;
  }
  if (!std::isfinite(d) || d <= 0.0) {
    PRINT_WARNING(YELLOW "[SCALE]: distance ignored, invalid value %.3f\n" RESET, d);
    return;
  }
  double dv = (hist_.back().second - mark_p_).norm();
  if (dv < 0.05) {
    PRINT_WARNING(YELLOW "[SCALE]: distance ignored, moved only %.3f m since the mark\n" RESET, dv);
    return;
  }
  PRINT_INFO("[SCALE]: known distance %.3f m, estimator moved %.3f m -> factor x%.4f\n", d, dv, d / dv);
  request(d / dv, "distance");
}

void ScaleCalibrator::cb_ref(const topic_tools::ShapeShifter::ConstPtr &msg) {
  double t = 0.0;
  Eigen::Vector3d p;
  const std::string &type = msg->getDataType();
  if (type == "nav_msgs/Odometry") {
    auto m = msg->instantiate<nav_msgs::Odometry>();
    t = m->header.stamp.toSec();
    p << m->pose.pose.position.x, m->pose.pose.position.y, m->pose.pose.position.z;
  } else if (type == "geometry_msgs/PoseStamped") {
    auto m = msg->instantiate<geometry_msgs::PoseStamped>();
    t = m->header.stamp.toSec();
    p << m->pose.position.x, m->pose.position.y, m->pose.position.z;
  } else if (type == "geometry_msgs/PoseWithCovarianceStamped") {
    auto m = msg->instantiate<geometry_msgs::PoseWithCovarianceStamped>();
    t = m->header.stamp.toSec();
    p << m->pose.pose.position.x, m->pose.pose.position.y, m->pose.pose.position.z;
  } else {
    if (warned_type_++ == 0)
      PRINT_WARNING(YELLOW "[SCALE]: reference topic has unsupported type %s\n" RESET, type.c_str());
    return;
  }
  if (!p.allFinite())
    return;
  std::lock_guard<std::mutex> lock(mtx_);
  refs_.emplace_back(t, p);
  while (refs_.size() > 400)
    refs_.pop_front();
}

void ScaleCalibrator::request(double s, const std::string &why) {
  if (!std::isfinite(s) || s <= 0.0 || s > 1.0e4) {
    PRINT_WARNING(YELLOW "[SCALE]: %s: invalid factor %.6f ignored\n" RESET, why.c_str(), s);
    return;
  }
  last_why_ = why;
  await_apply_ = true;
  app_->request_scale(s);
  publish_status(true);
}

void ScaleCalibrator::sync_with_app() {
  double tot = app_->scale_applied();
  if (tot == seen_total_)
    return;
  double r = tot / seen_total_;
  for (auto &h : hist_)
    h.second *= r;
  est_.rescale(r);
  if (mark_valid_)
    mark_p_ *= r;
  seen_total_ = tot;
  await_apply_ = false;
}

void ScaleCalibrator::on_state(double timestamp, const Eigen::Vector3d &p_IinG) {
  std::lock_guard<std::mutex> lock(mtx_);
  sync_with_app();
  if (first_t_ < 0.0)
    first_t_ = timestamp;
  hist_.emplace_back(timestamp, p_IinG);
  while (hist_.size() > 4000)
    hist_.pop_front();

  // Test hook: apply a known factor once
  if (!injected_ && inject_at_ >= 0.0 && inject_ != 1.0 && timestamp - first_t_ >= inject_at_) {
    injected_ = true;
    PRINT_WARNING(YELLOW "[SCALE]: test injection x%.4f at t=%.2f\n" RESET, inject_, timestamp - first_t_);
    request(inject_, "inject");
  }

  // Match buffered reference positions to the published states by time
  while (!refs_.empty()) {
    double tr = refs_.front().first;
    if (tr > hist_.back().first + match_tol_)
      break; // newer than anything we have: wait for the next state
    if (tr < hist_.front().first - match_tol_) {
      refs_.pop_front(); // older than our history
      continue;
    }
    double best_dt = 1e9;
    Eigen::Vector3d best_p = Eigen::Vector3d::Zero();
    for (auto it = hist_.rbegin(); it != hist_.rend(); ++it) {
      double dt = std::fabs(it->first - tr);
      if (dt < best_dt) {
        best_dt = dt;
        best_p = it->second;
      } else if (it->first < tr) {
        break;
      }
    }
    if (best_dt <= match_tol_)
      est_.add(best_p, refs_.front().second);
    refs_.pop_front();
  }

  // Estimate and (optionally) apply
  double s_rel, span, rms;
  int n;
  if (est_.estimate(s_rel, n, span, rms)) {
    have_est_ = true;
    est_rel_ = s_rel;
    est_n_ = n;
    est_span_ = span;
    est_rms_ = rms;
    est_abs_ = app_->scale_applied() * s_rel;
    std_msgs::Float64 m;
    m.data = est_abs_;
    pub_est_.publish(m);
    if (auto_apply_ && !await_apply_ && std::fabs(s_rel - 1.0) > auto_tol_) {
      PRINT_INFO("[SCALE]: auto: reference says x%.4f (n=%d span=%.2f m rms=%.3f m)\n", s_rel, n, span, rms);
      request(s_rel, "auto");
    }
  }
  publish_status(false);
}

void ScaleCalibrator::publish_status(bool force) {
  double now = ros::Time::now().toSec();
  if (!force && last_status_t_ >= 0.0 && now - last_status_t_ < 0.5)
    return;
  last_status_t_ = now;
  double dist_from_mark = (mark_valid_ && !hist_.empty()) ? (hist_.back().second - mark_p_).norm() : -1.0;
  char buf[768];
  std::snprintf(buf, sizeof(buf),
                "{\"scale_total\":%.6f,\"n_applied\":%d,\"await\":%d,\"last\":\"%s\","
                "\"have_est\":%d,\"est\":%.6f,\"est_rel\":%.6f,\"est_n\":%d,\"est_span\":%.3f,\"est_rms\":%.4f,"
                "\"n_pairs\":%d,\"span\":%.3f,\"min_span\":%.2f,\"min_pairs\":%d,"
                "\"mark\":%d,\"dist_from_mark\":%.3f,\"auto\":%d,\"auto_tol\":%.3f,\"ref_topic\":\"%s\"}",
                app_->scale_applied(), app_->scale_count(), (int)await_apply_, last_why_.c_str(), (int)have_est_, est_abs_, est_rel_, est_n_,
                est_span_, est_rms_, est_.size(), est_.span(), est_.min_span(), est_.min_pairs(), (int)mark_valid_, dist_from_mark,
                (int)auto_apply_, auto_tol_, ref_topic_.c_str());
  std_msgs::String m;
  m.data = buf;
  pub_status_.publish(m);
}
