/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * (ov360 fork) Unit test of the scale correction: StateHelper::rescale on every landmark representation,
 * and the Umeyama scale estimator. No ROS needed:  rosrun ov_msckf test_scale
 */

#include <Eigen/Eigen>
#include <cstdio>
#include <memory>
#include <random>
#include <vector>

#include "state/State.h"
#include "state/StateHelper.h"
#include "types/Landmark.h"
#include "types/LandmarkRepresentation.h"
#include "utils/ScaleUtils.h"

using namespace ov_msckf;
using namespace ov_type;

static int g_fail = 0;
static void check(bool ok, const std::string &name) {
  printf("  %s %s\n", ok ? "PASS" : "FAIL", name.c_str());
  if (!ok)
    g_fail++;
}

/// A state with 3 clones, nlm SLAM landmarks of the given representation and a dense (correlated) covariance
static std::shared_ptr<State> make_state(LandmarkRepresentation::Representation rep, int nlm, std::mt19937 &rng) {
  StateOptions opt;
  opt.num_cameras = 1;
  opt.max_clone_size = 5;
  opt.feat_rep_msckf = rep;
  opt.feat_rep_slam = rep;
  opt.do_calib_camera_pose = true;
  opt.do_calib_camera_timeoffset = true;
  auto state = std::make_shared<State>(opt);
  Eigen::Matrix<double, 16, 1> x;
  x << 0, 0, 0, 1, 1.0, 2.0, 3.0, 0.1, -0.2, 0.3, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06;
  state->_imu->set_value(x);
  state->_imu->set_fej(x);
  for (int k = 0; k < 3; k++) {
    state->_timestamp = 0.1 * (k + 1);
    Eigen::Matrix<double, 16, 1> xx = state->_imu->value();
    xx.block(4, 0, 3, 1) += Eigen::Vector3d(0.3, 0.1, -0.05);
    state->_imu->set_value(xx);
    state->_imu->set_fej(xx);
    StateHelper::augment_clone(state, Eigen::Vector3d(0.01, 0.02, 0.03));
  }
  for (int i = 0; i < nlm; i++) {
    int dim = (rep == LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) ? 1 : 3;
    auto lm = std::make_shared<Landmark>(dim);
    lm->_feat_representation = rep;
    lm->_anchor_cam_id = 0;
    lm->_anchor_clone_timestamp = state->_timestamp;
    Eigen::Vector3d p(0.5 + 0.3 * i, -0.2 + 0.1 * i, 2.0 + 0.5 * i);
    lm->set_from_xyz(p, false);
    lm->set_from_xyz(p, true);
    std::vector<std::shared_ptr<Type>> order{state->_clones_IMU.rbegin()->second};
    Eigen::MatrixXd H_R = Eigen::MatrixXd::Zero(dim, 6);
    Eigen::MatrixXd H_L = Eigen::MatrixXd::Identity(dim, dim);
    Eigen::MatrixXd R = 0.01 * Eigen::MatrixXd::Identity(dim, dim);
    Eigen::VectorXd res = Eigen::VectorXd::Zero(dim);
    StateHelper::initialize_invertible(state, lm, order, H_R, H_L, R, res);
    state->_features_SLAM.insert({(size_t)(100 + i), lm});
  }
  // Correlate everything with a zero-residual update of a random full-rank H (values stay, covariance mixes)
  std::vector<std::shared_ptr<Type>> all;
  all.push_back(state->_imu);
  for (auto &c : state->_clones_IMU)
    all.push_back(c.second);
  for (auto &f : state->_features_SLAM)
    all.push_back(f.second);
  int N = 0;
  for (auto &v : all)
    N += v->size();
  std::normal_distribution<double> nd(0.0, 1.0);
  Eigen::MatrixXd H(N, N);
  for (int r = 0; r < N; r++)
    for (int c = 0; c < N; c++)
      H(r, c) = nd(rng);
  Eigen::VectorXd res = Eigen::VectorXd::Zero(N);
  Eigen::MatrixXd R = 0.5 * Eigen::MatrixXd::Identity(N, N);
  StateHelper::EKFUpdate(state, all, H, res, R);
  return state;
}

static Eigen::VectorXd factors(std::shared_ptr<State> state, double s) {
  Eigen::VectorXd d = Eigen::VectorXd::Ones(state->max_covariance_size());
  d.segment(state->_imu->p()->id(), 3).setConstant(s);
  d.segment(state->_imu->v()->id(), 3).setConstant(s);
  for (auto &c : state->_clones_IMU)
    d.segment(c.second->p()->id(), 3).setConstant(s);
  for (auto &f : state->_features_SLAM) {
    auto lm = f.second;
    switch (lm->_feat_representation) {
    case LandmarkRepresentation::Representation::GLOBAL_3D:
    case LandmarkRepresentation::Representation::ANCHORED_3D:
      d.segment(lm->id(), 3).setConstant(s);
      break;
    case LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE:
      d(lm->id()) = 1.0 / s;
      break;
    default:
      d(lm->id() + 2) = 1.0 / s;
      break;
    }
  }
  return d;
}

static void test_rescale(LandmarkRepresentation::Representation rep, std::mt19937 &rng) {
  std::string name = LandmarkRepresentation::as_string(rep);
  auto state = make_state(rep, 4, rng);
  const double s = 1.3;
  Eigen::MatrixXd imu0 = state->_imu->value(), imu0f = state->_imu->fej();
  std::vector<Eigen::MatrixXd> cl0, cl0f;
  for (auto &c : state->_clones_IMU) {
    cl0.push_back(c.second->value());
    cl0f.push_back(c.second->fej());
  }
  std::vector<Eigen::Vector3d> lm0, lm0f;
  std::vector<Eigen::MatrixXd> lmraw0;
  for (auto &f : state->_features_SLAM) {
    lm0.push_back(f.second->get_xyz(false));
    lm0f.push_back(f.second->get_xyz(true));
    lmraw0.push_back(f.second->value());
  }
  Eigen::MatrixXd P0 = StateHelper::get_full_covariance(state);
  Eigen::VectorXd d = factors(state, s);
  Eigen::MatrixXd P_exp = d.asDiagonal() * P0 * d.asDiagonal();

  StateHelper::rescale(state, s);

  // IMU: q, bg, ba unchanged, p and v scaled (value and FEJ)
  Eigen::MatrixXd imu1 = state->_imu->value(), imu1f = state->_imu->fej();
  bool ok = (imu1.block(0, 0, 4, 1) - imu0.block(0, 0, 4, 1)).norm() < 1e-12 && (imu1.block(10, 0, 6, 1) - imu0.block(10, 0, 6, 1)).norm() < 1e-12 &&
            (imu1.block(4, 0, 6, 1) - s * imu0.block(4, 0, 6, 1)).norm() < 1e-9 && (imu1f.block(4, 0, 6, 1) - s * imu0f.block(4, 0, 6, 1)).norm() < 1e-9 &&
            (imu1f.block(0, 0, 4, 1) - imu0f.block(0, 0, 4, 1)).norm() < 1e-12;
  check(ok, name + ": IMU p,v x" + std::to_string(s) + " (value+FEJ), q/bg/ba unchanged");
  // clones
  ok = true;
  size_t k = 0;
  for (auto &c : state->_clones_IMU) {
    Eigen::MatrixXd v = c.second->value(), vf = c.second->fej();
    ok = ok && (v.block(0, 0, 4, 1) - cl0[k].block(0, 0, 4, 1)).norm() < 1e-12 && (v.block(4, 0, 3, 1) - s * cl0[k].block(4, 0, 3, 1)).norm() < 1e-9 &&
         (vf.block(4, 0, 3, 1) - s * cl0f[k].block(4, 0, 3, 1)).norm() < 1e-9;
    k++;
  }
  check(ok, name + ": clone positions scaled, orientations unchanged");
  // landmarks: xyz scaled for every representation
  ok = true;
  k = 0;
  for (auto &f : state->_features_SLAM) {
    Eigen::Vector3d p = f.second->get_xyz(false), pf = f.second->get_xyz(true);
    ok = ok && (p - s * lm0[k]).norm() < 1e-9 && (pf - s * lm0f[k]).norm() < 1e-9;
    k++;
  }
  check(ok, name + ": landmark xyz x" + std::to_string(s) + " (value+FEJ)");
  // covariance
  Eigen::MatrixXd P1 = StateHelper::get_full_covariance(state);
  double err = (P1 - P_exp).norm() / P_exp.norm();
  check(err < 1e-12, name + ": covariance = D P D (rel err " + std::to_string(err) + ")");
  // round trip
  StateHelper::rescale(state, 1.0 / s);
  Eigen::MatrixXd P2 = StateHelper::get_full_covariance(state);
  ok = (state->_imu->value() - imu0).norm() < 1e-9 && (state->_imu->fej() - imu0f).norm() < 1e-9 && (P2 - P0).norm() / P0.norm() < 1e-12;
  k = 0;
  for (auto &f : state->_features_SLAM) {
    ok = ok && (f.second->value() - lmraw0[k]).norm() < 1e-9;
    k++;
  }
  check(ok, name + ": rescale(1/s) restores values and covariance");
}

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  std::mt19937 rng(7);
  printf("=== StateHelper::rescale ===\n");
  test_rescale(LandmarkRepresentation::Representation::GLOBAL_3D, rng);
  test_rescale(LandmarkRepresentation::Representation::ANCHORED_3D, rng);
  test_rescale(LandmarkRepresentation::Representation::GLOBAL_FULL_INVERSE_DEPTH, rng);
  test_rescale(LandmarkRepresentation::Representation::ANCHORED_FULL_INVERSE_DEPTH, rng);
  test_rescale(LandmarkRepresentation::Representation::ANCHORED_MSCKF_INVERSE_DEPTH, rng);
  test_rescale(LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE, rng);
  {
    auto state = make_state(LandmarkRepresentation::Representation::ANCHORED_MSCKF_INVERSE_DEPTH, 2, rng);
    Eigen::MatrixXd P0 = StateHelper::get_full_covariance(state);
    Eigen::MatrixXd x0 = state->_imu->value();
    StateHelper::rescale(state, 1.0);
    check((StateHelper::get_full_covariance(state) - P0).norm() == 0.0 && (state->_imu->value() - x0).norm() == 0.0, "rescale(1.0) is a no-op");
  }

  printf("=== Umeyama scale / ScaleEstimator ===\n");
  std::normal_distribution<double> nd(0.0, 1.0);
  std::vector<Eigen::Vector3d> A, B;
  Eigen::Matrix3d R = Eigen::AngleAxisd(0.7, Eigen::Vector3d(0.2, 0.3, 0.9).normalized()).toRotationMatrix();
  Eigen::Vector3d t(0.5, -0.2, 0.1);
  for (int i = 0; i < 60; i++) {
    Eigen::Vector3d a(1.5 * nd(rng), 1.0 * nd(rng), 0.2 * nd(rng));
    A.push_back(a);
    B.push_back(1.3 * (R * a) + t);
  }
  double s = 0, rms = 0;
  check(umeyama_scale(A, B, s, rms) && std::fabs(s - 1.3) < 1e-9 && rms < 1e-9, "umeyama_scale recovers 1.3 (rms ~0) under rotation+translation");
  std::vector<Eigen::Vector3d> A2(A.begin(), A.begin() + 2), B2(B.begin(), B.begin() + 2);
  check(!umeyama_scale(A2, B2, s, rms), "fewer than 3 points -> false");
  std::vector<Eigen::Vector3d> Z(5, Eigen::Vector3d::Zero());
  std::vector<Eigen::Vector3d> B5(B.begin(), B.begin() + 5);
  check(!umeyama_scale(Z, B5, s, rms), "degenerate (no spread) -> false");
  ScaleEstimator est(1.0, 5, 600, 0.02);
  for (int i = 0; i < 10; i++)
    est.add(0.1 * A[i], 0.1 * B[i]);
  int n;
  double span;
  check(!est.estimate(s, n, span, rms) && est.span() < 1.0, "no estimate while span < min_span");
  est.reset();
  for (int i = 0; i < 60; i++)
    est.add(A[i], B[i]);
  check(!est.add(A[59], B[59]), "duplicate (stationary) pair rejected");
  check(est.estimate(s, n, span, rms) && std::fabs(s - 1.3) < 1e-9 && n == 60 && span >= 1.0, "estimate 1.3 once spread out");
  est.rescale(2.0);
  check(est.estimate(s, n, span, rms) && std::fabs(s - 0.65) < 1e-9, "after the VIO side is rescaled by 2 the remaining factor is 0.65");

  printf("%s\n", g_fail ? "FAILED" : "ALL PASSED");
  return g_fail ? 1 : 0;
}
