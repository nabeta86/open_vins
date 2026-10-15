/*
 * OpenVINS (mod): standalone checks of the camera models (ov_core/src/cam/*).
 * Added in the Indoor_drone_slam fork (see DERIVED_FROM.md). GPL-3.0 like the rest of OpenVINS.
 *
 *   rosrun ov_core test_cams     (exit code 0 = all checks passed)
 *
 *  1. project(bearing(uv)) == uv for pixels inside each model's domain (radtan, equidistant, double sphere, equirectangular)
 *  2. jacobian_project() vs central differences (2x3 w.r.t. the point, 2x8 w.r.t. the intrinsics)
 *  3. legacy models: the new default virtuals are composed from the old functions, so project(p) == distort_d(p/z)
 *     and bearing(uv) == normalize([undistort_d(uv), 1]) bit for bit (regression guard for the pinhole path)
 *  4. double sphere: points behind the camera (z < 0) project and unproject consistently; validity boundary
 *  5. equirectangular: horizontal wrap of the residual, cos(lat) weights, poles
 */

#include <cmath>
#include <cstdio>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "cam/CamBase.h"
#include "cam/CamDoubleSphere.h"
#include "cam/CamEqui.h"
#include "cam/CamEquirect.h"
#include "cam/CamFactory.h"
#include "cam/CamRadtan.h"

using namespace ov_core;

static int n_fail = 0;
static void check(const std::string &name, bool ok, double detail = 0.0) {
  printf("  %s %s", ok ? "[PASS]" : "[FAIL]", name.c_str());
  if (!ok)
    printf("  (%.3e)", detail);
  printf("\n");
  if (!ok)
    n_fail++;
}

static Eigen::VectorXd calib(double fx, double fy, double cx, double cy, double k1, double k2, double k3, double k4) {
  Eigen::VectorXd c(8);
  c << fx, fy, cx, cy, k1, k2, k3, k4;
  return c;
}

// numeric Jacobians of project() w.r.t. the point and the intrinsics
static void numeric_jacobians(std::shared_ptr<CamBase> cam, const Eigen::Vector3d &p, Eigen::Matrix<double, 2, 3> &Jp, Eigen::MatrixXd &Jz,
                              const Eigen::Vector2d &(*wrap)(const Eigen::Vector2d &) = nullptr) {
  (void)wrap;
  double h = 1e-6;
  for (int i = 0; i < 3; i++) {
    Eigen::Vector3d dp = Eigen::Vector3d::Zero();
    dp(i) = h;
    Eigen::Vector2d a = cam->project(p + dp), b = cam->project(p - dp);
    Eigen::Vector2d d = cam->residual(a, b);
    Jp.col(i) = d / (2 * h);
  }
  Eigen::VectorXd c0 = cam->get_value();
  Jz = Eigen::MatrixXd::Zero(2, 8);
  for (int i = 0; i < cam->num_params(); i++) {
    double hh = (i < 4) ? 1e-4 : 1e-6;
    Eigen::VectorXd c = c0;
    c(i) += hh;
    cam->set_value(c);
    Eigen::Vector2d a = cam->project(p);
    c(i) -= 2 * hh;
    cam->set_value(c);
    Eigen::Vector2d b = cam->project(p);
    cam->set_value(c0);
    Jz.col(i) = cam->residual(a, b) / (2 * hh);
  }
}

static double rel_err(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b) {
  double scale = std::max(1.0, b.cwiseAbs().maxCoeff());
  return (a - b).cwiseAbs().maxCoeff() / scale;
}

int main() {
  std::mt19937 rng(20261015);
  std::uniform_real_distribution<double> U(-1.0, 1.0);

  struct Model {
    std::string name;
    std::shared_ptr<CamBase> cam;
    double max_theta_deg; // sample bearings with angle to the optical axis up to this
  };
  std::vector<Model> models;
  models.push_back({"radtan", make_camera("radtan", 960, 600, calib(568.0, 567.0, 482.0, 294.0, -0.05, 0.02, 0.0005, -0.0003)), 40.0});
  models.push_back({"equidistant", make_camera("equidistant", 736, 736, calib(210.0, 211.0, 368.0, 367.0, 0.01, -0.002, 0.0003, 0.0)), 75.0});
  models.push_back({"double_sphere", make_camera("ds", 736, 736, calib(300.0, 301.0, 368.0, 367.0, -0.2, 0.6, 0.0, 0.0)), 100.0});
  models.push_back({"equirectangular", make_camera("equirect", 1472, 736, calib(1472.0 / (2 * M_PI), 736.0 / M_PI, 735.5, 367.5, 0, 0, 0, 0)), 170.0});

  for (auto &m : models) {
    printf("== %s (is_omni=%d, num_params=%d, model_name=%s)\n", m.name.c_str(), (int)m.cam->is_omni(), m.cam->num_params(),
           m.cam->model_name().c_str());
    double worst_rt = 0, worst_jp = 0, worst_jz = 0, worst_legacy_p = 0, worst_legacy_b = 0;
    int n_valid = 0;
    for (int t = 0; t < 300; t++) {
      // random bearing within max_theta of the optical axis
      double theta = std::acos(1.0 - (1.0 - std::cos(m.max_theta_deg * M_PI / 180.0)) * (0.5 * U(rng) + 0.5));
      double phi = M_PI * U(rng);
      Eigen::Vector3d b(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta));
      double depth = 1.0 + 4.0 * (0.5 * U(rng) + 0.5);
      Eigen::Vector3d p = depth * b;
      if (!m.cam->valid(p))
        continue;
      Eigen::Vector2d uv = m.cam->project(p);
      if (uv(0) < 0 || uv(1) < 0 || uv(0) > m.cam->w() - 1 || uv(1) > m.cam->h() - 1 || !m.cam->bearing_valid(uv))
        continue;
      n_valid++;
      // 1. round trip
      Eigen::Vector3d b2 = m.cam->bearing(uv);
      worst_rt = std::max(worst_rt, (b2 - b).norm());
      // 2. jacobians
      Eigen::Vector2d uv2;
      Eigen::Matrix<double, 2, 3> Jp;
      Eigen::MatrixXd Jz;
      m.cam->jacobian_project(p, uv2, Jp, Jz);
      Eigen::Matrix<double, 2, 3> Jp_num;
      Eigen::MatrixXd Jz_num;
      numeric_jacobians(m.cam, p, Jp_num, Jz_num);
      worst_jp = std::max(worst_jp, rel_err(Jp, Jp_num));
      worst_jz = std::max(worst_jz, rel_err(Jz, Jz_num));
      worst_rt = std::max(worst_rt, (uv2 - uv).norm());
      // 3. legacy equivalence for the pinhole models
      if (!m.cam->is_omni()) {
        Eigen::Vector2d n(p(0) / p(2), p(1) / p(2));
        Eigen::Vector2d uv_legacy = m.cam->distort_d(n);
        worst_legacy_p = std::max(worst_legacy_p, (uv_legacy - uv).norm());
        Eigen::Vector2d nn = m.cam->undistort_d(uv);
        Eigen::Vector3d bl(nn(0), nn(1), 1.0);
        bl.normalize();
        worst_legacy_b = std::max(worst_legacy_b, (bl - b2).norm());
      }
    }
    check(m.name + ": sampled points inside the domain (>= 150)", n_valid >= 150, n_valid);
    check(m.name + ": bearing(project(p)) == p/|p| and jacobian_project uv == project (< 1e-4)", worst_rt < 1e-4, worst_rt);
    check(m.name + ": d uv / d p vs central differences (rel < 1e-5)", worst_jp < 1e-5, worst_jp);
    check(m.name + ": d uv / d intrinsics vs central differences (rel < 1e-5)", worst_jz < 1e-5, worst_jz);
    if (!m.cam->is_omni()) {
      check(m.name + ": project(p) is exactly distort_d(p/z) (legacy path unchanged)", worst_legacy_p == 0.0, worst_legacy_p);
      check(m.name + ": bearing(uv) is exactly normalize([undistort_d(uv), 1])", worst_legacy_b == 0.0, worst_legacy_b);
    }
  }

  printf("== double sphere: behind the camera and the validity boundary\n");
  {
    auto ds = models[2].cam;
    Eigen::Vector3d p(2.0, 0.5, -0.4); // z < 0, well inside the 200 deg lens
    check("ds: point with z < 0 is valid", ds->valid(p));
    Eigen::Vector2d uv = ds->project(p);
    Eigen::Vector3d b = ds->bearing(uv);
    check("ds: z < 0 point projects inside the image and unprojects to the same bearing", (b - p.normalized()).norm() < 1e-6 && uv(0) > 0 &&
                                                                                           uv(0) < 736 && uv(1) > 0 && uv(1) < 736);
    Eigen::Vector3d back(0.0, 0.0, -1.0);
    check("ds: straight behind is not valid", !ds->valid(back));
    Eigen::Vector2f n = ds->undistort_f(uv.cast<float>());
    check("ds: legacy undistort_f returns NaN behind the camera", std::isnan(n(0)) && std::isnan(n(1)));
    Eigen::Vector2d far(368.0 + 5000.0, 367.0);
    check("ds: pixel outside the unprojection domain is flagged by bearing_valid", !ds->bearing_valid(far));
  }

  printf("== equirectangular: wrap, weights, poles\n");
  {
    auto eq = models[3].cam;
    double W = 1472.0;
    Eigen::Vector2d a(1470.0, 300.0), bb(2.0, 300.0);
    Eigen::Vector2d r = eq->residual(a, bb);
    check("equirect: residual wraps across the image edge (1470 - 2 -> -4 px)", std::abs(r(0) + 4.0) < 1e-9 && r(1) == 0.0, r(0));
    Eigen::Vector2d w = eq->residual_weights(Eigen::Vector2d(100.0, 367.5));
    check("equirect: weight 1 on the equator", std::abs(w(0) - 1.0) < 1e-9 && w(1) == 1.0);
    Eigen::Vector2d wp = eq->residual_weights(Eigen::Vector2d(100.0, 0.0));
    check("equirect: weight ~ cos(lat) near the pole (clamped >= 0.05)", wp(0) < 0.2 && wp(0) >= 0.05, wp(0));
    Eigen::Vector3d fwd(0.0, 0.0, 1.0), left(-1.0, 0.0, 0.0), up(0.0, -1.0, 0.0);
    Eigen::Vector2d uf = eq->project(fwd), ul = eq->project(left), uu = eq->project(up);
    check("equirect: forward -> image centre (cx, cy)", std::abs(uf(0) - 735.5) < 1e-9 && std::abs(uf(1) - 367.5) < 1e-9);
    check("equirect: -x (left) -> lon -90 deg (u = cx - W/4)", std::abs(ul(0) - (735.5 - W / 4)) < 1e-9, ul(0));
    check("equirect: up (-y) -> top row (v = cy - H/2)", std::abs(uu(1) - (367.5 - 368.0)) < 1e-9, uu(1));
    check("equirect: focal_equiv is W/2pi", std::abs(eq->focal_equiv() - W / (2 * M_PI)) < 1e-9);
  }

  printf("== factory\n");
  {
    check("factory: 'double_sphere' alias", make_camera("double_sphere", 10, 10, calib(1, 1, 5, 5, 0, 0.5, 0, 0))->is_omni());
    check("factory: 'equirectangular' alias", make_camera("equirectangular", 10, 10, calib(1, 1, 5, 5, 0, 0, 0, 0))->model_name() == "equirectangular");
    check("factory: radtan/equidistant are not omni", !make_camera("radtan", 10, 10, calib(1, 1, 5, 5, 0, 0, 0, 0))->is_omni() &&
                                                           !make_camera("equidistant", 10, 10, calib(1, 1, 5, 5, 0, 0, 0, 0))->is_omni());
  }

  printf("%s (%d failures)\n", n_fail == 0 ? "ALL PASSED" : "FAILED", n_fail);
  return n_fail == 0 ? 0 : 1;
}
