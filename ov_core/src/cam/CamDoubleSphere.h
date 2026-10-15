/*
 * OpenVINS (mod): double sphere camera model for >180 degree fisheye lenses.
 * Added in the Indoor_drone_slam fork (see DERIVED_FROM.md). GPL-3.0 like the rest of OpenVINS.
 */

#ifndef OV_CORE_CAM_DOUBLE_SPHERE_H
#define OV_CORE_CAM_DOUBLE_SPHERE_H

#include "CamBase.h"

#include <cmath>
#include <limits>

namespace ov_core {

/**
 * @brief (mod) Double sphere camera model (Usenko, Demmel, Cremers, "The Double Sphere Camera Model", 3DV 2018).
 *
 * Intrinsic vector (8): [fx fy cx cy xi alpha 0 0]. Kalibr "ds-none" gives [xi alpha fu fv pu pv]; kalibr2openvins.py reorders
 * it into intrinsics [fx fy cx cy] + distortion_coeffs [xi alpha 0 0] so the upstream 4+4 yaml parsing stays unchanged.
 *
 * Projection of p = (x, y, z):
 *   d1 = |p|,  s = xi*d1 + z,  d2 = sqrt(x^2 + y^2 + s^2),  den = alpha*d2 + (1-alpha)*s
 *   u = fx*x/den + cx,  v = fy*y/den + cy
 * valid iff z > -w2*d1,  w1 = alpha <= 0.5 ? alpha/(1-alpha) : (1-alpha)/alpha,  w2 = (w1 + xi)/sqrt(2*w1*xi + xi^2 + 1)
 * Unprojection of (u, v): mx = (u-cx)/fx, my = (v-cy)/fy, r2 = mx^2 + my^2 (defined iff alpha <= 0.5 or r2 <= 1/(2alpha-1))
 *   mz = (1 - alpha^2*r2) / (alpha*sqrt(1 - (2alpha-1)*r2) + 1 - alpha)
 *   b  = (mz*xi + sqrt(mz^2 + (1-xi^2)*r2)) / (mz^2 + r2) * (mx, my, mz) - (0, 0, xi)
 *
 * The legacy normalized-plane API (undistort_f / distort_f / compute_distort_jacobian) is a tangent-plane fallback so
 * existing call sites compile; the omni code paths use bearing() / project() / jacobian_project() instead.
 */
class CamDoubleSphere : public CamBase {

public:
  CamDoubleSphere(int width, int height) : CamBase(width, height) {}

  ~CamDoubleSphere() {}

  bool is_omni() const override { return true; }
  std::string model_name() const override { return "double_sphere"; }
  int num_params() const override { return 6; }

  bool valid(const Eigen::Vector3d &p) const override {
    double xi = camera_values(4), alpha = camera_values(5);
    double w1 = (alpha <= 0.5) ? alpha / (1.0 - alpha) : (1.0 - alpha) / alpha;
    double w2 = (w1 + xi) / std::sqrt(2.0 * w1 * xi + xi * xi + 1.0);
    return p(2) > -w2 * p.norm();
  }

  bool bearing_valid(const Eigen::Vector2d &uv) const override {
    double alpha = camera_values(5);
    double mx = (uv(0) - camera_values(2)) / camera_values(0);
    double my = (uv(1) - camera_values(3)) / camera_values(1);
    double r2 = mx * mx + my * my;
    return (alpha <= 0.5) || (r2 <= 1.0 / (2.0 * alpha - 1.0));
  }

  Eigen::Vector3d bearing(const Eigen::Vector2d &uv) override {
    double xi = camera_values(4), alpha = camera_values(5);
    double mx = (uv(0) - camera_values(2)) / camera_values(0);
    double my = (uv(1) - camera_values(3)) / camera_values(1);
    double r2 = mx * mx + my * my;
    double root = 1.0 - (2.0 * alpha - 1.0) * r2;
    if (root < 0.0)
      root = 0.0; // outside the unprojection domain (callers check bearing_valid first)
    double mz = (1.0 - alpha * alpha * r2) / (alpha * std::sqrt(root) + 1.0 - alpha);
    double k = (mz * xi + std::sqrt(mz * mz + (1.0 - xi * xi) * r2)) / (mz * mz + r2);
    Eigen::Vector3d b(k * mx, k * my, k * mz - xi);
    return b.normalized();
  }

  Eigen::Vector2d project(const Eigen::Vector3d &p) override {
    double xi = camera_values(4), alpha = camera_values(5);
    double d1 = p.norm();
    double s = xi * d1 + p(2);
    double d2 = std::sqrt(p(0) * p(0) + p(1) * p(1) + s * s);
    double den = alpha * d2 + (1.0 - alpha) * s;
    return Eigen::Vector2d(camera_values(0) * p(0) / den + camera_values(2), camera_values(1) * p(1) / den + camera_values(3));
  }

  void jacobian_project(const Eigen::Vector3d &p, Eigen::Vector2d &uv, Eigen::Matrix<double, 2, 3> &dz_dp,
                        Eigen::MatrixXd &dz_dzeta) override {
    double fx = camera_values(0), fy = camera_values(1), xi = camera_values(4), alpha = camera_values(5);
    double d1 = p.norm();
    double s = xi * d1 + p(2);
    double d2 = std::sqrt(p(0) * p(0) + p(1) * p(1) + s * s);
    double den = alpha * d2 + (1.0 - alpha) * s;
    double den2 = den * den;
    uv << fx * p(0) / den + camera_values(2), fy * p(1) / den + camera_values(3);
    Eigen::Vector3d ds_dp = xi * p / d1 + Eigen::Vector3d(0.0, 0.0, 1.0);
    Eigen::Vector3d dd2_dp = (Eigen::Vector3d(p(0), p(1), 0.0) + s * ds_dp) / d2;
    Eigen::Vector3d dden_dp = alpha * dd2_dp + (1.0 - alpha) * ds_dp;
    dz_dp.row(0) = fx * (Eigen::RowVector3d(1.0, 0.0, 0.0) * den - p(0) * dden_dp.transpose()) / den2;
    dz_dp.row(1) = fy * (Eigen::RowVector3d(0.0, 1.0, 0.0) * den - p(1) * dden_dp.transpose()) / den2;
    double dden_dxi = alpha * s * d1 / d2 + (1.0 - alpha) * d1;
    double dden_dalpha = d2 - s;
    dz_dzeta = Eigen::MatrixXd::Zero(2, 8);
    dz_dzeta(0, 0) = p(0) / den;
    dz_dzeta(0, 2) = 1.0;
    dz_dzeta(0, 4) = -fx * p(0) / den2 * dden_dxi;
    dz_dzeta(0, 5) = -fx * p(0) / den2 * dden_dalpha;
    dz_dzeta(1, 1) = p(1) / den;
    dz_dzeta(1, 3) = 1.0;
    dz_dzeta(1, 4) = -fy * p(1) / den2 * dden_dxi;
    dz_dzeta(1, 5) = -fy * p(1) / den2 * dden_dalpha;
  }

  // ---- legacy normalized-plane API (tangent plane z = 1; NaN behind the camera) ----
  Eigen::Vector2f undistort_f(const Eigen::Vector2f &uv_dist) override {
    Eigen::Vector3d b = bearing(uv_dist.cast<double>());
    if (b(2) <= 1e-6)
      return Eigen::Vector2f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
    return Eigen::Vector2f((float)(b(0) / b(2)), (float)(b(1) / b(2)));
  }

  Eigen::Vector2f distort_f(const Eigen::Vector2f &uv_norm) override {
    return project(Eigen::Vector3d(uv_norm(0), uv_norm(1), 1.0)).cast<float>();
  }

  void compute_distort_jacobian(const Eigen::Vector2d &uv_norm, Eigen::MatrixXd &H_dz_dzn, Eigen::MatrixXd &H_dz_dzeta) override {
    Eigen::Vector2d uv;
    Eigen::Matrix<double, 2, 3> dz_dp;
    jacobian_project(Eigen::Vector3d(uv_norm(0), uv_norm(1), 1.0), uv, dz_dp, H_dz_dzeta);
    H_dz_dzn = dz_dp.block(0, 0, 2, 2);
  }
};

} // namespace ov_core

#endif /* OV_CORE_CAM_DOUBLE_SPHERE_H */
