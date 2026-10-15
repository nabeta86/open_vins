/*
 * OpenVINS (mod): equirectangular (spherical panorama) camera model.
 * Added in the Indoor_drone_slam fork (see DERIVED_FROM.md). GPL-3.0 like the rest of OpenVINS.
 */

#ifndef OV_CORE_CAM_EQUIRECT_H
#define OV_CORE_CAM_EQUIRECT_H

#include "CamBase.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ov_core {

/**
 * @brief (mod) Equirectangular camera model (full sphere, e.g. the output of nodes/equirect.py).
 *
 * Intrinsic vector (8): [fx fy cx cy 0 0 0 0] with fx = W/(2*pi) [px/rad], fy = H/pi [px/rad], cx = W/2 - 0.5, cy = H/2 - 0.5
 * (OpenVINS / Kalibr pixel convention: pixel i is centred at coordinate i, while equirect.py centres it at i + 0.5).
 * Coordinates follow equirect.py build_maps (x right, y down, z forward):
 *   lon = atan2(x, z),  lat = atan2(-y, sqrt(x^2 + z^2)),  u = fx*lon + cx,  v = cy - fy*lat
 *   bearing(u, v) = (cos(lat) sin(lon), -sin(lat), cos(lat) cos(lon))
 * Every direction except p = 0 is valid. The horizontal residual wraps at +-W/2 and rows are whitened by cos(lat)
 * because the image is stretched by 1/cos(lat) horizontally (pixel noise grows accordingly).
 * The legacy normalized-plane API is a tangent-plane fallback (NaN behind the camera).
 */
class CamEquirect : public CamBase {

public:
  CamEquirect(int width, int height) : CamBase(width, height) {}

  ~CamEquirect() {}

  bool is_omni() const override { return true; }
  std::string model_name() const override { return "equirectangular"; }
  int num_params() const override { return 4; }

  bool valid(const Eigen::Vector3d &p) const override { return p.norm() > 1e-9; }

  bool bearing_valid(const Eigen::Vector2d &uv) const override {
    double lat = (camera_values(3) - uv(1)) / camera_values(1);
    return std::abs(lat) <= M_PI / 2.0 + 1e-9;
  }

  Eigen::Vector3d bearing(const Eigen::Vector2d &uv) override {
    double lon = (uv(0) - camera_values(2)) / camera_values(0);
    double lat = (camera_values(3) - uv(1)) / camera_values(1);
    return Eigen::Vector3d(std::cos(lat) * std::sin(lon), -std::sin(lat), std::cos(lat) * std::cos(lon));
  }

  Eigen::Vector2d project(const Eigen::Vector3d &p) override {
    double lon = std::atan2(p(0), p(2));
    double lat = std::atan2(-p(1), std::sqrt(p(0) * p(0) + p(2) * p(2)));
    return Eigen::Vector2d(camera_values(0) * lon + camera_values(2), camera_values(3) - camera_values(1) * lat);
  }

  void jacobian_project(const Eigen::Vector3d &p, Eigen::Vector2d &uv, Eigen::Matrix<double, 2, 3> &dz_dp,
                        Eigen::MatrixXd &dz_dzeta) override {
    double fx = camera_values(0), fy = camera_values(1);
    double rho2 = p(0) * p(0) + p(2) * p(2);
    double rho = std::sqrt(rho2);
    double r2 = rho2 + p(1) * p(1);
    double lon = std::atan2(p(0), p(2));
    double lat = std::atan2(-p(1), rho);
    uv << fx * lon + camera_values(2), camera_values(3) - fy * lat;
    Eigen::RowVector3d dlon(p(2) / rho2, 0.0, -p(0) / rho2);
    Eigen::RowVector3d dlat(p(1) * p(0) / (r2 * rho), -rho / r2, p(1) * p(2) / (r2 * rho));
    dz_dp.row(0) = fx * dlon;
    dz_dp.row(1) = -fy * dlat;
    dz_dzeta = Eigen::MatrixXd::Zero(2, 8);
    dz_dzeta(0, 0) = lon;
    dz_dzeta(0, 2) = 1.0;
    dz_dzeta(1, 1) = -lat;
    dz_dzeta(1, 3) = 1.0;
  }

  Eigen::Vector2d residual(const Eigen::Vector2d &uv_meas, const Eigen::Vector2d &uv_pred) const override {
    Eigen::Vector2d d = uv_meas - uv_pred;
    double W = 2.0 * M_PI * camera_values(0);
    d(0) -= std::round(d(0) / W) * W;
    return d;
  }

  Eigen::Vector2d residual_weights(const Eigen::Vector2d &uv_meas) const override {
    double lat = (camera_values(3) - uv_meas(1)) / camera_values(1);
    return Eigen::Vector2d(std::max(std::cos(lat), 0.05), 1.0);
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

#endif /* OV_CORE_CAM_EQUIRECT_H */
