/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OV_CORE_CAM_BASE_H
#define OV_CORE_CAM_BASE_H

#include <Eigen/Eigen>
#include <cmath>
#include <string>
#include <unordered_map>

#include <opencv2/opencv.hpp>

namespace ov_core {

/**
 * @brief Base pinhole camera model class
 *
 * This is the base class for all our camera models.
 * All these models are pinhole cameras, thus just have standard reprojection logic.
 * See each derived class for detailed examples of each model.
 */
class CamBase {

public:
  /**
   * @brief Default constructor
   * @param width Width of the camera (raw pixels)
   * @param height Height of the camera (raw pixels)
   */
  CamBase(int width, int height) : _width(width), _height(height) {}

  virtual ~CamBase() {}

  /**
   * @brief This will set and update the camera calibration values.
   * This should be called on startup for each camera and after update!
   * @param calib Camera calibration information (f_x & f_y & c_x & c_y & k_1 & k_2 & k_3 & k_4)
   */
  virtual void set_value(const Eigen::MatrixXd &calib) {

    // Assert we are of size eight
    assert(calib.rows() == 8);
    camera_values = calib;

    // Camera matrix
    cv::Matx33d tempK;
    tempK(0, 0) = calib(0);
    tempK(0, 1) = 0;
    tempK(0, 2) = calib(2);
    tempK(1, 0) = 0;
    tempK(1, 1) = calib(1);
    tempK(1, 2) = calib(3);
    tempK(2, 0) = 0;
    tempK(2, 1) = 0;
    tempK(2, 2) = 1;
    camera_k_OPENCV = tempK;

    // Distortion parameters
    cv::Vec4d tempD;
    tempD(0) = calib(4);
    tempD(1) = calib(5);
    tempD(2) = calib(6);
    tempD(3) = calib(7);
    camera_d_OPENCV = tempD;
  }

  /**
   * @brief Given a raw uv point, this will undistort it based on the camera matrices into normalized camera coords.
   * @param uv_dist Raw uv coordinate we wish to undistort
   * @return 2d vector of normalized coordinates
   */
  virtual Eigen::Vector2f undistort_f(const Eigen::Vector2f &uv_dist) = 0;

  /**
   * @brief Given a raw uv point, this will undistort it based on the camera matrices into normalized camera coords.
   * @param uv_dist Raw uv coordinate we wish to undistort
   * @return 2d vector of normalized coordinates
   */
  Eigen::Vector2d undistort_d(const Eigen::Vector2d &uv_dist) {
    Eigen::Vector2f ept1, ept2;
    ept1 = uv_dist.cast<float>();
    ept2 = undistort_f(ept1);
    return ept2.cast<double>();
  }

  /**
   * @brief Given a raw uv point, this will undistort it based on the camera matrices into normalized camera coords.
   * @param uv_dist Raw uv coordinate we wish to undistort
   * @return 2d vector of normalized coordinates
   */
  cv::Point2f undistort_cv(const cv::Point2f &uv_dist) {
    Eigen::Vector2f ept1, ept2;
    ept1 << uv_dist.x, uv_dist.y;
    ept2 = undistort_f(ept1);
    cv::Point2f pt_out;
    pt_out.x = ept2(0);
    pt_out.y = ept2(1);
    return pt_out;
  }

  /**
   * @brief Given a normalized uv coordinate this will distort it to the raw image plane
   * @param uv_norm Normalized coordinates we wish to distort
   * @return 2d vector of raw uv coordinate
   */
  virtual Eigen::Vector2f distort_f(const Eigen::Vector2f &uv_norm) = 0;

  /**
   * @brief Given a normalized uv coordinate this will distort it to the raw image plane
   * @param uv_norm Normalized coordinates we wish to distort
   * @return 2d vector of raw uv coordinate
   */
  Eigen::Vector2d distort_d(const Eigen::Vector2d &uv_norm) {
    Eigen::Vector2f ept1, ept2;
    ept1 = uv_norm.cast<float>();
    ept2 = distort_f(ept1);
    return ept2.cast<double>();
  }

  /**
   * @brief Given a normalized uv coordinate this will distort it to the raw image plane
   * @param uv_norm Normalized coordinates we wish to distort
   * @return 2d vector of raw uv coordinate
   */
  cv::Point2f distort_cv(const cv::Point2f &uv_norm) {
    Eigen::Vector2f ept1, ept2;
    ept1 << uv_norm.x, uv_norm.y;
    ept2 = distort_f(ept1);
    cv::Point2f pt_out;
    pt_out.x = ept2(0);
    pt_out.y = ept2(1);
    return pt_out;
  }

  /**
   * @brief Computes the derivative of raw distorted to normalized coordinate.
   * @param uv_norm Normalized coordinates we wish to distort
   * @param H_dz_dzn Derivative of measurement z in respect to normalized
   * @param H_dz_dzeta Derivative of measurement z in respect to intrinic parameters
   */
  virtual void compute_distort_jacobian(const Eigen::Vector2d &uv_norm, Eigen::MatrixXd &H_dz_dzn, Eigen::MatrixXd &H_dz_dzeta) = 0;

  // =====================================================================================
  // (mod) Omnidirectional support. New virtuals whose DEFAULT implementations are composed
  // from the three legacy functions above, so CamRadtan / CamEqui behave bit-identically to
  // upstream. Models that can see behind the camera (double sphere, equirectangular) override
  // them and the bearing-based code paths (feature bearings, bearing triangulation, 3D
  // projection Jacobians) are selected by is_omni().
  // =====================================================================================

  /// (mod) True if this model can represent bearings with z <= 0 (needs the bearing-based code paths)
  virtual bool is_omni() const { return false; }

  /// (mod) Model name as published in loop_intrinsics ("plumb_bob", "equidistant", "double_sphere", "equirectangular")
  virtual std::string model_name() const { return "plumb_bob"; }

  /// (mod) Number of used entries of the 8-vector (fx fy cx cy + model parameters)
  virtual int num_params() const { return 8; }

  /// (mod) Equivalent focal length [px/rad] (RANSAC thresholds, angular disparity). Default fx
  virtual double focal_equiv() const { return camera_values(0); }

  /// (mod) True if a 3D point in the camera frame can be projected (in front of the camera / inside the model's domain)
  virtual bool valid(const Eigen::Vector3d &p_FinC) const { return p_FinC(2) > 0.0; }

  /// (mod) True if the raw pixel has a defined bearing (double sphere: inside the unprojection domain)
  virtual bool bearing_valid(const Eigen::Vector2d &uv_dist) const {
    (void)uv_dist;
    return true;
  }

  /// (mod) Unit bearing vector (camera frame) of a raw pixel. Default: normalize([undistort(uv), 1])
  virtual Eigen::Vector3d bearing(const Eigen::Vector2d &uv_dist) {
    Eigen::Vector2d n = undistort_d(uv_dist);
    Eigen::Vector3d b(n(0), n(1), 1.0);
    return b.normalized();
  }

  /// (mod) Project a 3D point in the camera frame to raw pixels. Default: distort(p / z)
  virtual Eigen::Vector2d project(const Eigen::Vector3d &p_FinC) {
    Eigen::Vector2d n(p_FinC(0) / p_FinC(2), p_FinC(1) / p_FinC(2));
    return distort_d(n);
  }

  /**
   * @brief (mod) Projection with its Jacobians w.r.t. the 3D point (2x3) and the intrinsics (2x8).
   * Default: chain rule through the normalized plane, i.e. compute_distort_jacobian(p/z) * d(p/z)/dp.
   * @param p_FinC 3D point in the camera frame
   * @param uv Output raw pixel
   * @param dz_dpfc Output 2x3 Jacobian of uv w.r.t. p_FinC
   * @param dz_dzeta Output 2x8 Jacobian of uv w.r.t. the intrinsic vector
   */
  virtual void jacobian_project(const Eigen::Vector3d &p_FinC, Eigen::Vector2d &uv, Eigen::Matrix<double, 2, 3> &dz_dpfc,
                                Eigen::MatrixXd &dz_dzeta) {
    Eigen::Vector2d n(p_FinC(0) / p_FinC(2), p_FinC(1) / p_FinC(2));
    uv = distort_d(n);
    Eigen::MatrixXd dz_dzn;
    compute_distort_jacobian(n, dz_dzn, dz_dzeta);
    Eigen::Matrix<double, 2, 3> dzn_dpfc;
    dzn_dpfc << 1.0 / p_FinC(2), 0.0, -p_FinC(0) / (p_FinC(2) * p_FinC(2)), 0.0, 1.0 / p_FinC(2), -p_FinC(1) / (p_FinC(2) * p_FinC(2));
    dz_dpfc = dz_dzn * dzn_dpfc;
  }

  /// (mod) Measurement residual in raw pixels (equirectangular wraps the horizontal component). Default: plain difference
  virtual Eigen::Vector2d residual(const Eigen::Vector2d &uv_meas, const Eigen::Vector2d &uv_pred) const { return uv_meas - uv_pred; }

  /// (mod) Per-row whitening weights for anisotropic pixel noise (equirectangular: (cos(lat), 1)). Default (1, 1)
  virtual Eigen::Vector2d residual_weights(const Eigen::Vector2d &uv_meas) const {
    (void)uv_meas;
    return Eigen::Vector2d::Ones();
  }

  /// Gets the complete intrinsic vector
  Eigen::MatrixXd get_value() { return camera_values; }

  /// Gets the camera matrix
  cv::Matx33d get_K() { return camera_k_OPENCV; }

  /// Gets the camera distortion
  cv::Vec4d get_D() { return camera_d_OPENCV; }

  /// Gets the width of the camera images
  int w() { return _width; }

  /// Gets the height of the camera images
  int h() { return _height; }

protected:
  // Cannot construct the base camera class, needs a distortion model
  CamBase() = default;

  /// Raw set of camera intrinic values (f_x & f_y & c_x & c_y & k_1 & k_2 & k_3 & k_4)
  Eigen::MatrixXd camera_values;

  /// Camera intrinsics in OpenCV format
  cv::Matx33d camera_k_OPENCV;

  /// Camera distortion in OpenCV format
  cv::Vec4d camera_d_OPENCV;

  /// Width of the camera (raw pixels)
  int _width;

  /// Height of the camera (raw pixels)
  int _height;
};

} // namespace ov_core

#endif /* OV_CORE_CAM_BASE_H */