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

#ifndef OPEN_VINS_FEATUREINITIALIZER_H
#define OPEN_VINS_FEATUREINITIALIZER_H

#include <unordered_map>

#include "FeatureInitializerOptions.h"

namespace ov_core {

class Feature;

/**
 * @brief Class that triangulates feature
 *
 * This class has the functions needed to triangulate and then refine a given 3D feature.
 * As in the standard MSCKF, we know the clones of the camera from propagation and past updates.
 * Thus, we just need to triangulate a feature in 3D with the known poses and then refine it.
 * One should first call the single_triangulation() function afterwhich single_gaussnewton() allows for refinement.
 * Please see the @ref update-featinit page for detailed derivations.
 */
class FeatureInitializer {

public:
  /**
   * @brief Structure which stores pose estimates for use in triangulation
   *
   * - R_GtoC - rotation from global to camera
   * - p_CinG - position of camera in global frame
   */
  struct ClonePose {

    /// Rotation
    Eigen::Matrix<double, 3, 3> _Rot;

    /// Position
    Eigen::Matrix<double, 3, 1> _pos;

    /// Constructs pose from rotation and position
    ClonePose(const Eigen::Matrix<double, 3, 3> &R, const Eigen::Matrix<double, 3, 1> &p) {
      _Rot = R;
      _pos = p;
    }

    /// Constructs pose from quaternion and position
    ClonePose(const Eigen::Matrix<double, 4, 1> &q, const Eigen::Matrix<double, 3, 1> &p) {
      _Rot = quat_2_Rot(q);
      _pos = p;
    }

    /// Default constructor
    ClonePose() {
      _Rot = Eigen::Matrix<double, 3, 3>::Identity();
      _pos = Eigen::Matrix<double, 3, 1>::Zero();
    }

    /// Accessor for rotation
    const Eigen::Matrix<double, 3, 3> &Rot() { return _Rot; }

    /// Accessor for position
    const Eigen::Matrix<double, 3, 1> &pos() { return _pos; }
  };

  /**
   * @brief Default constructor
   * @param options Options for the initializer
   */
  FeatureInitializer(FeatureInitializerOptions &options) : _options(options) {}

  /**
   * @brief Uses a linear triangulation to get initial estimate for the feature
   *
   * The derivations for this method can be found in the @ref featinit-linear documentation page.
   *
   * @param feat Pointer to feature
   * @param clonesCAM Map between camera ID to map of timestamp to camera pose estimate (rotation from global to camera, position of camera
   * in global frame)
   * @return Returns false if it fails to triangulate (based on the thresholds)
   */
  bool single_triangulation(std::shared_ptr<Feature> feat, std::unordered_map<size_t, std::unordered_map<double, ClonePose>> &clonesCAM);

  /**
   * @brief Uses a linear triangulation to get initial estimate for the feature, treating the anchor observation as a true bearing.
   *
   * The derivations for this method can be found in the @ref featinit-linear-1d documentation page.
   * This function should be used if you want speed, or know your anchor bearing is reasonably accurate.
   *
   * @param feat Pointer to feature
   * @param clonesCAM Map between camera ID to map of timestamp to camera pose estimate (rotation from global to camera, position of camera
   * in global frame)
   * @return Returns false if it fails to triangulate (based on the thresholds)
   */
  bool single_triangulation_1d(std::shared_ptr<Feature> feat, std::unordered_map<size_t, std::unordered_map<double, ClonePose>> &clonesCAM);

  /**
   * @brief (mod) Unit bearing (camera frame) of measurement m of camera cam: the stored bearing if present, else [uv_norm, 1] normalised
   */
  Eigen::Vector3d obs_bearing(const std::shared_ptr<Feature> &feat, size_t cam, size_t m) const;

  /**
   * @brief (mod) Orthonormal tangent basis (t1, t2) of a unit vector b
   */
  static void tangent_basis(const Eigen::Vector3d &b, Eigen::Vector3d &t1, Eigen::Vector3d &t2);

  /**
   * @brief Uses a nonlinear triangulation to refine initial linear estimate of the feature
   * @param feat Pointer to feature
   * @param clonesCAM Map between camera ID to map of timestamp to camera pose estimate (rotation from global to camera, position of camera
   * in global frame)
   * @return Returns false if it fails to be optimize (based on the thresholds)
   */
  bool single_gaussnewton(std::shared_ptr<Feature> feat, std::unordered_map<size_t, std::unordered_map<double, ClonePose>> &clonesCAM);

  /**
   * @brief Gets the current configuration of the feature initializer
   * @return Const feature initializer config
   */
  const FeatureInitializerOptions config() { return _options; }

protected:
  /// Contains options for the initializer process
  FeatureInitializerOptions _options;

  /**
   * @brief (mod) Gauss-Newton refinement on the sphere for omnidirectional cameras.
   * Parameters (a, b, rho) with p_FinA = (b0 + a t1 + b t2) / rho, b0 = anchor bearing, (t1, t2) its tangent basis;
   * residual = components of the predicted bearing in the tangent plane of the measured bearing [rad].
   */
  bool single_gaussnewton_bearing(std::shared_ptr<Feature> feat, std::unordered_map<size_t, std::unordered_map<double, ClonePose>> &clonesCAM);

  /// (mod) Sum of squared tangent-plane residuals [rad^2] for the bearing parameterisation
  double compute_error_bearing(std::unordered_map<size_t, std::unordered_map<double, ClonePose>> &clonesCAM, std::shared_ptr<Feature> feat,
                               const Eigen::Vector3d &b0, const Eigen::Vector3d &t1, const Eigen::Vector3d &t2, double a, double b, double rho);

  /**
   * @brief Helper function for the gauss newton method that computes error of the given estimate
   * @param clonesCAM Map between camera ID to map of timestamp to camera pose estimate
   * @param feat Pointer to the feature
   * @param alpha x/z in anchor
   * @param beta y/z in anchor
   * @param rho 1/z inverse depth
   */
  double compute_error(std::unordered_map<size_t, std::unordered_map<double, ClonePose>> &clonesCAM, std::shared_ptr<Feature> feat,
                       double alpha, double beta, double rho);
};

} // namespace ov_core

#endif // OPEN_VINS_FEATUREINITIALIZER_H