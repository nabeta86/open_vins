/*
 * OpenVINS (mod): SLAM landmark with its 3x3 position covariance in the global frame.
 * Added in the Indoor_drone_slam fork (see DERIVED_FROM.md). GPL-3.0 like the rest of OpenVINS.
 */

#ifndef OV_MSCKF_LANDMARKWITHCOV_H
#define OV_MSCKF_LANDMARKWITHCOV_H

#include <Eigen/Eigen>

namespace ov_msckf {

/**
 * @brief A SLAM landmark (persistent feature in the state) with its position covariance.
 *
 * The covariance is that of p_FinG (global frame). For anchored representations it is obtained by
 * chaining the marginal covariance of {landmark parameters, anchor clone pose (, anchor camera extrinsic)}
 * through the same Jacobians the update uses (UpdaterHelper::get_feature_jacobian_representation), so the
 * uncertainty of the anchor pose is included. Linearised at the current estimate (not FEJ).
 */
struct LandmarkWithCov {
  /// Frontend feature id (same id as in /loop_feats channel "id")
  size_t id = 0;
  /// ov_type::LandmarkRepresentation::Representation as int
  int representation = -1;
  /// Anchor camera id (-1 for global representations)
  int anchor_cam_id = -1;
  /// Anchor clone timestamp (-1 for global representations)
  double anchor_clone_time = -1;
  /// Position in the global frame (identical to what VioManager::get_features_SLAM returns)
  Eigen::Vector3d p_FinG = Eigen::Vector3d::Zero();
  /// 3x3 covariance of p_FinG
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
};

} // namespace ov_msckf

#endif // OV_MSCKF_LANDMARKWITHCOV_H
