/*
 * OpenVINS (mod): single place that maps the yaml "distortion_model" string to a camera model.
 * Added in the Indoor_drone_slam fork (see DERIVED_FROM.md). GPL-3.0 like the rest of OpenVINS.
 */

#ifndef OV_CORE_CAM_FACTORY_H
#define OV_CORE_CAM_FACTORY_H

#include <cstdlib>
#include <memory>
#include <string>

#include "CamBase.h"
#include "CamDoubleSphere.h"
#include "CamEqui.h"
#include "CamEquirect.h"
#include "CamRadtan.h"
#include "utils/colors.h"
#include "utils/print.h"

namespace ov_core {

/**
 * @brief (mod) Create the camera model named by the yaml key "distortion_model".
 *
 *   radtan                  -> CamRadtan        (upstream)
 *   equidistant             -> CamEqui          (upstream, Kalibr pinhole-equi, theta < 90 deg only)
 *   ds | double_sphere      -> CamDoubleSphere  (Kalibr ds-none, intrinsics [fx fy cx cy] + distortion_coeffs [xi alpha 0 0])
 *   equirect | equirectangular -> CamEquirect   (intrinsics [W/2pi, H/pi, W/2-0.5, H/2-0.5], distortion_coeffs [0 0 0 0])
 *
 * Unknown names abort (upstream silently fell back to radtan). Both the ov_msckf and the ov_init parser use this.
 */
inline std::shared_ptr<CamBase> make_camera(const std::string &dist_model, int width, int height, const Eigen::VectorXd &calib8) {
  std::shared_ptr<CamBase> cam;
  if (dist_model == "radtan" || dist_model == "plumb_bob" || dist_model == "pinhole") {
    cam = std::make_shared<CamRadtan>(width, height);
  } else if (dist_model == "equidistant") {
    cam = std::make_shared<CamEqui>(width, height);
  } else if (dist_model == "ds" || dist_model == "double_sphere") {
    cam = std::make_shared<CamDoubleSphere>(width, height);
  } else if (dist_model == "equirect" || dist_model == "equirectangular") {
    cam = std::make_shared<CamEquirect>(width, height);
  } else {
    PRINT_ERROR(RED "make_camera(): unknown distortion_model '%s' (radtan | equidistant | ds | equirect)\n" RESET, dist_model.c_str());
    std::exit(EXIT_FAILURE);
  }
  cam->set_value(calib8);
  return cam;
}

} // namespace ov_core

#endif /* OV_CORE_CAM_FACTORY_H */
