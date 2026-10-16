/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * (ov360 fork) Scale-correction helpers. See DERIVED_FROM.md.
 */

#ifndef OV_MSCKF_SCALE_UTILS_H
#define OV_MSCKF_SCALE_UTILS_H

#include <Eigen/Eigen>
#include <cmath>
#include <deque>
#include <vector>

namespace ov_msckf {

/**
 * @brief (ov360) Scale of the similarity transform dst ≈ s R src + t (Umeyama 1991, closed form).
 *
 * src are VIO positions (unknown scale) and dst the metric reference positions, so s is the factor
 * that has to be applied to the VIO. Rotation and translation are solved jointly but only s is returned.
 * @param rms residual of dst − (s R src + t) in metres
 * @return false if fewer than 3 points, no spread, or a non-positive / non-finite scale
 */
inline bool umeyama_scale(const std::vector<Eigen::Vector3d> &src, const std::vector<Eigen::Vector3d> &dst, double &s, double &rms) {
  size_t n = std::min(src.size(), dst.size());
  if (n < 3)
    return false;
  Eigen::Vector3d ma = Eigen::Vector3d::Zero(), mb = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < n; i++) {
    ma += src[i];
    mb += dst[i];
  }
  ma /= (double)n;
  mb /= (double)n;
  double var_a = 0.0;
  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < n; i++) {
    Eigen::Vector3d a = src[i] - ma, b = dst[i] - mb;
    var_a += a.squaredNorm();
    H += b * a.transpose();
  }
  var_a /= (double)n;
  H /= (double)n;
  if (var_a < 1e-12)
    return false;
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
  if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0.0)
    S(2, 2) = -1.0;
  Eigen::Matrix3d D = svd.singularValues().asDiagonal();
  s = (D * S).trace() / var_a;
  if (!std::isfinite(s) || s <= 0.0)
    return false;
  Eigen::Matrix3d R = svd.matrixU() * S * svd.matrixV().transpose();
  double se = 0.0;
  for (size_t i = 0; i < n; i++) {
    Eigen::Vector3d r = (dst[i] - mb) - s * (R * (src[i] - ma));
    se += r.squaredNorm();
  }
  rms = std::sqrt(se / (double)n);
  return true;
}

/**
 * @brief (ov360) Accumulates (VIO, reference) position pairs and estimates the remaining scale factor.
 *
 * Pairs only constrain the scale once they are spread out (the span is the largest distance from the
 * centroid), so nothing is returned below min_span. The VIO side holds positions that already include
 * every correction applied so far, so the estimate is the correction *relative to the current scale*
 * (absolute factor w.r.t. the raw estimator = current total × estimate).
 */
class ScaleEstimator {
public:
  ScaleEstimator(double min_span = 1.0, int min_pairs = 30, int max_pairs = 600, double min_step = 0.02)
      : min_span_(min_span), min_pairs_(min_pairs), max_pairs_(max_pairs), min_step_(min_step) {}

  /// Add one pair. Rejected (false) if non-finite or the VIO moved less than min_step since the last pair (stationary duplicates)
  bool add(const Eigen::Vector3d &p_vio, const Eigen::Vector3d &p_ref) {
    if (!p_vio.allFinite() || !p_ref.allFinite())
      return false;
    if (!src_.empty() && (p_vio - src_.back()).norm() < min_step_)
      return false;
    src_.push_back(p_vio);
    dst_.push_back(p_ref);
    while ((int)src_.size() > max_pairs_) {
      src_.pop_front();
      dst_.pop_front();
    }
    return true;
  }

  /// The VIO side was rescaled by ratio (a correction was applied): keep the pairs consistent
  void rescale(double ratio) {
    for (auto &p : src_)
      p *= ratio;
  }

  void reset() {
    src_.clear();
    dst_.clear();
  }

  int size() const { return (int)src_.size(); }

  /// Largest distance of the VIO positions from their centroid [m]
  double span() const {
    if (src_.size() < 2)
      return 0.0;
    Eigen::Vector3d m = Eigen::Vector3d::Zero();
    for (auto const &p : src_)
      m += p;
    m /= (double)src_.size();
    double best = 0.0;
    for (auto const &p : src_)
      best = std::max(best, (p - m).norm());
    return best;
  }

  /// Relative scale estimate; false while there are too few pairs or too little spread
  bool estimate(double &s_rel, int &n, double &span_out, double &rms) const {
    n = (int)src_.size();
    span_out = span();
    if (n < min_pairs_ || span_out < min_span_)
      return false;
    std::vector<Eigen::Vector3d> a(src_.begin(), src_.end()), b(dst_.begin(), dst_.end());
    return umeyama_scale(a, b, s_rel, rms);
  }

  double min_span() const { return min_span_; }
  int min_pairs() const { return min_pairs_; }

private:
  double min_span_;
  int min_pairs_, max_pairs_;
  double min_step_;
  std::deque<Eigen::Vector3d> src_, dst_;
};

} // namespace ov_msckf

#endif // OV_MSCKF_SCALE_UTILS_H
