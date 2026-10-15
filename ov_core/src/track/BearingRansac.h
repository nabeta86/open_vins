/*
 * OpenVINS (mod): RANSAC outlier rejection on unit bearing vectors (any field of view).
 * Added in the Indoor_drone_slam fork (see DERIVED_FROM.md). GPL-3.0 like the rest of OpenVINS.
 */

#ifndef OV_CORE_BEARING_RANSAC_H
#define OV_CORE_BEARING_RANSAC_H

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

namespace ov_core {

/**
 * @brief (mod) 8-point essential matrix RANSAC on unit bearings.
 *
 * cv::findFundamentalMat works on normalized image coordinates, which do not exist behind the camera
 * (omnidirectional lenses). Here the epipolar constraint q^T E p = 0 is solved on unit bearings and the
 * error is the angular distance of a bearing from its epipolar plane (symmetric in the two views), so
 * thresholds are in radians (pixel threshold / equivalent focal length). Deterministic (fixed seed).
 */
class BearingRansac {

public:
  /**
   * @param b0 Unit bearings in view 0
   * @param b1 Unit bearings in view 1 (same order)
   * @param thresh_rad Inlier threshold [rad]
   * @param prob Desired confidence (e.g. 0.999)
   * @param max_iters Hard cap on iterations
   * @param mask Output inlier mask (1 = inlier). All zero if fewer than 8 correspondences
   */
  static void findEssential(const std::vector<Eigen::Vector3d> &b0, const std::vector<Eigen::Vector3d> &b1, double thresh_rad, double prob,
                            int max_iters, std::vector<unsigned char> &mask) {
    size_t n = b0.size();
    mask.assign(n, 0);
    if (n < 8 || b1.size() != n)
      return;
    std::mt19937 rng(0);
    std::uniform_int_distribution<size_t> U(0, n - 1);
    int best = -1;
    std::vector<unsigned char> best_mask(n, 0);
    int iters = max_iters;
    for (int it = 0; it < iters; it++) {
      std::vector<size_t> idx;
      while (idx.size() < 8) {
        size_t k = U(rng);
        if (std::find(idx.begin(), idx.end(), k) == idx.end())
          idx.push_back(k);
      }
      Eigen::Matrix3d E;
      if (!solve8(b0, b1, idx, E))
        continue;
      int cnt = 0;
      std::vector<unsigned char> m(n, 0);
      for (size_t i = 0; i < n; i++) {
        if (error(E, b0[i], b1[i]) < thresh_rad) {
          m[i] = 1;
          cnt++;
        }
      }
      if (cnt > best) {
        best = cnt;
        best_mask = m;
        // adaptive stopping: N = log(1-p) / log(1 - w^8), computed in double (an int cast overflows for small w)
        double w = (double)cnt / (double)n;
        double denom = 1.0 - std::pow(w, 8);
        if (denom > 1e-12 && denom < 1.0) {
          double need = std::ceil(std::log(1.0 - prob) / std::log(denom));
          if (need < (double)iters)
            iters = std::max((int)need, it + 1);
        } else if (denom <= 1e-12) {
          iters = it + 1;
        }
      }
    }
    // refit E on all inliers of the best sample (least squares over N >= 8 rows) and re-classify; keep the larger set
    if (best >= 8) {
      std::vector<size_t> inl;
      for (size_t i = 0; i < n; i++)
        if (best_mask[i])
          inl.push_back(i);
      Eigen::Matrix3d E;
      if (solveN(b0, b1, inl, E)) {
        int cnt = 0;
        std::vector<unsigned char> m(n, 0);
        for (size_t i = 0; i < n; i++) {
          if (error(E, b0[i], b1[i]) < thresh_rad) {
            m[i] = 1;
            cnt++;
          }
        }
        if (cnt >= best)
          best_mask = m;
      }
    }
    mask = best_mask;
  }

  /// Angular distance [rad] of the bearings from the epipolar planes implied by E (max of the two views)
  static double error(const Eigen::Matrix3d &E, const Eigen::Vector3d &p, const Eigen::Vector3d &q) {
    Eigen::Vector3d n1 = E * p;
    Eigen::Vector3d n0 = E.transpose() * q;
    double e1 = (n1.norm() > 1e-12) ? std::abs(q.dot(n1)) / n1.norm() : 0.0;
    double e0 = (n0.norm() > 1e-12) ? std::abs(p.dot(n0)) / n0.norm() : 0.0;
    return std::asin(std::min(1.0, std::max(e0, e1)));
  }

  /// Linear 8-point solve for E (rank-2 enforced). q^T E p = 0 for each pair.
  static bool solve8(const std::vector<Eigen::Vector3d> &b0, const std::vector<Eigen::Vector3d> &b1, const std::vector<size_t> &idx,
                     Eigen::Matrix3d &E) {
    return solveN(b0, b1, idx, E);
  }

  /// Linear least-squares solve for E over N >= 8 correspondences (rank-2 enforced)
  static bool solveN(const std::vector<Eigen::Vector3d> &b0, const std::vector<Eigen::Vector3d> &b1, const std::vector<size_t> &idx,
                     Eigen::Matrix3d &E) {
    if (idx.size() < 8)
      return false;
    Eigen::MatrixXd A(idx.size(), 9);
    for (size_t r = 0; r < idx.size(); r++) {
      const Eigen::Vector3d &p = b0[idx[r]];
      const Eigen::Vector3d &q = b1[idx[r]];
      A.row(r) << q(0) * p(0), q(0) * p(1), q(0) * p(2), q(1) * p(0), q(1) * p(1), q(1) * p(2), q(2) * p(0), q(2) * p(1), q(2) * p(2);
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    Eigen::Matrix<double, 9, 1> e = svd.matrixV().col(8);
    E << e(0), e(1), e(2), e(3), e(4), e(5), e(6), e(7), e(8);
    if (!std::isfinite(E.sum()))
      return false;
    Eigen::JacobiSVD<Eigen::Matrix3d> s2(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3d sv = s2.singularValues();
    sv(2) = 0.0;
    E = s2.matrixU() * sv.asDiagonal() * s2.matrixV().transpose();
    return true;
  }
};

} // namespace ov_core

#endif /* OV_CORE_BEARING_RANSAC_H */
