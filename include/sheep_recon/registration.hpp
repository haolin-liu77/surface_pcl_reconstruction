#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/elch.h>

#include "sheep_recon/types.hpp"

namespace sheep_recon {

inline CloudPtr make_loo_target(const ::std::vector<CloudPtr>& chain, size_t skip_i, float voxel_leaf) {
  CloudPtr merged(new Cloud);
  for (size_t j = 0; j < chain.size(); ++j) {
    if (j != skip_i) *merged += *chain[j];
  }
  CloudPtr tgt(new Cloud);
  ::pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(merged);
  vg.setLeafSize(voxel_leaf, voxel_leaf, voxel_leaf);
  vg.filter(*tgt);
  return tgt;
}

inline float mean_nn_distance(CloudConstPtr src, CloudConstPtr tgt) {
  if (src->empty() || tgt->empty()) return 0.f;
  ::pcl::KdTreeFLANN<PointT> kdt;
  kdt.setInputCloud(tgt);
  float s = 0.f;
  ::std::vector<int> id(1);
  ::std::vector<float> d2(1);
  for (const auto& pt : src->points) {
    kdt.nearestKSearch(pt, 1, id, d2);
    s += std::sqrt(d2[0]);
  }
  return s / static_cast<float>(src->size());
}

inline bool run_icp(CloudConstPtr src, CloudConstPtr tgt, float max_dist, int max_iter, double epsilon,
                    Eigen::Matrix4f& T_out) {
  ::pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setInputSource(src);
  icp.setInputTarget(tgt);
  icp.setMaxCorrespondenceDistance(max_dist);
  icp.setMaximumIterations(max_iter);
  icp.setTransformationEpsilon(static_cast<float>(epsilon));
  icp.setEuclideanFitnessEpsilon(static_cast<float>(epsilon));
  Cloud aligned;
  icp.align(aligned);
  if (!icp.hasConverged()) return false;
  T_out = icp.getFinalTransformation();
  return true;
}

inline bool two_stage_icp_pair(CloudConstPtr src_orig, CloudConstPtr tgt, const ReconParams& p,
                               Eigen::Matrix4f& T_total) {
  Eigen::Matrix4f T12;
  if (!run_icp(src_orig, tgt, static_cast<float>(p.icp_corr_coarse), p.icp_coarse_max_iter, p.icp_epsilon, T12))
    return false;
  CloudPtr mid(new Cloud);
  ::pcl::transformPointCloud(*src_orig, *mid, T12);
  Eigen::Matrix4f T12f;
  if (!run_icp(mid, tgt, static_cast<float>(p.icp_corr_dist), p.icp_max_iter, p.icp_epsilon, T12f))
    return false;
  T_total = T12f * T12;
  return true;
}

inline bool sequential_icp_chain(const ::std::vector<CloudConstPtr>& sheep, const ReconParams& p,
                                 ::std::vector<Eigen::Matrix4f>& T_to0) {
  const size_t n = sheep.size();
  T_to0.assign(n, Eigen::Matrix4f::Identity());
  if (n < 2) return true;
  for (size_t k = 1; k < n; ++k) {
    Eigen::Matrix4f T_pair;
    if (!two_stage_icp_pair(sheep[k], sheep[k - 1], p, T_pair)) return false;
    T_to0[k] = T_to0[k - 1] * T_pair;
  }
  return true;
}

inline ::std::vector<CloudPtr> apply_transforms(const ::std::vector<CloudConstPtr>& sheep,
                                                const ::std::vector<Eigen::Matrix4f>& T_to0) {
  ::std::vector<CloudPtr> out;
  out.reserve(sheep.size());
  for (size_t i = 0; i < sheep.size(); ++i) {
    CloudPtr c(new Cloud);
    ::pcl::transformPointCloud(*sheep[i], *c, T_to0[i]);
    out.push_back(c);
  }
  return out;
}

inline void elch_optimize(::std::vector<CloudPtr>& chain, const ReconParams& p) {
  if (chain.size() < 2) return;
  ::pcl::registration::ELCH<PointT> elch;
  elch.getReg()->setMaxCorrespondenceDistance(static_cast<float>(p.icp_corr_dist));
  elch.getReg()->setMaximumIterations(p.icp_max_iter);
  elch.getReg()->setTransformationEpsilon(static_cast<float>(p.icp_epsilon));
  elch.getReg()->setEuclideanFitnessEpsilon(static_cast<float>(p.icp_epsilon));
  for (auto& c : chain) elch.addPointCloud(c);
  elch.setLoopStart(0);
  elch.setLoopEnd(static_cast<int>(chain.size()) - 1);
  elch.compute();
}

inline void refine_loo_icp(::std::vector<CloudPtr>& chain, const ReconParams& p, int iters_override = -1) {
  const size_t n = chain.size();
  const int nloop = iters_override >= 0 ? iters_override : p.loo_refine_iters;
  if (n < 2 || nloop <= 0) return;

  const float leaf = static_cast<float>(p.loo_target_voxel);

  for (int it = 0; it < nloop; ++it) {
    for (size_t i = 0; i < n; ++i) {
      CloudPtr tgt = make_loo_target(chain, i, leaf);
      Eigen::Matrix4f T;
      if (!two_stage_icp_pair(chain[i], tgt, p, T)) continue;

      Cloud aligned;
      ::pcl::transformPointCloud(*chain[i], aligned, T);
      *chain[i] = aligned;
    }
  }
}

inline void strip_consensus_outliers(::std::vector<CloudPtr>& chain, const ReconParams& p) {
  if (!p.consensus_strip_enable || chain.size() < 2) return;

  const float leaf = static_cast<float>(p.loo_target_voxel);
  const float dmax = static_cast<float>(p.consensus_nn_max_dist);
  ::pcl::KdTreeFLANN<PointT> kdt;
  ::std::vector<int> id(1);
  ::std::vector<float> d2(1);

  for (size_t i = 0; i < chain.size(); ++i) {
    CloudPtr tgt = make_loo_target(chain, i, leaf);
    if (tgt->empty()) continue;
    kdt.setInputCloud(tgt);
    CloudPtr kept(new Cloud);
    kept->reserve(chain[i]->size());
    for (const auto& pt : chain[i]->points) {
      kdt.nearestKSearch(pt, 1, id, d2);
      if (d2[0] <= dmax * dmax) kept->push_back(pt);
    }
    chain[i]->swap(*kept);
  }
}

inline void refine_worst_yaw_multistart(::std::vector<CloudPtr>& chain, const ReconParams& p) {
  if (!p.worst_yaw_multistart_enable || chain.size() < 2) return;

  const float leaf = static_cast<float>(p.loo_target_voxel);
  const int h = p.multistart_yaw_half_steps;
  if (h <= 0) return;

  size_t worst = 0;
  float worst_mean = 0.f;
  for (size_t i = 0; i < chain.size(); ++i) {
    CloudPtr tgt = make_loo_target(chain, i, leaf);
    float m = mean_nn_distance(chain[i], tgt);
    if (i == 0 || m > worst_mean) {
      worst_mean = m;
      worst = i;
    }
  }

  CloudPtr tgt = make_loo_target(chain, worst, leaf);
  if (tgt->empty()) return;

  Cloud best = *chain[worst];
  float best_m = mean_nn_distance(chain[worst], tgt);

  for (int k = -h; k <= h; ++k) {
    const float rad = float(k) * p.multistart_yaw_step_deg * static_cast<float>(M_PI) / 180.f;
    Eigen::Matrix4f R = Eigen::Matrix4f::Identity();
    R.topLeftCorner<3, 3>() = Eigen::AngleAxisf(rad, Eigen::Vector3f::UnitZ()).toRotationMatrix();

    CloudPtr rotp(new Cloud);
    ::pcl::transformPointCloud(*chain[worst], *rotp, R);
    Eigen::Matrix4f T_icp;
    if (!two_stage_icp_pair(rotp, tgt, p, T_icp)) continue;
    Cloud cand;
    ::pcl::transformPointCloud(*rotp, cand, T_icp);
    CloudPtr candp(new Cloud(cand));
    float m = mean_nn_distance(candp, tgt);
    if (m < best_m) {
      best_m = m;
      best = cand;
    }
  }
  *chain[worst] = best;
}

}  // namespace sheep_recon
