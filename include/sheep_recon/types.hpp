#pragma once

#include <cmath>

#include <pcl/point_types.h>

namespace sheep_recon {

using PointT = ::pcl::PointXYZRGB;
using Cloud = ::pcl::PointCloud<PointT>;
using CloudPtr = Cloud::Ptr;
using CloudConstPtr = Cloud::ConstPtr;

struct ReconParams {
  int sor_k = 50;
  double sor_stddev = 1.0;
  double mls_search_radius = 0.03;
  double mls_sqr_gauss = 0.0025;
  double voxel_leaf = 0.005;
  double ransac_dist = 0.01;
  int ransac_max_iter = 1000;

  double icp_corr_dist = 0.04;
  double icp_corr_coarse = 0.12;
  int icp_max_iter = 100;
  int icp_coarse_max_iter = 30;
  double icp_epsilon = 1e-8;

  /// ELCH 后：每帧对「除自己外融合」做 ICP，纠正单帧链式/闭环 residual
  int loo_refine_iters = 2;
  double loo_target_voxel = 0.022;
  /// strip + multistart 之后再跑 LOO 的轮数（0 关闭）
  int loo_after_consensus_iters = 1;

  /// 相对其余帧融合表面：删掉离共识过远的点（错角度重叠层、悬空半截腿）
  bool consensus_strip_enable = true;
  double consensus_nn_max_dist = 0.038;

  /// 对「与共识偏差最大」的那一帧，绕 Z 做多初值 yaw + ICP，跳出错误局部最优
  bool worst_yaw_multistart_enable = true;
  float multistart_yaw_step_deg = 4.f;
  int multistart_yaw_half_steps = 5;

  double fuse_voxel = 0.012;
  bool passthrough_enable = true;
  double passthrough_z_min = 0.12;
  double passthrough_z_max = 2.0;

  /// 融合后欧式聚类：只保留最大簇
  bool cluster_largest_enable = true;
  double cluster_tolerance = 0.028;
  int cluster_min_size = 150;
  int cluster_max_size = 20000000;

  /// 聚类之后：统计离群（羊身飞点） + 半径离群（孤立碎屑、小地面点）
  bool fused_sor_enable = true;
  int fused_sor_k = 35;
  double fused_sor_stddev = 0.9;
  bool fused_ror_enable = true;
  double fused_ror_radius = 0.022;
  int fused_ror_min_neighbors = 12;

  double mesh_voxel = 0.015;
  double normal_radius = 0.04;

  /// Greedy Projection Triangulation (PCL gp3)
  double gpt_search_radius = 0.055;
  double gpt_mu = 2.5;
  int gpt_max_nn = 120;
  double gpt_max_surf_angle = M_PI / 4.0;
  double gpt_min_angle = M_PI / 18.0;
  double gpt_max_angle = 2.0 * M_PI / 3.0;
  bool gpt_normal_consistency = false;
};

}  // namespace sheep_recon
