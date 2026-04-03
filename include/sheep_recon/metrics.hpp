#pragma once

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

#include "sheep_recon/registration.hpp"
#include "sheep_recon/types.hpp"

namespace sheep_recon {

struct PipelineTimingsMs {
  double load = 0, preprocess = 0, sequential_icp = 0, apply_transforms = 0;
  double elch = 0, loo = 0, strip = 0, yaw_ms = 0, loo_after = 0;
  double fuse = 0, passthrough = 0, cluster = 0, fused_outliers = 0;
  double mesh_down = 0, mesh_recon = 0;
  double save_fused_pcd = 0, save_mesh_ply = 0;
};

inline double elapsed_ms_reset(::std::chrono::steady_clock::time_point& t) {
  auto now = ::std::chrono::steady_clock::now();
  double ms =
      ::std::chrono::duration<double, ::std::milli>(now - t).count();
  t = now;
  return ms;
}

struct RegistrationMetrics {
  double consensus_nn_mean_m = 0;
  double consensus_nn_max_m = 0;
  double consensus_nn_min_m = 0;
  size_t frames = 0;
};

inline RegistrationMetrics compute_registration_metrics(const ::std::vector<CloudPtr>& chain,
                                                        const ReconParams& p) {
  RegistrationMetrics m;
  m.frames = chain.size();
  if (chain.size() < 2) return m;

  const float leaf = static_cast<float>(p.loo_target_voxel);
  double sum = 0;
  double vmin = 1e100;
  double vmax = 0;
  for (size_t i = 0; i < chain.size(); ++i) {
    CloudPtr tgt = make_loo_target(chain, i, leaf);
    float d = mean_nn_distance(chain[i], tgt);
    sum += d;
    vmin = ::std::min(vmin, static_cast<double>(d));
    vmax = ::std::max(vmax, static_cast<double>(d));
  }
  m.consensus_nn_mean_m = sum / static_cast<double>(chain.size());
  m.consensus_nn_max_m = vmax;
  m.consensus_nn_min_m = vmin;
  return m;
}

inline ::std::string format_metrics_report(const PipelineTimingsMs& tm, const RegistrationMetrics& reg,
                                           size_t fused_points, size_t mesh_xyz_points,
                                           size_t mesh_polygons, double total_wall_ms) {
  ::std::string o;
  o += "=== wall_time_ms ===\n";
  o += "total " + ::std::to_string(total_wall_ms) + "\n";
  o += "load_pcfs " + ::std::to_string(tm.load) + "\n";
  o += "preprocess_frames " + ::std::to_string(tm.preprocess) + "\n";
  o += "sequential_icp " + ::std::to_string(tm.sequential_icp) + "\n";
  o += "apply_transforms " + ::std::to_string(tm.apply_transforms) + "\n";
  o += "elch " + ::std::to_string(tm.elch) + "\n";
  o += "loo_refine " + ::std::to_string(tm.loo) + "\n";
  o += "strip_consensus " + ::std::to_string(tm.strip) + "\n";
  o += "yaw_multistart " + ::std::to_string(tm.yaw_ms) + "\n";
  o += "loo_after_consensus " + ::std::to_string(tm.loo_after) + "\n";
  o += "fuse_voxel " + ::std::to_string(tm.fuse) + "\n";
  o += "passthrough_z " + ::std::to_string(tm.passthrough) + "\n";
  o += "cluster_largest " + ::std::to_string(tm.cluster) + "\n";
  o += "fused_sor_ror " + ::std::to_string(tm.fused_outliers) + "\n";
  o += "mesh_voxel_down " + ::std::to_string(tm.mesh_down) + "\n";
  o += "mesh_greedy " + ::std::to_string(tm.mesh_recon) + "\n";
  o += "save_fused_pcd " + ::std::to_string(tm.save_fused_pcd) + "\n";
  o += "save_mesh_ply " + ::std::to_string(tm.save_mesh_ply) + "\n";
  o += "=== registration_quality (lower mean/max NN to LOO consensus is better) ===\n";
  o += "frames " + ::std::to_string(reg.frames) + "\n";
  o += "consensus_nn_mean_m " + ::std::to_string(reg.consensus_nn_mean_m) + "\n";
  o += "consensus_nn_max_m " + ::std::to_string(reg.consensus_nn_max_m) + "\n";
  o += "consensus_nn_min_m " + ::std::to_string(reg.consensus_nn_min_m) + "\n";
  o += "=== reconstruction_io ===\n";
  o += "fused_points " + ::std::to_string(fused_points) + "\n";
  o += "mesh_input_points_xyz " + ::std::to_string(mesh_xyz_points) + "\n";
  o += "mesh_triangles " + ::std::to_string(mesh_polygons) + "\n";
  return o;
}

inline void print_and_save_metrics(const ::std::string& report, const ::std::string& out_txt_path) {
  ::std::cout << report;
  ::std::ofstream f(out_txt_path);
  if (f) f << report;
}

}  // namespace sheep_recon
