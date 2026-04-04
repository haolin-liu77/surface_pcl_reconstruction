#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/PolygonMesh.h>

#include "sheep_recon/cluster.hpp"
#include "sheep_recon/fuse_mesh.hpp"
#include "sheep_recon/metrics.hpp"
#include "sheep_recon/preprocess.hpp"
#include "sheep_recon/registration.hpp"
#include "sheep_recon/types.hpp"

namespace fs = ::std::filesystem;

static void collect_pcds(const fs::path& root, ::std::vector<fs::path>& out) {
  if (!fs::exists(root)) return;
  for (auto it = fs::recursive_directory_iterator(root); it != fs::recursive_directory_iterator(); ++it) {
    if (!it->is_regular_file()) continue;
    if (it->path().extension() != ".pcd") continue;
    out.push_back(it->path());
  }
  ::std::sort(out.begin(), out.end());
}

int main(int argc, char** argv) {
  const auto t_wall0 = ::std::chrono::steady_clock::now();
  fs::path dataset = "dataset";
  fs::path output = "output";
  for (int i = 1; i < argc; ++i) {
    ::std::string a = argv[i];
    if (a == "--dataset" && i + 1 < argc) dataset = argv[++i];
    else if (a == "--output" && i + 1 < argc) output = argv[++i];
  }

  sheep_recon::ReconParams params;
  sheep_recon::PipelineTimingsMs tm;

  ::std::vector<fs::path> paths;
  collect_pcds(dataset, paths);
  if (paths.empty()) {
    ::std::cerr << "no .pcd under " << dataset << "\n";
    return 1;
  }

  auto t = ::std::chrono::steady_clock::now();
  ::std::cout << "loaded " << paths.size() << " pcd(s)\n";
  ::std::vector<sheep_recon::CloudConstPtr> raw;
  raw.reserve(paths.size());
  for (const auto& p : paths) {
    auto c = ::std::make_shared<sheep_recon::Cloud>();
    if (::pcl::io::loadPCDFile<::pcl::PointXYZRGB>(p.string(), *c) < 0) {
      ::std::cerr << "load failed: " << p << "\n";
      return 1;
    }
    raw.push_back(c);
  }
  tm.load = sheep_recon::elapsed_ms_reset(t);

  ::std::vector<sheep_recon::CloudConstPtr> sheep;
  sheep.reserve(raw.size());
  for (const auto& c : raw) sheep.push_back(sheep_recon::process_one_frame(c, params));
  tm.preprocess = sheep_recon::elapsed_ms_reset(t);

  ::std::vector<Eigen::Matrix4f> T;
  if (!sheep_recon::sequential_icp_chain(sheep, params, T)) {
    ::std::cerr << "sequential ICP failed\n";
    return 1;
  }
  tm.sequential_icp = sheep_recon::elapsed_ms_reset(t);

  ::std::vector<sheep_recon::CloudPtr> chain = sheep_recon::apply_transforms(sheep, T);
  tm.apply_transforms = sheep_recon::elapsed_ms_reset(t);

  if (chain.size() >= 2) {
    sheep_recon::elch_optimize(chain, params);
    tm.elch = sheep_recon::elapsed_ms_reset(t);

    sheep_recon::refine_loo_icp(chain, params);
    tm.loo = sheep_recon::elapsed_ms_reset(t);

    sheep_recon::strip_consensus_outliers(chain, params);
    tm.strip = sheep_recon::elapsed_ms_reset(t);

    sheep_recon::refine_worst_yaw_multistart(chain, params);
    tm.yaw_ms = sheep_recon::elapsed_ms_reset(t);

    sheep_recon::refine_loo_icp(chain, params, params.loo_after_consensus_iters);
    tm.loo_after = sheep_recon::elapsed_ms_reset(t);
  }

  sheep_recon::RegistrationMetrics reg = sheep_recon::compute_registration_metrics(chain, params);

  fs::create_directories(output);
  t = ::std::chrono::steady_clock::now();

  auto fused = sheep_recon::fuse_voxel(chain, static_cast<float>(params.fuse_voxel));
  tm.fuse = sheep_recon::elapsed_ms_reset(t);

  if (params.passthrough_enable) fused = sheep_recon::passthrough_z(fused, params);
  tm.passthrough = sheep_recon::elapsed_ms_reset(t);

  fused = sheep_recon::extract_largest_euclidean_cluster(fused, params);
  tm.cluster = sheep_recon::elapsed_ms_reset(t);

  fused = sheep_recon::filter_fused_outliers(fused, params);
  tm.fused_outliers = sheep_recon::elapsed_ms_reset(t);

  ::std::string fused_path = (output / "fused.pcd").string();
  {
    const auto ts = ::std::chrono::steady_clock::now();
    if (sheep_recon::save_pcd_binary(fused_path, fused) < 0) return 1;
    tm.save_fused_pcd = ::std::chrono::duration<double, ::std::milli>(
                            ::std::chrono::steady_clock::now() - ts)
                            .count();
  }
  ::std::cout << "wrote " << fused_path << "\n";

  t = ::std::chrono::steady_clock::now();
  auto mesh_cloud = sheep_recon::voxel_down_for_mesh(fused, static_cast<float>(params.mesh_voxel));
  tm.mesh_down = sheep_recon::elapsed_ms_reset(t);

  ::pcl::PolygonMesh mesh;
  sheep_recon::greedy_projection_mesh(mesh_cloud, params, mesh);
  tm.mesh_recon = sheep_recon::elapsed_ms_reset(t);

  ::std::string mesh_path = (output / "mesh_greedy.ply").string();
  {
    const auto ts = ::std::chrono::steady_clock::now();
    if (::pcl::io::savePLYFile(mesh_path, mesh) < 0) return 1;
    tm.save_mesh_ply = ::std::chrono::duration<double, ::std::milli>(
                           ::std::chrono::steady_clock::now() - ts)
                           .count();
  }
  ::std::cout << "wrote " << mesh_path << "\n";

  const double total_ms = ::std::chrono::duration<double, ::std::milli>(
                              ::std::chrono::steady_clock::now() - t_wall0)
                              .count();
  ::std::string report = sheep_recon::format_metrics_report(tm, reg, fused->size(),
                                                             mesh_cloud->size(), mesh.polygons.size(),
                                                             total_ms);
  sheep_recon::print_and_save_metrics(report, (output / "metrics.txt").string());

  return 0;
}
