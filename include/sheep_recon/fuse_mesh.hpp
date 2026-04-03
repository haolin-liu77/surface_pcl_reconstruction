#pragma once

#include <string>
#include <vector>

#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/PolygonMesh.h>

#include "sheep_recon/types.hpp"

namespace sheep_recon {

inline CloudPtr fuse_voxel(const ::std::vector<CloudPtr>& clouds, float leaf) {
  CloudPtr merged(new Cloud);
  for (const auto& c : clouds) *merged += *c;
  CloudPtr out(new Cloud);
  ::pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(merged);
  vg.setLeafSize(leaf, leaf, leaf);
  vg.filter(*out);
  return out;
}

inline CloudPtr passthrough_z(CloudConstPtr in, const ReconParams& p) {
  ::pcl::PassThrough<PointT> pt;
  pt.setInputCloud(in);
  pt.setFilterFieldName("z");
  pt.setFilterLimits(static_cast<float>(p.passthrough_z_min), static_cast<float>(p.passthrough_z_max));
  CloudPtr out(new Cloud);
  pt.filter(*out);
  return out;
}

inline int save_pcd_binary(const ::std::string& path, CloudConstPtr c) {
  return ::pcl::io::savePCDFileBinary(path, *c);
}

inline CloudPtr filter_fused_outliers(CloudConstPtr in, const ReconParams& p) {
  CloudPtr work(new Cloud(*in));
  if (p.fused_sor_enable) {
    CloudPtr t(new Cloud);
    ::pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(work);
    sor.setMeanK(p.fused_sor_k);
    sor.setStddevMulThresh(p.fused_sor_stddev);
    sor.filter(*t);
    work = t;
  }
  if (p.fused_ror_enable) {
    CloudPtr t(new Cloud);
    ::pcl::RadiusOutlierRemoval<PointT> ror;
    ror.setInputCloud(work);
    ror.setRadiusSearch(static_cast<float>(p.fused_ror_radius));
    ror.setMinNeighborsInRadius(p.fused_ror_min_neighbors);
    ror.filter(*t);
    work = t;
  }
  return work;
}

inline ::pcl::PointCloud<::pcl::PointXYZ>::Ptr voxel_down_xyz(CloudConstPtr in, float leaf) {
  ::pcl::PointCloud<::pcl::PointXYZ>::Ptr xyz(new ::pcl::PointCloud<::pcl::PointXYZ>);
  ::pcl::copyPointCloud(*in, *xyz);
  ::pcl::VoxelGrid<::pcl::PointXYZ> vg;
  vg.setInputCloud(xyz);
  vg.setLeafSize(leaf, leaf, leaf);
  ::pcl::PointCloud<::pcl::PointXYZ>::Ptr out(new ::pcl::PointCloud<::pcl::PointXYZ>);
  vg.filter(*out);
  return out;
}

inline void greedy_projection_mesh(::pcl::PointCloud<::pcl::PointXYZ>::ConstPtr cloud, const ReconParams& p,
                                   ::pcl::PolygonMesh& mesh) {
  ::pcl::NormalEstimation<::pcl::PointXYZ, ::pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ::pcl::search::KdTree<::pcl::PointXYZ>::Ptr tree(new ::pcl::search::KdTree<::pcl::PointXYZ>);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(static_cast<float>(p.normal_radius));
  ::pcl::PointCloud<::pcl::Normal>::Ptr normals(new ::pcl::PointCloud<::pcl::Normal>);
  ne.compute(*normals);
  ::pcl::PointCloud<::pcl::PointNormal>::Ptr pts(new ::pcl::PointCloud<::pcl::PointNormal>);
  ::pcl::concatenateFields(*cloud, *normals, *pts);

  ::pcl::GreedyProjectionTriangulation<::pcl::PointNormal> gp3;
  gp3.setSearchRadius(static_cast<float>(p.gpt_search_radius));
  gp3.setMu(static_cast<float>(p.gpt_mu));
  gp3.setMaximumNearestNeighbors(p.gpt_max_nn);
  gp3.setMaximumSurfaceAngle(static_cast<float>(p.gpt_max_surf_angle));
  gp3.setMinimumAngle(static_cast<float>(p.gpt_min_angle));
  gp3.setMaximumAngle(static_cast<float>(p.gpt_max_angle));
  gp3.setNormalConsistency(p.gpt_normal_consistency);
  gp3.setInputCloud(pts);
  gp3.reconstruct(mesh);
}

}  // namespace sheep_recon
