#pragma once

#include <memory>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>

#include "sheep_recon/types.hpp"

namespace sheep_recon {

inline CloudPtr process_one_frame(CloudConstPtr input, const ReconParams& p) {
  CloudPtr a(new Cloud);
  ::pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(input);
  sor.setMeanK(p.sor_k);
  sor.setStddevMulThresh(p.sor_stddev);
  sor.filter(*a);

  ::pcl::search::KdTree<PointT>::Ptr tree(new ::pcl::search::KdTree<PointT>);
  ::pcl::MovingLeastSquares<PointT, PointT> mls;
  mls.setInputCloud(a);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(static_cast<float>(p.mls_search_radius));
  mls.setSqrGaussParam(static_cast<float>(p.mls_sqr_gauss));
  mls.setPolynomialOrder(2);
  CloudPtr b(new Cloud);
  mls.process(*b);

  CloudPtr c(new Cloud);
  ::pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(b);
  vg.setLeafSize(static_cast<float>(p.voxel_leaf), static_cast<float>(p.voxel_leaf),
                 static_cast<float>(p.voxel_leaf));
  vg.filter(*c);

  ::pcl::ModelCoefficients coeffs;
  ::pcl::PointIndices inliers;
  ::pcl::SACSegmentation<PointT> seg;
  seg.setInputCloud(c);
  seg.setModelType(::pcl::SACMODEL_PLANE);
  seg.setMethodType(::pcl::SAC_RANSAC);
  seg.setDistanceThreshold(static_cast<float>(p.ransac_dist));
  seg.setMaxIterations(p.ransac_max_iter);
  seg.segment(inliers, coeffs);

  if (inliers.indices.empty()) return c;

  ::pcl::ExtractIndices<PointT> ex;
  ex.setInputCloud(c);
  ex.setIndices(::std::make_shared<::pcl::PointIndices>(inliers));
  ex.setNegative(true);
  CloudPtr out(new Cloud);
  ex.filter(*out);
  return out;
}

}  // namespace sheep_recon
