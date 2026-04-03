#pragma once

#include <vector>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "sheep_recon/types.hpp"

namespace sheep_recon {

/// 融合后保留点数最多的连通簇，去掉小团块（错位后腿、地面碎片等）。
inline CloudPtr extract_largest_euclidean_cluster(CloudConstPtr in, const ReconParams& p) {
  if (!p.cluster_largest_enable) {
    CloudPtr out(new Cloud(*in));
    return out;
  }

  ::pcl::search::KdTree<PointT>::Ptr tree(new ::pcl::search::KdTree<PointT>);
  tree->setInputCloud(in);
  ::std::vector<::pcl::PointIndices> clusters;
  ::pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(static_cast<float>(p.cluster_tolerance));
  ec.setMinClusterSize(p.cluster_min_size);
  ec.setMaxClusterSize(p.cluster_max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(in);
  ec.extract(clusters);

  if (clusters.empty()) {
    CloudPtr out(new Cloud(*in));
    return out;
  }

  size_t best = 0;
  for (size_t i = 1; i < clusters.size(); ++i) {
    if (clusters[i].indices.size() > clusters[best].indices.size()) best = i;
  }

  CloudPtr out(new Cloud);
  out->reserve(clusters[best].indices.size());
  for (int idx : clusters[best].indices) out->push_back((*in)[idx]);
  return out;
}

}  // namespace sheep_recon
