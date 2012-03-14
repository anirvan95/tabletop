/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <fstream>
#include <iostream>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <ecto/ecto.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tabletop/table/tabletop_segmenter.h>

using ecto::tendrils;

namespace tabletop
{
  /** Ecto implementation of a module that takes
   *
   */
  struct Clusterer
  {
    static void
    declare_params(ecto::tendrils& params)
    {
      params.declare(&Clusterer::clustering_voxel_size_, "clustering_voxel_size",
                     "The minimum number of points deemed necessary to find a table.", 0.003);
      params.declare(&Clusterer::cluster_distance_, "cluster_distance", "The size of a voxel cell when downsampling ",
                     0.01);
      params.declare(&Clusterer::min_cluster_size_, "min_cluster_size", "Min number of points for a cluster", 300);
      params.declare(&Clusterer::table_z_filter_min_, "table_z_filter_min",
                     "Min distance (in meters) from the table to get clusters from.", 0.01);
      params.declare(&Clusterer::table_z_filter_max_, "table_z_filter_max",
                     "Max distance (in meters) from the table to get clusters from.", 0.5);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&Clusterer::cloud_, "cloud", "The point cloud in which to find the clusters.");
      inputs.declare(&Clusterer::clouds_hull_, "clouds_hull", "The hull in which to find the clusters.");

      outputs.declare(&Clusterer::clusters_, "clusters", "For each table, a vector of clusters.");
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
    }

    /** Get the 2d keypoints and figure out their 3D position from the depth map
     * @param inputs
     * @param outputs
     * @return
     */
    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      BlobSegmenter blob_segmenter(*clustering_voxel_size_, *cluster_distance_, *min_cluster_size_,
                                   *table_z_filter_min_, *table_z_filter_max_);

      clusters_->clear();
      for (size_t table_index = 0; table_index < clouds_hull_->size(); ++table_index)
      {
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
        blob_segmenter.process<pcl::PointXYZ>(*cloud_, (*clouds_hull_)[table_index], clusters);

        clusters_->push_back(clusters);
      }

      return ecto::OK;
    }
  private:
    /** Size of downsampling grid before performing clustering */
    ecto::spore<float> clustering_voxel_size_;
    /** Min distance between two clusters */
    ecto::spore<float> cluster_distance_;
    /** Min number of points for a cluster */
    ecto::spore<int> min_cluster_size_;
    /** Limits used when clustering points off the plane */
    ecto::spore<float> table_z_filter_min_;
    ecto::spore<float> table_z_filter_max_;

    /** The input cloud */
    ecto::spore<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> cloud_;
    /** The hull of the input cloud */
    ecto::spore<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> > clouds_hull_;
    /** The resulting clusters: for each table, return a vector of clusters */
    ecto::spore<std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> > > clusters_;
  };
}

ECTO_CELL(tabletop_table, tabletop::Clusterer, "Clusterer",
          "Given a point cloud and the hull of the table, find clusters.");
