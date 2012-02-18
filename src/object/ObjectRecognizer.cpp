/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <tabletop/Table.h>
#include <tabletop/table/tabletop_segmenter.h>
#include <tabletop/object/tabletop_object_detector.h>

#include <object_recognition/common/pose_result.h>
#include <object_recognition/db/ModelReader.h>

#include "db/db_mesh.h"

using object_recognition::common::PoseResult;

using ecto::tendrils;

namespace tabletop
{
  /** Ecto implementation of a module that recognizes objects using the tabletop code
   *
   */
  struct ObjectRecognizer: public object_recognition::db::bases::ModelReaderImpl
  {
    virtual void
    ParameterCallback(const object_recognition::db::Documents & db_documents)
    {
      object_recognizer_ = tabletop_object_detector::TabletopObjectRecognizer();
      BOOST_FOREACH(const object_recognition::db::Document & document, db_documents)
          {
            int model_id = document.get_value("model_id").get_int();
            arm_navigation_msgs::Shape mesh;
            document.get_attachment("mesh", mesh);

            object_recognizer_.addObject(model_id, mesh);
          }
    }

    static void
    declare_params(ecto::tendrils& params)
    {
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&ObjectRecognizer::clusters_, "clusters", "The object clusters.").required(true);

      outputs.declare(&ObjectRecognizer::pose_results_, "pose_results", "The results of object recognition");
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
    }

    /** Compute the pose of the table plane
     * @param inputs
     * @param outputs
     * @return
     */
    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      object_recognizer_.objectDetection<pcl::PointCloud<pcl::PointXYZ> >(*clusters_, num_models_, perform_fit_merge_,
                                                                          *raw_fit_results, *cluster_model_indices_);
      BOOST_FOREACH(const ModelFitInfos & model_fit_infos, *raw_fit_results)
          {
            BOOST_FOREACH(const tabletop_object_detector::ModelFitInfo & model_fit_info, model_fit_infos)
                {
                  PoseResult pose_result;
                  geometry_msgs::Pose pose = model_fit_info.getPose();
                  pose_result.set_T(Eigen::Vector3f(pose.position.x, pose.position.y, pose.position.z));
                  pose_result.set_R(
                      Eigen::Quaternionf(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                                         pose.orientation.z));
                  pose_results_->push_back(pose_result);
                }
          }

      return ecto::OK;
    }
  private:
    typedef std::vector<tabletop_object_detector::ModelFitInfo> ModelFitInfos;
    /** The object recognizer */
    tabletop_object_detector::TabletopObjectRecognizer object_recognizer_;
    /** The resulting poses of the objects */
    ecto::spore<std::vector<PoseResult> > pose_results_;

    ecto::spore<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> > clusters_;
    int num_models_;
    bool perform_fit_merge_;
    ecto::spore<std::vector<ModelFitInfos> > raw_fit_results;
    ecto::spore<std::vector<size_t> > cluster_model_indices_;
  };
}

ECTO_CELL(tabletop_object, object_recognition::db::bases::ModelReaderBase<tabletop::ObjectRecognizer>,
          "ObjectRecognizer", "Given clusters on a table, identify them as objects.");
