/*
 * Copyright (C) 2009 by Ulrich Friedrich Klank <klank@in.tum.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * Copyright (c) 2009 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: plane_clusters.cpp 17089 2009-06-15 18:52:12Z veedee $
 *
 */
#define BOOST_THREAD

#include "ClusterDetector.h"
#include "SegmentPrototype.h"
#include "XMLTag.h"
#include "SwissRangerReading.h"
#include "BoostUtils.h"


// Sample Consensus
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/msac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>

#include <angles/angles.h>

// Kd Tree
#include <point_cloud_mapping/kdtree/kdtree_ann.h>

// Cloud geometry
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/areas.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/distances.h>
#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/geometry/transforms.h>
#include <point_cloud_mapping/geometry/statistics.h>

#include <sys/time.h>



//#include <utilities/yarp_communication.h>

#define SR_COLS 176
#define SR_ROWS 144
#define DEBUG 1

#define XML_ATTRIBUTE_SR4LO "sr_loid"
#define XML_ATTRIBUTE_PTULO "ptu_loid"

using namespace cop;

using namespace std;
using namespace ros;
//using namespace std_msgs;
using namespace sensor_msgs;
using namespace geometry_msgs;

// Comparison operator for a vector of vectors
bool
  compareRegions (const std::vector<int> &a, const std::vector<int> &b)
{
  return (a.size () < b.size ());
}

  class PlaneClusterResult
  {
    public:
    class ObjectOnTable
    {

     public:
      std::vector<double> cov33;
      geometry_msgs::Point32 center;

    };

    public:
    double a;
    double b;
    double c;
    double d;
    geometry_msgs::Point32 pcenter;
    std::vector<ObjectOnTable> oclusters;
  };


class PlaneClustersSR
{
  public:

    // ROS messages
    PointCloud cloud_in_trans_;

    PointCloud cloud_down_;
    Point leaf_width_;
    PointCloud cloud_annotated_;
    Point32 axis_;

    // Parameters
    string input_cloud_topic_;
    int k_;
    double max_z_;
    int clusters_min_pts_;

    int object_cluster_min_pts_;
    double object_cluster_tolerance_;

    bool need_cloud_data_;

    double sac_distance_threshold_, eps_angle_;

    double delta_z_, object_min_dist_from_table_;

    double min_angle_, max_angle_;


    int downsample_factor_;

    boost::mutex m_mutexUsage;
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    PlaneClustersSR (XMLTag* tag)
    {
      // 0.198669 0 0.980067 0 0 -1 0 0 0.980067 0 -0.198669 0 0 0 0 1
      axis_.x = 0; axis_.y = 0; axis_.z = 1;

      downsample_factor_ = 4; // Use every nth point
      k_ = 2;                  // 5 k-neighbors by default
      max_z_ = 0.03;

      eps_angle_ = 15.0;       // 15 degrees
      eps_angle_ = angles::from_degrees (eps_angle_);          // convert to radians

      clusters_min_pts_ = 10;  // 10 points

      object_cluster_tolerance_ = 0.07;   // 7cm between two objects
      object_cluster_min_pts_ = 30;         // 30 points per object cluster

      delta_z_ = 0.03;                              // consider objects starting at 3cm from the table
      object_min_dist_from_table_ = 0.1; // objects which have their support more 10cm from the table will not be considered

      min_angle_ = 10.0;
      max_angle_ = 170.0;
      // This should be set to whatever the leaf_width factor is in the downsampler
      sac_distance_threshold_ = 0.03;     // 3 cm
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   void
      updateParametersFromServer ()
    {
     /* nh_.getParam ("~input_cloud_topic", input_cloud_topic_);
      nh_.getParam ("~downsample_factor", downsample_factor_);*/
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Obtain a 24-bit RGB coded value from 3 independent <r, g, b> channel values
      * \param r the red channel value
      * \param g the green channel value
      * \param b the blue channel value
      */
    inline double
      getRGB (float r, float g, float b)
    {
      int res = (int(r * 255) << 16) | (int(g*255) << 8) | int(b*255);
      double rgb = *(float*)(&res);
      return (rgb);
    }


    void cloud_trans (unsigned long swissranger_jlo_id, unsigned long ptu_base_jlo_id,  Point32& viewpoint_cloud, const PointCloud& cloud_in)
    {
       RelPose* pose = RelPoseFactory::GetRelPose(swissranger_jlo_id, ptu_base_jlo_id);

       if(pose == NULL)
       {
          RelPose* pose1 = RelPoseFactory::GetRelPose("/sr4");
          RelPose* pose2 = RelPoseFactory::GetRelPose("/base_link");
          if(pose1 && pose2)
          {
            pose = RelPoseFactory::GetRelPose(pose1->m_uniqueID, pose2->m_uniqueID);
            RelPoseFactory::FreeRelPose(&pose1);
            RelPoseFactory::FreeRelPose(&pose2);
          }
          if(pose == NULL)
           throw "Cloud trans not possible: Location not clear";
       }
       Matrix m = pose->GetMatrix(0);

       RelPoseFactory::FreeRelPose(&pose);

       Matrix m_tmp = m;
       cloud_in_trans_.points.clear();
       if(cloud_in.points.size() == 0 || cloud_in.points.size() > 1000000000)
          throw "Error in pointcloud, failed check 0 =< num_points < 1000000000";
       for(size_t i = 0; i < cloud_in.points.size(); i++)
       {
         ColumnVector v(4);
         v <<  cloud_in.points[i].x <<  cloud_in.points[i].y <<  cloud_in.points[i].z << 1;
         ColumnVector a = m_tmp*v;
         Point32 pt;
         pt.x = a.element(0);
         pt.y = a.element(1);
         pt.z = a.element(2);
         cloud_in_trans_.points.push_back(pt);
       }
       cloud_in_trans_.channels.clear();
       for(size_t channels = 0 ; channels < cloud_in.channels.size(); channels++)
       {

         cloud_in_trans_.channels.push_back(ChannelFloat32());
         cloud_in_trans_.channels[channels].name = cloud_in.channels[channels].name;
         for(size_t points = 0; points < cloud_in.channels[channels].values.size(); points++)
         {
           cloud_in_trans_.channels[channels].values.push_back(
           cloud_in.channels[channels].values[points]);
         }
       }
       ColumnVector vp(4);
       vp <<  viewpoint_cloud.x <<  viewpoint_cloud.y << viewpoint_cloud.z << 1;
       ColumnVector ap = m_tmp*vp;
       viewpoint_cloud.x = ap.element(0);
       viewpoint_cloud.y = ap.element(1);
       viewpoint_cloud.z = ap.element(2);
   }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool
      plane_clusters_service (PlaneClusterResult &resp, int swissranger_jlo_id, int ptu_base_jlo_id, const PointCloud& cloud, bool parallel)
    {
      printf("plane_clusters_service::lock!\n");
      m_mutexUsage.lock();
      try
      {
        /*ROS_INFO ("Service request initiated.");
        updateParametersFromServer ();*/
        Point32 vp;
        vp.x = vp.y = vp.z = 0.0;
        cloud_trans(swissranger_jlo_id, ptu_base_jlo_id, vp, cloud);
        if(!detectTable (cloud_in_trans_, resp, vp, parallel))
        {
          printf("plane_clusters_service::unlock!\n");

          m_mutexUsage.unlock();
          return false;
        }
        ROS_INFO ("Service request terminated.");
        printf("plane_clusters_service::lock!\n");
        m_mutexUsage.unlock();
        return (true);
      }
      catch(const char text)
      {
       printf("plane_clusters_service::lock!\n");
       m_mutexUsage.unlock();
        throw text;
      }
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool
      detectTable (const PointCloud &cloud, PlaneClusterResult &resp,  Point32 viewpoint_cloud,bool parallel)
    {
      ros::Time ts = ros::Time::now ();
      bool assuming_ar4 = true;
      // Create a downsampled representation of the cloud
      cloud_down_.header = cloud.header;

      // Estimate point normals and copy the relevant data
      try
      {

       if (abs((int)cloud.points.size() - SR_COLS*SR_ROWS) < 30)
       {
         cloud_geometry::nearest::computeOrganizedPointCloudNormalsWithFiltering (cloud_down_, cloud, k_, downsample_factor_, SR_COLS, SR_ROWS, max_z_, min_angle_, max_angle_, viewpoint_cloud);
       }
       else
       {
         assuming_ar4 = false;
         geometry_msgs::PointStamped view;
         view.point.x = viewpoint_cloud.x;
         view.point.y = viewpoint_cloud.y;
         view.point.z = viewpoint_cloud.z;
         cloud_down_.points = cloud.points;
         cloud_geometry::nearest::computePointCloudNormals(cloud_down_, k_,  view);
       }
      }
      catch(std::out_of_range ex)
      {
        throw "Extraction failed";
      }

      // ---[ Select points whose normals are perpendicular to the Z-axis
      vector<int> indices_z;
      if(parallel)
      {
        cloud_geometry::getPointIndicesAxisParallelNormals (cloud_down_, 0, 1, 2, eps_angle_, axis_, indices_z);
      }
      else
      {
        cloud_geometry::getPointIndicesAxisPerpendicularNormals( cloud_down_, 0, 1, 2, eps_angle_, axis_, indices_z);
      }
#ifdef DEBUG
      ROS_INFO ("Number of points with normals parallel to Z: %d.", (int)indices_z.size ());
#endif
      if(indices_z.size () < 10)
      {
       if(parallel)
       {
         cloud_geometry::getPointIndicesAxisParallelNormals (cloud_down_, 0, 1, 2, eps_angle_*2, axis_, indices_z);
       }
       else
       {
         cloud_geometry::getPointIndicesAxisPerpendicularNormals( cloud_down_, 0, 1, 2, eps_angle_*2, axis_, indices_z);
       }
#ifdef DEBUG
       ROS_INFO ("Number of points with normals parallel to Z in second try: %d. with axis (%f %f %f )", (int)indices_z.size (), axis_.x, axis_.y, axis_.z);
#endif
      }
      if(indices_z.size () < 10)
         return false;
      // Find the best plane in this cluster (later, we can optimize and process more clusters individually)
      vector<int> inliers_down;
      vector<double> coeff;
      printf("fitSACPlane \n");
      fitSACPlane (&cloud_down_, indices_z, inliers_down, coeff, viewpoint_cloud, sac_distance_threshold_);

      printf("end fitSACPlane \n");
      // Filter the original pointcloud data with the same min/max angle for jump edges
      PointCloud cloud_filtered;// = cloud;
      if (assuming_ar4)
      {
        cloud_geometry::nearest::filterJumpEdges (cloud, cloud_filtered, 1, SR_COLS, SR_ROWS, min_angle_, max_angle_, viewpoint_cloud);
      }
      else
      {
        cloud_filtered = cloud;
        cloud_filtered.points = cloud.points;
      }
      // Refine plane
      vector<int> inliers (cloud_filtered.points.size ());
      int j = 0;
      for (unsigned int i = 0; i < cloud_filtered.points.size (); i++)
      {
        double dist_to_plane = cloud_geometry::distances::pointToPlaneDistance (cloud_filtered.points[i], coeff);
        if (dist_to_plane < sac_distance_threshold_)
          inliers[j++] = i;
      }
      inliers.resize (j);

      // Obtain the bounding 2D polygon of the table
      Polygon table;
      printf("convexHull2D  \n");
      cloud_geometry::areas::convexHull2D (cloud_down_, inliers_down, coeff, table);
#ifdef DEBUG
      /*PolygonalMap pmap;
      pmap.header = cloud.header;
      pmap.polygons.resize (1);
      pmap.polygons[0] = table;
      pmap_pub_.publish (pmap);*/
#endif

      // Find the object clusters supported by the table
      Point32 min_p, max_p;
      printf("getMinMax \n");
      cloud_geometry::statistics::getMinMax (cloud_filtered, inliers, min_p, max_p);
      vector<int> object_inliers;
      printf("findObjectClusters \n");
      findObjectClusters (cloud_filtered, coeff, table, axis_, min_p, max_p, object_inliers, resp);

#ifdef DEBUG
      // Send the table
      //cloud_clusters_pub_.publish (cloud_annotated_);
      //cloud_geometry::getPointCloud (cloud_filtered, object_inliers, cloud_annotated_);


      // Send the clusters
      //cloud_geometry::getPointCloud (cloud_down_, indices_z, cloud_annotated_);   // downsampled version
      /*cloud_geometry::getPointCloud (cloud_down_, inliers_down, cloud_annotated_);*/   // downsampled version
      //cloud_geometry::getPointCloud (cloud_filtered, inliers, cloud_annotated_);              // full version
      /*cloud_table_pub_.publish (cloud_annotated_);*/
#endif
      ROS_INFO ("Results estimated in %g xseconds.", (ros::Time::now () - ts).toSec ());
      // Copy the plane parameters back in the response
      resp.a = coeff[0]; resp.b = coeff[1]; resp.c = coeff[2]; resp.d = coeff[3];
      cloud_geometry::nearest::computeCentroid (cloud_filtered, inliers, resp.pcenter);

      return true;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      findObjectClusters (const PointCloud &cloud, const vector<double> &coeff, const Polygon &table,
                          const Point32 &axis, const Point32 &min_p, const Point32 &max_p, vector<int> &object_indices, PlaneClusterResult &resp)
    {
      int nr_p = 0;
      Point32 pt;
      object_indices.resize (cloud.points.size ());

      // Iterate over the entire cloud to extract the object clusters
      for (unsigned int i = 0; i < cloud.points.size (); i++)
      {
        // Select all the points in the given bounds - check all axes
        if ( axis.x == 1 && ( cloud.points.at (i).y < min_p.y || cloud.points.at (i).y > max_p.y || cloud.points.at (i).z < min_p.z || cloud.points.at (i).z > max_p.z ) )
          continue;

        else if ( axis.y == 1 && ( cloud.points.at (i).x < min_p.x || cloud.points.at (i).x > max_p.x || cloud.points.at (i).z < min_p.z || cloud.points.at (i).z > max_p.z ) )
          continue;

        else if ( axis.z == 1 && ( cloud.points.at (i).x < min_p.x || cloud.points.at (i).x > max_p.x || cloud.points.at (i).y < min_p.y || cloud.points.at (i).y > max_p.y ) )
          continue;

        // Calculate the distance from the point to the plane
        double dist_to_plane = coeff.at (0) * cloud.points.at (i).x +
                               coeff.at (1) * cloud.points.at (i).y +
                               coeff.at (2) * cloud.points.at (i).z +
                               coeff.at (3) * 1;
        // Calculate the projection of the point on the plane
        pt.x = cloud.points.at (i).x - dist_to_plane * coeff.at (0);
        pt.y = cloud.points.at (i).y - dist_to_plane * coeff.at (1);
        pt.z = cloud.points.at (i).z - dist_to_plane * coeff.at (2);

        if (dist_to_plane > delta_z_ && cloud_geometry::areas::isPointIn2DPolygon (pt, table))
        {
          object_indices[nr_p] = i;
          nr_p++;
        }
      }
      object_indices.resize (nr_p);

      // Find the clusters
      nr_p = 0;
      vector<vector<int> > object_clusters;
      cloud_geometry::nearest::extractEuclideanClusters (cloud, object_indices, object_cluster_tolerance_,
                                                         object_clusters, -1, -1, -1, -1, object_cluster_min_pts_);

#ifdef DEBUG
        int total_nr_pts = 0;
        for (unsigned int i = 0; i < object_clusters.size (); i++)
          total_nr_pts += object_clusters[i].size ();

        cloud_annotated_.header = cloud.header;
        cloud_annotated_.points.resize (total_nr_pts);
        cloud_annotated_.channels.resize (1);
        cloud_annotated_.channels[0].name = "rgb";
        cloud_annotated_.channels[0].values.resize (total_nr_pts);
        ROS_INFO ("Number of clusters found: %d", (int)object_clusters.size ());
#endif

      Point32 min_p_cluster, max_p_cluster;

      resp.oclusters.resize (object_clusters.size ());
      for (unsigned int i = 0; i < object_clusters.size (); i++)
      {
#ifdef DEBUG
        float rgb = getRGB (rand () / (RAND_MAX + 1.0), rand () / (RAND_MAX + 1.0), rand () / (RAND_MAX + 1.0));
#endif
        vector<int> object_idx = object_clusters.at (i);

        // Check whether this object cluster is supported by the table or just flying through thin air
        cloud_geometry::statistics::getMinMax (cloud, object_idx, min_p_cluster, max_p_cluster);
        // Select all the points in the given bounds - check all axes
        if ( axis.x == 1 && ( min_p_cluster.x > max_p.x + object_min_dist_from_table_ ) )
          continue;
        if ( axis.y == 1 && ( min_p_cluster.y > max_p.y + object_min_dist_from_table_ ) )
          continue;
        if ( axis.z == 1 && ( min_p_cluster.z > max_p.z + object_min_dist_from_table_ ) )
          continue;

        // Process this cluster and extract the centroid and the bounds
        for (unsigned int j = 0; j < object_idx.size (); j++)
        {
          object_indices[nr_p] = object_idx.at (j);
#ifdef DEBUG
            cloud_annotated_.points[nr_p] = cloud.points.at (object_idx.at (j));
            cloud_annotated_.channels[0].values[nr_p] = rgb;
#endif
          nr_p++;
        }
        cloud_geometry::nearest::computeCentroid (cloud, object_idx, resp.oclusters[i].center);
        resp.oclusters[i].cov33.clear();
        for(unsigned int l = 0; l < 9; l++)
        {
          resp.oclusters[i].cov33.push_back(0.0);
        }

        for(unsigned int k = 0; k < object_idx.size (); k++)
        {
            Point32 p = cloud.points.at (object_idx.at (k));
            p.x = p.x - resp.oclusters[i].center.x;
            p.y = p.y - resp.oclusters[i].center.y;
            p.z = p.z - resp.oclusters[i].center.z;
            resp.oclusters[i].cov33[0] += p.x*p.x;
            resp.oclusters[i].cov33[1] += p.y*p.x;
            resp.oclusters[i].cov33[2] += p.z*p.x;
            resp.oclusters[i].cov33[3] += p.x*p.y;
            resp.oclusters[i].cov33[4] += p.y*p.y;
            resp.oclusters[i].cov33[5] += p.z*p.y;
            resp.oclusters[i].cov33[6] += p.x*p.z;
            resp.oclusters[i].cov33[7] += p.y*p.z;
            resp.oclusters[i].cov33[8] += p.z*p.z;
        }
        for(unsigned int t = 0; t < resp.oclusters[i].cov33.size(); t++)
        {
          resp.oclusters[i].cov33[t] /= object_idx.size ();
          if(resp.oclusters[i].cov33[t] <= 0.0)
            resp.oclusters[i].cov33[t] = 0.000000001;
        }
        /*cloud_geometry::statistics::getMinMax (cloud, object_idx, resp.oclusters[i].min_bound, resp.oclusters[i].max_bound);*/
      }
      object_indices.resize (nr_p);
#ifdef DEBUG
        cloud_annotated_.points.resize (nr_p);
        cloud_annotated_.channels[0].values.resize (nr_p);
#endif
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool
      fitSACPlane (PointCloud *points, vector<int> &indices, vector<int> &inliers, vector<double> &coeff,
                   const Point32 &viewpoint_cloud, double dist_thresh)
    {
      if ((int)indices.size () < clusters_min_pts_)
      {
        inliers.resize (0);
        coeff.resize (0);
        return (false);
      }

      // Create and initialize the SAC model
      sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
      sample_consensus::SAC *sac             = new sample_consensus::MSAC (model, sac_distance_threshold_);
      sac->setMaxIterations (200);
      sac->setProbability (0.99);
      model->setDataSet (points, indices);

      // Search for the best plane
      if (sac->computeModel ())
      {
        if ((int)sac->getInliers ().size () < clusters_min_pts_)
        {
          //ROS_ERROR ("fitSACPlane: Inliers.size (%d) < sac_min_points_per_model (%d)!", sac->getInliers ().size (), sac_min_points_per_model_);
          inliers.resize (0);
          coeff.resize (0);
          return (false);
        }

        sac->computeCoefficients (coeff);     // Compute the model coefficients
        sac->refineCoefficients (coeff);      // Refine them using least-squares
        model->selectWithinDistance (coeff, dist_thresh, inliers);

        cloud_geometry::angles::flipNormalTowardsViewpoint (coeff, points->points.at (inliers[0]), viewpoint_cloud);

        // Project the inliers onto the model
        model->projectPointsInPlace (inliers, coeff);
      }
      delete model;
      delete sac;
      return (true);
    }
};

PlaneClustersSR* s_planeCluster = NULL;

ClusterDetector::ClusterDetector(int srid, int ptuid) :
  m_swissranger_jlo_id(srid),
  m_ptu_jlo_id(ptuid)
{
  if(s_planeCluster == NULL)
    s_planeCluster = new PlaneClustersSR(NULL);
}

ClusterDetector::ClusterDetector()
{
}

void ClusterDetector::SetData(XMLTag* tag)
{
  if(s_planeCluster == NULL)
    s_planeCluster = new PlaneClustersSR(tag);


  if(tag != NULL)
  {
      m_swissranger_jlo_id = tag->GetPropertyInt(XML_ATTRIBUTE_SR4LO, 0);
      if(m_swissranger_jlo_id == 0)
      {
        std::string name = tag->GetProperty(XML_ATTRIBUTE_SR4LO, "/sr4");
        RelPose* pose = RelPoseFactory::GetRelPose(name);
        if(pose == NULL)
          m_swissranger_jlo_id = 1;
        else
        {
          printf("Read Clusterdetector with %ld as camera position (%s)\n", pose->m_uniqueID, name.c_str());
          m_swissranger_jlo_id =pose->m_uniqueID;
          RelPoseFactory::FreeRelPose(&pose);
        }
      }
      m_ptu_jlo_id = tag->GetPropertyInt(XML_ATTRIBUTE_PTULO, 0);
      if(m_ptu_jlo_id == 0)
      {
        std::string name = tag->GetProperty(XML_ATTRIBUTE_PTULO, "/base_link");
        RelPose* pose = RelPoseFactory::GetRelPose(name);
        if(pose == NULL)
          m_ptu_jlo_id = 1;
        else
        {
          m_ptu_jlo_id =pose->m_uniqueID;
          RelPoseFactory::FreeRelPose(&pose);
        }
      }
  }
}


ClusterDetector::~ClusterDetector()
{
  delete s_planeCluster;
  s_planeCluster = NULL;
}

std::vector<RelPose*> ClusterDetector::Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
  std::vector<RelPose*> results;
    //Calibration* calib = &cam[0]->m_calibration;
  SegmentPrototype* proto = (SegmentPrototype*)object.GetElement(0, DESCRIPTOR_SEGMPROTO);
  printf("ClusterDetector::Perform: Got SegmentPrototype\n");
  for(std::vector<Sensor*>::const_iterator it = sensors.begin(); it != sensors.end(); it++)
  {
    if((*it)->GetName().compare(XML_NODE_SWISSRANGER) == 0)
    {
      try
      {
        results = Inner(*it, proto, numOfObjects, qualityMeasure);
      }
      catch (const char* text )
      {
         printf("Error in ClusterDetector: %s\n", text);
      }
      break;
    }
  }
  /*TODO plane clusters*/
  return results;
}



inline void normalize(double &a,double &b, double &c)
{
  double norm = sqrt(a*a + b*b + c*c);
  a /= norm;
  b /= norm;
  c /= norm;
}

inline double scalarproduct(const double &a,const double &b, const double &c, const double &d, const double &e, const double &f)
{
  return a * d + b* e + c*f;
}

inline void CrossProduct_l(const double b_x, const double b_y,const double b_z,const double c_x,const double c_y,const double c_z,double &a_x,double &a_y,double &a_z)
{
    a_x = b_y*c_z - b_z*c_y;
    a_y = b_z*c_x - b_x*c_z;
    a_z = b_x*c_y - b_y*c_x;
}

bool ClusterDetector::CallStaticPlaneClusterExtractor(Sensor* sensor, PlaneClusterResult* response, int ptu_jlo_id, bool parallel)
{
  if(s_planeCluster == NULL)
    s_planeCluster = new PlaneClustersSR(NULL);
  SwissRangerReading* reading = (SwissRangerReading*)sensor->GetReading(-1);
  try
  {
    bool b =  s_planeCluster->plane_clusters_service(*response, reading->m_relPose->m_uniqueID, ptu_jlo_id, reading->m_image, parallel);
    reading->Free();
    return b;
  }
  catch(const char* text)
  {
    printf("Error in ClusterDetector: %s\n", text);
  }
  return false;
}

std::vector<RelPose*> ClusterDetector::Inner(Sensor* sens, SegmentPrototype* obj_descr, int &numOfObjects, double& qualityMeasure)
{
  std::vector<RelPose*> results;
  PlaneClusterResult response;
  qualityMeasure = 0.0;
  bool parallel = true;
  unsigned long ref_frame = m_ptu_jlo_id;

  if(obj_descr != NULL)
  {
    ref_frame = obj_descr->GetFrameId();
    parallel = obj_descr->m_parallel;
  }

  if(!CallStaticPlaneClusterExtractor(sens, &response, ref_frame, parallel))
  {
    return results;
  }

  double a = response.a;
  double b = response.b;
  double c = response.c;
  double s = response.d;
  std::vector<PlaneClusterResult::ObjectOnTable> &vec = response.oclusters;
  printf("Got plane equation %f x + %f y + %f z + %f = 0\n", a,b,c,s);
   /*Norm v1*/
  normalize(a,b,c);

  /*Init v2*/
  double d,e,f,g,h,i;
  if (a == b && a == c)
  {
     d = 1; e = 0; f = 0;
  }
  else
  {
    d = b; e = a; f = c;
  }
  /*Orthogonalize v2*/
  double tmp = scalarproduct(a,b,c,d,e,f);
  d = d - tmp * a;
  e = e - tmp * b;
  f = f - tmp * c;

  /*Norma v2*/
  normalize(d,e,f);

  /*Create v3*/

  CrossProduct_l(a,b,c,d,e,f, g,h,i);
  /**  Build Matrix:
  *   d g a p.x
  *   e h b p.y
  *   f i c p.z
  *   0 0 0 1
  *   for every cluster
  */
  if(vec.size() == 0)
  {
   /* printf("No Clusters found, adding a meaningless cluster");
        PlaneClusterResult::ObjectOnTable on;
     on.center.x = pcenter.x;
     on.center.y = pcenter.y;
     on.center.z = pcenter.z;

     on.min_bound.x = pcenter.x - 0.5;
     on.min_bound.y = pcenter.y - 0.3;
     on.min_bound.z = pcenter.z - 0.2;

     on.max_bound.x = pcenter.x + 0.5;
     on.max_bound.y = pcenter.y + 0.3;
     on.max_bound.z = pcenter.z + 0.2;
     vec.push_back(on);*/
  }
  printf("Creating a pose for every cluster\n");
  double prob = 1.0;
  RelPose* ptu = RelPoseFactory::FRelPose(ref_frame);

  for(size_t x = 0; x < vec.size(); x++)
  {
    prob = prob * 0.9;
    printf("a: %f , b: %f , c: %f\n", a, b, c);
    const geometry_msgs::Point32 &center = vec[x].center;
    Matrix rotmat(4,4);

    rotmat << d << g << a << center.x
           << e << h << b << center.y
           << f << i << c << center.z
           << 0 << 0 << 0 << 1;

    cout <<  "Matrix from plane_clusters:" << endl << rotmat << endl;
    Matrix cov (6,6);
    if(vec[x].cov33.size() < 9)
    {
      ROS_ERROR("Error detectin Clusters: size of cov33 (%ld) smaller than expected: 9\n", vec[x].cov33.size());
    }
    else
    {
    /*double covz = max(fabs(center.z - max_bound.z), fabs(center.z - min_bound.z ));*/
    /*Fill covariance with the cluster size and hardcoded full rotation in normal direction */
     if(parallel)
     {
/*      cout << sqrt(vec[x].cov33[0]) << sqrt(vec[x].cov33[1]) << sqrt(vec[x].cov33[2]) << 0   << 0   << 0<< endl;
      cout << sqrt(vec[x].cov33[3]) << sqrt(vec[x].cov33[4]) << sqrt(vec[x].cov33[5]) <<  0  << 0   << 0<< endl;
      cout << sqrt(vec[x].cov33[6]) << sqrt(vec[x].cov33[7]) << sqrt(vec[x].cov33[8]) << 0   << 0   << 0<< endl;
      cout <<0    << 0    << 0    <<  ((obj_descr == NULL) ? 0.2 : obj_descr->m_covRotX) << 0   << 0<< endl;
      cout <<0    << 0    << 0    << 0   << ((obj_descr == NULL) ? 0.2 :obj_descr->m_covRotY) << 0<< endl;
      cout <<0    << 0    << 0    << 0   << 0   << ((obj_descr == NULL) ? 0.8 :obj_descr->m_covRotZ)<< endl;*/

      cov << sqrt(vec[x].cov33[0]) << sqrt(vec[x].cov33[1]) << sqrt(vec[x].cov33[2]) << 0   << 0   << 0
         << sqrt(vec[x].cov33[3]) << sqrt(vec[x].cov33[4]) << sqrt(vec[x].cov33[5]) <<  0  << 0   << 0
         << sqrt(vec[x].cov33[6]) << sqrt(vec[x].cov33[7]) << sqrt(vec[x].cov33[8]) << 0   << 0   << 0
         <<0    << 0    << 0    <<  ((obj_descr == NULL) ? 0.2 : obj_descr->m_covRotX) << 0   << 0
         <<0    << 0    << 0    << 0   << ((obj_descr == NULL) ? 0.2 :obj_descr->m_covRotY) << 0
         <<0    << 0    << 0    << 0   << 0   << ((obj_descr == NULL) ? 0.8 :obj_descr->m_covRotZ);
     }
     else
     {
       cov << sqrt(vec[x].cov33[3]) << vec[x].cov33[4] << vec[x].cov33[5] << 0  << 0   << 0
         << vec[x].cov33[0] << vec[x].cov33[1] << vec[x].cov33[2] << 0  << 0   << 0
         << vec[x].cov33[6] << vec[x].cov33[7] << vec[x].cov33[8] << 0   << 0   << 0
         <<0    << 0    << 0    << ((obj_descr == NULL) ? 0.2 :obj_descr->m_covRotY) << 0   << 0
         <<0    << 0    << 0    << 0   << ((obj_descr == NULL) ? 0.2 :obj_descr->m_covRotX) << 0
         <<0    << 0    << 0    << 0   << 0   << ((obj_descr == NULL) ? 0.8 :obj_descr->m_covRotZ);
      }

      cout << "Cov from pc: "<< endl <<  cov << endl;
      RelPose* pose_temp = RelPoseFactory::FRelPose(ptu, rotmat, cov);
      double temp_qual = min(1.0, max(0.0, fabs((sqrt(vec[x].cov33[4])*sqrt(vec[x].cov33[0])*sqrt(vec[x].cov33[8]))) * 500 ));
      if(x == 0)
       qualityMeasure =temp_qual;

      if(pose_temp == NULL)
        continue;
      pose_temp->m_qualityMeasure = temp_qual;
      results.push_back(pose_temp);
    }
  }
  RelPoseFactory::FreeRelPose(&ptu);
  return results;
}

double ClusterDetector::CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors)
{
  for(std::vector<Sensor*>::const_iterator it = sensors.begin(); it != sensors.end(); it++)
  {
    if((*it)->GetName().compare(XML_NODE_SWISSRANGER) == 0)
    {
      if(object.GetElement(0, DESCRIPTOR_SEGMPROTO) != NULL )
        return 0.1;
      else
        return 0.0;
    }
  }
  return 0.0;
}


bool ClusterDetector::TrackingPossible(const Reading& img, const Signature& sig, RelPose* pose)
{
    return false;
}

XMLTag* ClusterDetector::Save()
{
    XMLTag* tag = new XMLTag(XML_NODE_CLUSTERDETECTOR);
    tag->AddProperty(XML_ATTRIBUTE_SR4LO, m_swissranger_jlo_id);
    tag->AddProperty(XML_ATTRIBUTE_PTULO, m_ptu_jlo_id);
    return tag;
}

