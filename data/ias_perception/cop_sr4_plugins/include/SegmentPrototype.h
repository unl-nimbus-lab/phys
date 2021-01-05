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


#ifndef BLOB_H
#define BLOB_H

#include "Descriptor.h"
#include <sensor_msgs/PointCloud.h>

#define XML_NODE_SEGMENTPROTOTYPE "SegmentPrototype"

namespace cop
{

  sensor_msgs::PointCloud cloud_trans (LocatedObjectID_t swissranger_jlo_id, LocatedObjectID_t ptu_base_jlo_id,  const sensor_msgs::PointCloud& cloud_in);

  class SegmentPrototype :
    public Descriptor
  {
  public:
    SegmentPrototype();
    SegmentPrototype(std::string sensor_frame, const sensor_msgs::PointCloud& pcd, std::string classname, ObjectID_t id, ObjectID_t class_id);


    void SaveTo(XMLTag* tag);
    virtual void Show(RelPose* pose, Sensor* cam);

    virtual std::string GetNodeName() const{return XML_NODE_SEGMENTPROTOTYPE;}
    virtual ElemType_t GetType() const{return DESCRIPTOR_SEGMPROTO;}

    LocatedObjectID_t GetFrameId();
    LocatedObjectID_t GetSensorFrameId(const LocatedObjectID_t &id);
    void UpdateRefFrame();

    /** Segment prototype is a special object that holds also most of the collsion info
      the following function are to provide this capabilities to the cop_collision_interface */

    /***
    *   SetTable
    *   @brief stores the table the cluster was detected on
    *   @param id   a lo id where te table is in space and how is the oriantation
    *   @param pcd  a point cloud containing all points that were classified as table points
    */
    void SetTable(const LocatedObjectID_t &id, const sensor_msgs::PointCloud &pcd){m_LastTableID = id; m_mapPCD[id] = pcd;}

    /***
    *   GetTable
    *   @brief Get the current table
    *   @return the lo aid and a point cloudwith table points from the last detection
    */
    std::pair<LocatedObjectID_t, sensor_msgs::PointCloud> GetTable()
    {
      std::pair<LocatedObjectID_t, sensor_msgs::PointCloud> pair;
      pair.first = m_LastTableID;
      pair.second = m_mapPCD[m_LastTableID];
      return pair;}


    /***
    *   SetPointCloud
    *   @brief stores the clusters that was detected, this list is cleared on each search for clusters,
              so on all detected clusters it will contain all other clusters detected at the same time
    *   @param id   a lo id where the cluster is in space and how is the oriantation
    *   @param pcd  a point cloud containing all points that were classified as cluster points
    */
    void SetPointCloud(const LocatedObjectID_t &id, const sensor_msgs::PointCloud &pcd, const LocatedObjectID_t &sensor_id);
    /**
    *   GetPointCloud
    *   @param id   the location this cluster is located to identify which is right point cloud (retireve this from the signature)
    */
    sensor_msgs::PointCloud GetPointCloud(LocatedObjectID_t id){if(m_mapPCD.find(id) != m_mapPCD.end())return m_mapPCD[id]; else return sensor_msgs::PointCloud();}

    /**
     *  @brief Clear the list for a new detection tun
    */
    void ClearPointClouds(){m_mapPCD.clear();}
    /**
    *Get approximated shape
    */
    virtual bool GetShape(GeometricShape &objectShape) const;


    virtual Elem* Duplicate(bool bStaticCopy);
    virtual void PropagatePose(RelPose* pose);
  protected:
    virtual void SetData(XMLTag* tag);
  public:
    ~SegmentPrototype(void);

  private:
    std::map<LocatedObjectID_t, sensor_msgs::PointCloud> m_mapPCD;
    LocatedObjectID_t  m_LastTableID;

    std::string m_relFrame;
    LocatedObjectID_t  m_frameID;
    RelPose* m_relPoseOfRefFrame;
    
    std::map<LocatedObjectID_t, LocatedObjectID_t>  m_sensorFrameID;
  public:
    double m_covRotX;
    double m_covRotY;
    double m_covRotZ;
    bool   m_parallel;
  };
}
#endif /*BLOB_H*/
