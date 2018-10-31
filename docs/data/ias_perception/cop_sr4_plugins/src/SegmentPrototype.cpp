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


#include "SegmentPrototype.h"
#include "XMLTag.h"


#define XML_ATTRIBUTE_RELFRAME   "RelFrame"
#define XML_ATTRIBUTE_PARALLEL "ParallelSearch"
#define XML_ATTRIBUTE_COVX       "CovX"
#define XML_ATTRIBUTE_COVY       "CovY"
#define XML_ATTRIBUTE_COVZ       "CovZ"

using namespace cop;


SegmentPrototype::SegmentPrototype() :
  m_relFrame("/base_link"),
  m_relPoseOfRefFrame(NULL),
  m_covRotX(0.2),
  m_covRotY(0.2),
  m_covRotZ(0.8),
  m_parallel(true)

{
}





SegmentPrototype::SegmentPrototype(std::string sensor_frame, const sensor_msgs::PointCloud& pcd, std::string classname, ObjectID_t id, ObjectID_t class_id) :
    Descriptor(new Class(classname, class_id)),
    m_relFrame("/base_link"),
    m_relPoseOfRefFrame(NULL),
    m_covRotX(0.2),
    m_covRotY(0.2),
    m_covRotZ(0.8),
    m_parallel(true)
{
  double meanx = 0.0, meany = 0.0,  meanz = 0.0;
  double  cov[9];
  memset(cov, 0, sizeof(*cov)*9);

  m_ID = id;
  UpdateRefFrame();
  RelPose* p_sensor_frame = RelPoseFactory::GetRelPose(sensor_frame);
  if(p_sensor_frame == NULL)
    throw "Sensor frame is invalid";
  for(size_t i = 0; i < pcd.points.size(); i++)
  {
    meanx += pcd.points[i].x;
    meany += pcd.points[i].y;
    meanz += pcd.points[i].z;

    for(int j = 0; j < 3; j++)
    {
      for(int k = 0; k < 3; k++)
      {
        double tmp = 0.0;
        switch(j)
        {
          case 0:
            tmp = pcd.points[i].x;
            break;
          case 1:
            tmp = pcd.points[i].y;
            break;
          case 2:
            tmp = pcd.points[i].z;
            break;
        }
        switch(k)
        {
          case 0:
            tmp *= pcd.points[i].x;
            break;
          case 1:
            tmp *= pcd.points[i].y;
            break;
          case 2:
            tmp *= pcd.points[i].z;
            break;
        }
        cov[j*3+k] += tmp;
      }
    }
  }
  Matrix m = IdentityMatrix(4);
  Matrix covm(6,6);
  for(int j = 0; j < 6; j++)
  {
     for(int k = 0; k < 6; k++)
     {
       covm.element(j,k) = 0.0;
     }
  }

  m.element(0,3) = meanx / pcd.points.size();
  m.element(1,3) = meany / pcd.points.size();
  m.element(2,3) = meanz / pcd.points.size();
  covm.element(0,0) = cov[0] / pcd.points.size();
  covm.element(0,1) = cov[1] / pcd.points.size();
  covm.element(0,2) = cov[2] / pcd.points.size();
  covm.element(1,0) = cov[3] / pcd.points.size();
  covm.element(1,1) = cov[4] / pcd.points.size();
  covm.element(1,2) = cov[5] / pcd.points.size();
  covm.element(2,0) = cov[6] / pcd.points.size();
  covm.element(2,1) = cov[7] / pcd.points.size();
  covm.element(2,2) = cov[8] / pcd.points.size();

  RelPose *pose = RelPoseFactory::FRelPose(ID_WORLD, m, covm);

  SetLastMatchedImage(NULL, pose);

  SetPointCloud(pose->m_uniqueID, pcd, p_sensor_frame->m_uniqueID);

}



void SegmentPrototype::SetData(XMLTag* tag)
{
  Descriptor::SetData(tag);
  m_relFrame =   tag->GetProperty(XML_ATTRIBUTE_RELFRAME, m_relFrame);
  m_parallel =   tag->GetPropertyInt(XML_ATTRIBUTE_PARALLEL, 0) == 0;
  UpdateRefFrame();
  m_covRotX =    tag->GetPropertyDouble(XML_ATTRIBUTE_COVX, m_covRotX);
  m_covRotY =    tag->GetPropertyDouble(XML_ATTRIBUTE_COVY, m_covRotY);
  m_covRotZ =    tag->GetPropertyDouble(XML_ATTRIBUTE_COVZ, m_covRotZ);

  RelPose* pose = GetLastMatchedPose();
  std::string filename = tag->GetProperty(XML_ATTRIBUTE_FILENAME, "");

  if(filename.length() > 0)
  {
    sensor_msgs::PointCloud pcd;
    FILE* file = fopen(filename.c_str(), "r");
    if (file==NULL)
    {
      ROS_ERROR("Could not read message from file (filename: %s in Class %s)\n", filename.c_str(), GetNodeName().c_str());
      throw("Error reading class");
    }

    // obtain file size:
    fseek (file, 0 , SEEK_END);
    long lSize = ftell (file);
    rewind (file);
    uint8_t*  buffer = new uint8_t[lSize];
    if (buffer == NULL)
    {
      ROS_ERROR("Could not read message from file (filename: %s in Class %s)\n", filename.c_str(), GetNodeName().c_str());
      throw("Error reading class");
    }

    // copy the file into the buffer:
    long result = fread (buffer,1,lSize,file);
    if (result != lSize)
    {
      ROS_ERROR("Could not read message from file (filename: %s in Class %s)\n", filename.c_str(), GetNodeName().c_str());
      delete buffer;
      throw("Error reading class");
    }
    pcd.deserialize(buffer);

    if(pose == NULL)
    {
      Matrix m = IdentityMatrix(4);
      Matrix cov = IdentityMatrix(6);
      double  cov33[9];
      for(unsigned int t = 0; t < 9; t++)
        cov33[t] = 0;
       for(unsigned int k = 0; k < pcd.points.size (); k++)
       {
         geometry_msgs::Point32 p = pcd.points[k];
         cov33[0] += p.x*p.x;
         cov33[1] += p.y*p.x;
         cov33[2] += p.z*p.x;
         cov33[3] += p.x*p.y;
         cov33[4] += p.y*p.y;
         cov33[5] += p.z*p.y;
         cov33[6] += p.x*p.z;
         cov33[7] += p.y*p.z;
         cov33[8] += p.z*p.z;
       }
       for(unsigned int t = 0; t < 9; t++)
       {
         cov.element(t/3, (t%3)) =   cov33[t] / pcd.points.size ();
         if(cov33[t] <= 0.0)
            cov33[t] = 0.000000001;
       }
      cov.element(3,3) = m_covRotX;
      cov.element(4,4) = m_covRotY;
      cov.element(5,5) = m_covRotZ;

      pose = RelPoseFactory::FRelPose(ID_WORLD, m, cov);
      SetLastMatchedImage(NULL, pose);
    }

    m_mapPCD[pose->m_uniqueID] = pcd;
    fclose (file);
    delete buffer;
  }
}

LocatedObjectID_t SegmentPrototype::GetFrameId()
{
  UpdateRefFrame();
  return m_frameID;
}

 LocatedObjectID_t SegmentPrototype::GetSensorFrameId(const LocatedObjectID_t &id)
 {
   if(m_sensorFrameID.find(id) != m_sensorFrameID.end())
     return m_sensorFrameID[id];
   else
  {
    printf("No entry for %ld in SegmentPrototype::GetSensorFrameId\n", id);
     return ID_WORLD;
   }
 }

void SegmentPrototype::UpdateRefFrame()
{
  RelPose* temp = m_relPoseOfRefFrame;
  m_relPoseOfRefFrame  = RelPoseFactory::GetRelPose(m_relFrame);
  if(temp  != NULL)
  {
    RelPoseFactory::FreeRelPose(&temp );
  }
  m_frameID = m_relPoseOfRefFrame->m_uniqueID;
}

SegmentPrototype::~SegmentPrototype(void)
{
  RelPoseFactory::FreeRelPose(&m_relPoseOfRefFrame);
}


void SegmentPrototype::SaveTo(XMLTag* tag)
{
  Descriptor::SaveTo(tag);

  if(GetLastMatchedPose() != NULL)
  {
    const RelPose* pose2 = GetLastMatchedPose();
    std::map<LocatedObjectID_t, sensor_msgs::PointCloud>::const_iterator test = m_mapPCD.find(pose2->m_uniqueID);
    if(test != m_mapPCD.end())
    {
      std::ostringstream os;
      os << "pcd_"  << m_ID << ".pcd";
      std::string filename = os.str();
      uint32_t length = (*test).second.serializationLength();
      uint8_t* write_pointer = new uint8_t[length];
      uint8_t seq = 0, *outp;
      outp = (*test).second.serialize(write_pointer, seq);
      FILE* file = fopen(filename.c_str(), "w");
      if(file != NULL)
      {
        fwrite(write_pointer, 1, length, file);
        tag->AddProperty(XML_ATTRIBUTE_FILENAME, filename);
        fclose(file);
      }
      else
      {
        ROS_ERROR("Could not write message to file (filename: %s in Class %s)\n", filename.c_str(), GetNodeName().c_str());
      }
      delete write_pointer;
      tag->AddProperty(XML_ATTRIBUTE_FILENAME, os.str());
     }
  }
  tag->AddProperty(XML_ATTRIBUTE_RELFRAME, m_relFrame);
  tag->AddProperty(XML_ATTRIBUTE_PARALLEL, m_parallel);
  tag->AddProperty(XML_ATTRIBUTE_COVX, m_covRotX);
  tag->AddProperty(XML_ATTRIBUTE_COVY, m_covRotY);
  tag->AddProperty(XML_ATTRIBUTE_COVZ, m_covRotZ);
}


void SegmentPrototype::Show(RelPose* pose, Sensor* camin)
{
  printf("in SegmentPrototype::Show(RelPose* pose, Sensor* camin)\n");
  if(camin != NULL && pose != NULL && camin->GetRelPose() != NULL)
  {
    Matrix m = pose->GetMatrix(camin->GetRelPose()->m_uniqueID);
    printf("Pose %ld in %ld\n", pose->m_uniqueID, camin->GetRelPose()->m_uniqueID);
    cout << m;
    double row, column;
    camin->ProjectPoint3DToSensor(m.element(0, 3), m.element(1,3), m.element(2,3), row, column);

    printf("projected to %f, %f\n", row, column);
  }
  if(camin != NULL && pose != NULL)
  {
    RelPose* pose2;
    LocatedObjectID_t id = pose->m_uniqueID;
    LocatedObjectID_t parent = pose->m_parentID;
    if(GetLastMatchedPose() != NULL && GetLastMatchedPose()->m_uniqueID != id)
    {
      id = GetLastMatchedPose()->m_uniqueID;
      Matrix m = pose->GetMatrix(id);
      Matrix cov = pose->GetCovariance(id);
      pose2 = RelPoseFactory::FRelPose(GetLastMatchedPose()->m_parentID, m , cov);
      parent = pose2->m_uniqueID;
    }
    if(m_mapPCD.find(id) != m_mapPCD.end())
    {
     std::vector<double> x,y,z;
     sensor_msgs::PointCloud pcd = cloud_trans(pose->m_uniqueID, ID_WORLD, m_mapPCD[id]);
     for (size_t index = 0; index < pcd.points.size(); index++)
     {
        x.push_back(pcd.points[index].x);
        y.push_back(pcd.points[index].y);
        z.push_back(pcd.points[index].z);
      }
      camin->Publish3DData(x,y,z);
    }
    if(GetLastMatchedPose() != NULL && GetLastMatchedPose()->m_uniqueID != id)
    {
      RelPoseFactory::FreeRelPose(&pose2);
    }

  }
}

Elem* SegmentPrototype::Duplicate(bool bStaticCopy)
{
  SegmentPrototype* new_obj = (SegmentPrototype*)Descriptor::Duplicate(bStaticCopy);
  /** Assign SegmentPrototype Members*/
  new_obj->m_mapPCD = m_mapPCD;
  new_obj->m_sensorFrameID = m_sensorFrameID;
  /** Assign Descriptor Memebers*/
  new_obj->m_class = m_class;
  new_obj->m_imgLastMatchReading = m_imgLastMatchReading;
  new_obj->m_poseLastMatchReading = m_poseLastMatchReading;
  new_obj->m_qualityMeasure = m_qualityMeasure;
  return new_obj;
}

void SegmentPrototype::PropagatePose(RelPose* pose)
{
  if(m_mapPCD.find(pose->m_uniqueID) != m_mapPCD.end())
  {
    m_poseLastMatchReading = pose;
  }
}

bool SegmentPrototype::GetShape(GeometricShape &objectShape) const
{
  if(objectShape.type > 1)
  {
    printf("Shape already set\n");
    return false;
  }
  if(m_poseLastMatchReading == NULL)
  {
    printf("No pose\n");
  }
  objectShape.type = 4;/*undefined PCD;*/
  /*Matrix cov = m_poseLastMatchReading->GetCovarianceMatrix();*/
  LocatedObjectID_t id;
  LocatedObjectID_t parent;
  if(GetLastMatchedPose() != NULL)
  {
    const RelPose* pose2 = GetLastMatchedPose();
    id = pose2->m_uniqueID;
    parent = pose2->m_parentID;
    std::map<LocatedObjectID_t, sensor_msgs::PointCloud>::const_iterator test = m_mapPCD.find(id);
    if(test != m_mapPCD.end())
    {
      std::vector<double> x,y,z;

      sensor_msgs::PointCloud pcd = cloud_trans(id, ID_WORLD, (*test).second);
      for(size_t i = 0;i < pcd.points.size(); i++)
      {
        cop::PointShape p;
        p.x = pcd.points[i].x;
        p.y = pcd.points[i].y;
        p.z = pcd.points[i].z;
        objectShape.vertices.push_back(p);
      }
    }
  }
  return true;
}



sensor_msgs::PointCloud cop::cloud_trans (LocatedObjectID_t swissranger_jlo_id, LocatedObjectID_t ptu_base_jlo_id,  const sensor_msgs::PointCloud& cloud_in)
{
   RelPose* pose = RelPoseFactory::GetRelPose(swissranger_jlo_id, ptu_base_jlo_id);
   sensor_msgs::PointCloud cloud_in_trans;
   if(pose == NULL)
   {
     printf("Strange Fallback solution\n");
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
   cout << "Matrix in cloud_trans: ("<< pose->m_uniqueID << " -> "<< pose->m_parentID << " )  (asked"<< swissranger_jlo_id << "-> "<< ptu_base_jlo_id << ")\n" << m << "\n";
   RelPoseFactory::FreeRelPose(&pose);

   Matrix m_tmp = m;
   cloud_in_trans.points.clear();
   if(cloud_in.points.size() == 0 || cloud_in.points.size() > 1000000000)
      throw "Error in pointcloud, failed check 0 =< num_points < 1000000000";
   for(size_t i = 0; i < cloud_in.points.size(); i++)
   {
     ColumnVector v(4);
     v <<  cloud_in.points[i].x <<  cloud_in.points[i].y <<  cloud_in.points[i].z << 1;
     ColumnVector a = m_tmp*v;
     geometry_msgs::Point32 pt;
     pt.x = a.element(0);
     pt.y = a.element(1);
     pt.z = a.element(2);
     cloud_in_trans.points.push_back(pt);
   }
   cloud_in_trans.channels.clear();
   for(size_t channels = 0 ; channels < cloud_in.channels.size(); channels++)
   {

     cloud_in_trans.channels.push_back(sensor_msgs::ChannelFloat32());
     cloud_in_trans.channels[channels].name = cloud_in.channels[channels].name;
     for(size_t points = 0; points < cloud_in.channels[channels].values.size(); points++)
     {
       cloud_in_trans.channels[channels].values.push_back(
       cloud_in.channels[channels].values[points]);
     }
   }
   return cloud_in_trans;
}


void SegmentPrototype::SetPointCloud(const LocatedObjectID_t &id, const sensor_msgs::PointCloud &cloud_in, const LocatedObjectID_t &sensor_id)
{
  sensor_msgs::PointCloud cloud_in_trans;
  RelPose* pose = RelPoseFactory::FRelPose(id);
  Matrix m_in = pose->GetMatrix(0);
  Matrix m = m_in.i();

  for(size_t i = 0; i < cloud_in.points.size(); i++)
  {
    ColumnVector v(4);
    v <<  cloud_in.points[i].x <<  cloud_in.points[i].y <<  cloud_in.points[i].z << 1;
    ColumnVector a = m*v;
    geometry_msgs::Point32 pt;
    pt.x = a.element(0);
    pt.y = a.element(1);
    pt.z = a.element(2);
    cloud_in_trans.points.push_back(pt);
  }
  m_mapPCD[id] = cloud_in_trans;
  m_sensorFrameID[id] = sensor_id;
  RelPoseFactory::FreeRelPose(&pose);
}


