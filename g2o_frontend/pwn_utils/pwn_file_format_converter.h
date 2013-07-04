#ifndef _PWN_FILE_FORMAT_CONVERTER_H_
#define _PWN_FILE_FORMAT_CONVERTER_H_

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/basemath/bm_se3.h"

using namespace std;
using namespace Eigen;
using namespace pwn;

bool pwnToPcd(const char *pcdFile, const char *pwnFile) {
  // Load points from the .pwn file
  Frame frame;
  Isometry3f transform;
  if(!frame.load(transform, pwnFile)) {
    cerr << "Couldn't read file " << pwnFile << endl;
    return false;
  }
  transform.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

  // Fill the pcl cloud
  pcl::PointCloud<pcl::PointNormal> cloud;
  cloud.width = frame.points().size();
  cloud.height = 1;
  cloud.sensor_origin_ = Vector4f(transform.translation().x(),
				  transform.translation().y(),
				  transform.translation().z(),
				  1.0f);
  cloud.sensor_orientation_ = Quaternionf(transform.linear());  
  cloud.is_dense = true;
  cloud.points.resize(frame.points().size());
  for(size_t i = 0; i < cloud.points.size (); ++i) {
    const Point &p = frame.points()[i];
    const Normal &n = frame.normals()[i];
    pcl::PointNormal &pointNormal = cloud.points[i];

    // Fill the current PointNormal
    pointNormal.x = p.x();
    pointNormal.y = p.y();
    pointNormal.z = p.z();
    pointNormal.normal_x = n.x();
    pointNormal.normal_y = n.y();
    pointNormal.normal_z = n.z();
    pointNormal.curvature = 0.0f;
  }

  // Save the cloud in the .pcd file
  pcl::io::savePCDFileASCII(pcdFile, cloud);
  cerr << "Saved " << cloud.points.size () << " data points to "<< pcdFile << endl;

  return true;
}

bool pcdToPwn(const char *pwnFile, const char *pcdFile) {
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);

  // Read the .pcd file
  if(pcl::io::loadPCDFile<pcl::PointNormal>(pcdFile, *cloud) == -1) {
    cerr << "Couldn't read file " << pcdFile << endl;
    return false;
  }
  
  // Get the transform
  Isometry3f transform;
  transform.translation() = cloud->sensor_origin_.head<3>();
  transform.linear() = cloud->sensor_orientation_.toRotationMatrix();
  transform.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

  // Fill the frame
  Frame frame;
  frame.points().resize(cloud->points.size());
  frame.normals().resize(cloud->points.size());

  for(size_t i = 0; i < frame.points().size(); ++i) {
    const pcl::PointNormal &pointNormal = cloud->points[i];
    Point &p = frame.points()[i];
    Normal &n = frame.normals()[i];

    // Fill the current Point and Normal
    p.x() = pointNormal.x;
    p.y() = pointNormal.y;
    p.z() = pointNormal.z;
    n.x() = pointNormal.normal_x;
    n.y() = pointNormal.normal_y;
    n.z() = pointNormal.normal_z;
  }

  // Save the frame in the .pwn file
  frame.save(pwnFile, 1, true, transform);
  cerr << "Saved " << frame.points().size () << " data points to "<< pwnFile << endl;

  return true;
}

#endif
