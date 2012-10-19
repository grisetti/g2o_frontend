/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id: registration_visualizer.cpp 2011-07-24 01:06:01Z georgeLisca $
 *
 */

// PCL
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/registration/icp.h"
#include "pcl/registration/icp_nl.h"
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include "pcl/visualization/registration_visualizer.h"
#include <iostream>

using namespace std;

void m2tq(Eigen::Vector3f& t, Eigen::Quaternionf& q, const Eigen::Matrix4f& m){
  t.x()=m(0,3);
  t.y()=m(1,3);
  t.z()=m(2,3);
  Eigen::Matrix3f R=m.block<3,3>(0,0);
  q=Eigen::Quaternionf(R);
}

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, Eigen::Matrix4f &final_transform, bool downsample = false)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  double leafSize=0.01;
  if (downsample)
  {
    grid.setLeafSize (leafSize, leafSize, leafSize);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }


  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);


  //
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  //pcl::IterativeClosestPoint<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.1);  

  reg.setInputCloud (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);



  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (2);
  for (int i = 0; i < 30; ++i)
    {
      PCL_INFO ("Iteration Nr. %d.\n", i);
      
      // save cloud for visualization purpose
      points_with_normals_src = reg_result;
      
      // Estimate
      reg.setInputCloud (points_with_normals_src);
      reg.align (*reg_result);
      
      //accumulate transformation between each Iteration
      Ti = reg.getFinalTransformation () * Ti;
      
      //if the difference between this transformation and the previous one
	//is smaller than the threshold, refine the process by reducing
	//the maximal correspondence distance
      if (0 && fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
	reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
      
      prev = reg.getLastIncrementalTransformation ();
      
    }
  
  final_transform = Ti;
 }

int
main (int argc, char** argv)
{

  ofstream os("out.g2o");
//  /////////////////////////////////////////////////////////////////////////////////////////////////////

  int n;
  
  
  if (argc != 2)
  {
    std::cerr << "please specify the number of bloody clouds o be registered " << std::endl;
    exit (0);
  }
  n=atoi(argv[1]);
  
  std::cerr << "attempting to read " <<  n << " bloody clouds" << std::endl;
//  /////////////////////////////////////////////////////////////////////////////////////////////////////

//  /////////////////////////////////////////////////////////////////////////////////////////////////////
  
  pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
  //  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.setLeafSize (0.05, 0.05, 0.05);
  //  sor.setLeafSize (0.05, 0.05, 0.05);
//  sor.setLeafSize (0.1, 0.1, 0.1);
//  sor.setLeafSize (0.4, 0.4, 0.4);
//  sor.setLeafSize (0.5, 0.5, 0.5);

  // sor.setInputCloud (inputCloud.makeShared());
  // std::cout<<"\n inputCloud.size()="<<inputCloud.size()<<std::endl;
  // sor.filter (inputCloudFiltered);
  // std::cout<<"\n inputCloudFiltered.size()="<<inputCloudFiltered.size()<<std::endl;

  // sor.setInputCloud (targetCloud.makeShared());
  // std::cout<<"\n targetCloud.size()="<<targetCloud.size()<<std::endl;
  // sor.filter (targetCloudFiltered);
  // std::cout<<"\n targetCloudFiltered.size()="<<targetCloudFiltered.size()<<std::endl;


//   pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

//   icp.setMaximumIterations(10000);

// //  icp.setMaxCorrespondenceDistance (0.6);
//   icp.setMaxCorrespondenceDistance (0.8);
// //  icp.setMaxCorrespondenceDistance (1.5);

// //  icp.setRANSACOutlierRejectionThreshold (0.1);
//   icp.setRANSACOutlierRejectionThreshold (0.6);
// //  icp.setRANSACOutlierRejectionThreshold (1.5);
// //  icp.setRANSACOutlierRejectionThreshold (5.0);


  std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > clouds(n);
  std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > filteredClouds(n);

  Eigen::Matrix4f tTotal=Eigen::Matrix4f::Identity();


  const float min_scale = 0.0005; 
  const int nr_octaves = 4; 
  const int nr_scales_per_octave = 5; 
  const float min_contrast = 1; 
  
  //pcl::SIFTKeypoint<pcl::PointXYZRGBA, pcl::PointWithScale> sift;

  for (int i=0; i<n; i++){
    char filename[20];
    char ffilename[20];
    sprintf(filename,"base-%03d.pcd", i);
    sprintf(ffilename,"filt-%d.pcd", i);
    cerr << "filename= " << filename <<endl;
    clouds[i]=pcl::PointCloud<pcl::PointXYZRGBA>();
    cerr << "ok up to here" << endl;
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (filename, clouds[i]) == -1) //* load the file
      {
	std::cerr << "Couldn't read the "<< i <<"th pcd file \n" << std::endl;
	return (-1);
      }

    
    // {
    //   pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA> ());;//new API
    //   pcl::PointCloud<pcl::PointWithScale>::Ptr sifts (new pcl::PointCloud<pcl::PointWithScale>);
    //   sift.setInputCloud(clouds[i]);
    //   sift.setSearchMethod (tree);
    //   sift.setScales(min_scale, nr_octaves, nr_scales_per_octave);
    //   sift.setMinimumContrast(min_contrast);
    //   sift.compute (*sifts);
    
    //   cerr << "Computed " << sifts->points.size () << " SIFT Keypoints " << endl;
    // }

    cerr << "sor input" << endl;
    sor.setInputCloud (clouds[i].makeShared());
    std::cout<<"\n inputCloud.size()="<<clouds[i].size()<<std::endl;

    cerr << "sor target" << endl;
    filteredClouds[i]=pcl::PointCloud<pcl::PointXYZRGBA>();

    cerr << "sor filter" << endl;
    sor.filter (filteredClouds[i]);

    std::cout<<"\n inputCloudFiltered.size()="<<filteredClouds[i].size()<<std::endl;
    pcl::io::savePCDFile(ffilename, filteredClouds[i]);

    cerr << "compounding" << endl;
    Eigen::Matrix4f transform =Eigen::Matrix4f::Identity();
    if (i==0) {
      tTotal=Eigen::Matrix4f::Identity();
    } else {
      const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source = filteredClouds[i].makeShared();
      const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target = filteredClouds[i-1].makeShared();
      
      Eigen::Matrix4f transform;
      pairAlign (source, target, transform, true);
      tTotal = tTotal * transform;
    }
    Eigen::Vector3f t;
    Eigen::Quaternionf q;
    m2tq(t,q,tTotal);
    os << "VERTEX_SE3:QUAT " << i << " "
       << t.x() << " " << t.y() << " " << t.z() << " "
       << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
    os << "DATA_PCL 0 " << ffilename << " " << i << " 0" << endl;

    if (i>0){
	Eigen::Vector3f t;
	Eigen::Quaternionf q;
	m2tq(t,q,transform);
	os << "EDGE_SE3:QUAT " << i-1 << " " << i << " "
	   << t.x() << " " << t.y() << " " << t.z() << " "
	   << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " ";
	Eigen::Matrix<float, 6, 6> m= Eigen::Matrix<float, 6, 6>::Identity();
	for (int i=0; i<m.rows(); i++)
	  for (int j=i; j<m.cols(); j++)
	    os << m(i,j) << " ";
	os << endl;
    }
  }
}
