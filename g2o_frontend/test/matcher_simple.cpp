#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include <stdio.h>
#include <set>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"

using namespace cv;
using namespace std;

void  makePointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
		     Mat image, Mat depth, float imageFocalLength){
  unsigned char* iptr=reinterpret_cast<unsigned char*>(image.data);
  unsigned short* dptr=reinterpret_cast<unsigned short*>(depth.data);
  assert(image.rows==depth.rows && image.cols==depth.cols);
  int w=image.cols;
  int h=image.rows;

  float f=imageFocalLength; 
  cloud->height = h;
  cloud->width = w;
  cloud->is_dense = false;
  cloud->points.resize (h*w);
  register float constant = 1.0f / f;
  int v=-h/2;
  int k=0;
  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  for (int i=0; i<image.rows; i++, v++){
    int u=-w/2;
    for (int j=0; j<image.cols; j++, u++){
      pcl::PointXYZI& pt=cloud->points[k];
      unsigned short d=*dptr;
      if (d==0){
	 pt.x = pt.y = pt.z = bad_point;
	 pt.intensity=0;
      } else {
	pt.z = d*1e-3f;
	pt.x = u * pt.z * constant;
	pt.y = v * pt.z * constant;
	pt.intensity = (float)(*iptr);
      }
      iptr++;
      dptr++;
      k++;
    }
  }
}

struct IntensityDepthImage{
  IntensityDepthImage(cv::Mat& intensity_, cv::Mat& depth_, float baseline_, float focalLength_) {
    intensity=intensity_;
    depth=depth_;
    assert(intensity.rows==depth.rows &&intensity.cols == depth.cols);
    baseline=baseline_;
    focalLength=focalLength_;
    computeXYZ();
  }

  void computeXYZ(){
    float n=std::numeric_limits<float>::quiet_NaN();
    xyzi=cv::Mat(intensity.rows, intensity.cols, CV_32FC4,Scalar(n,n,n,0));

    unsigned char* iptr=reinterpret_cast<unsigned char*>(intensity.data);
    unsigned short* dptr=reinterpret_cast<unsigned short*>(depth.data);
    float* xyziptr=reinterpret_cast<float*>(xyzi.data);
    int w=intensity.cols;
    int h=intensity.rows;

    float f=focalLength; 
    register float constant = 1.0f / f;
    int v=-h/2;
    float bad_point = std::numeric_limits<float>::quiet_NaN ();

    for (int i=0; i<intensity.rows; i++, v++){
      int u=-w/2;
      for (int j=0; j<intensity.cols; j++, u++){
	float& x=*xyziptr;
	float& y=*(xyziptr+1);
	float& z=*(xyziptr+2);
	float& intns=*(xyziptr+3);

	unsigned short d=*dptr;
	if (d==0){
	  x=y=z=bad_point;
	  intns=0;
	} else {
	  z = d*1e-3f;
	  x = u * z * constant;
	  y = v * z * constant;
	  intns= (float)(*iptr);
	}
	iptr++;
	dptr++;
	xyziptr+=4;
      }
    }
  }

  void  computePointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud){
    int w=xyzi.cols;
    int h=xyzi.rows;
    cloud->height = h;
    cloud->width = w;
    cloud->is_dense = false;
    cloud->points.resize (h*w);
    int v=-h/2;
    int k=0;
    const float* xyziptr=reinterpret_cast<const float*>(xyzi.data);
    for (int i=0; i<xyzi.rows; i++, v++){
      int u=-w/2;
      for (int j=0; j<xyzi.cols; j++, u++){
	const float& x=*xyziptr;
	const float& y=*(xyziptr+1);
	const float& z=*(xyziptr+2);
	const float& intns=*(xyziptr+3);
	pcl::PointXYZI& pt=cloud->points[k];
	pt.x = x;
	pt.y = y;
	pt.z = z;
	pt.intensity = intns;
	xyziptr+=4;
	k++;
      }
    }
  }

  inline Eigen::Vector4f p3d(int ix, int iy) {
    float* xyziptr=reinterpret_cast<float*>(xyzi.data);
    xyziptr+=(xyzi.cols*iy+ix)*4;
    return Eigen::Vector4f(*xyziptr,*(xyziptr+1), *(xyziptr+2), *(xyziptr+3));
    // Eigen::Map<Eigen::Vector4f> _p3d(xyziptr+(xyzi.cols*iy+ix)*4);
    // return _p3d;
  }

  void computeKeyPoints(FeatureDetector* detector){
    keyPoints.clear();
    detector->detect(intensity, keyPoints);
    spatialKeyPoints.resize(keyPoints.size());
    std::vector<KeyPoint> newKp(keyPoints.size());
    float bad_point = std::numeric_limits<float>::quiet_NaN();
    int rx=0;
    int ry=0;
    int k=0;
    for (size_t i=0; i<keyPoints.size(); i++){
      int ix=keyPoints[i].pt.x;
      int iy=keyPoints[i].pt.y;
      Eigen::Vector4f closestPoint=p3d(ix,iy);
      for (int dy=-ry; dy<=ry; dy++){
	for (int dx=-rx; dx<=rx; dx++){
	  Eigen::Vector4f p=p3d(rx+ix, ry+iy);
	  if (isnan(p.z()))
	    continue;
	  if (isnan(closestPoint.z())) {
	    closestPoint=p;
	    continue;
	  }
	  if (closestPoint.z()>p.z()) {
	    closestPoint=p;
	  }
	}
      }
      if(!isnan(closestPoint.z())){
	newKp[k]=keyPoints[i];
	spatialKeyPoints[k]=closestPoint;
	k++;
	//cerr << "distance ok, accepting     ";
	// cerr <<  closestPoint.x() << " ";
	// cerr <<  closestPoint.y() << " ";
	// cerr <<  closestPoint.z() << " ";
	// cerr <<  closestPoint.w() << " " <<endl;

      } else {
	//cerr << "error in distance, dropping" << endl;
      }
    }
    newKp.resize(k);
    cerr << "kp: " << keyPoints.size() << "/" << newKp.size();
    spatialKeyPoints.resize(k);
    keyPoints=newKp;
  }

  void computeDescriptors(DescriptorExtractor* extractor){
    extractor->compute(intensity,keyPoints,descriptors);
  }

  cv::Mat depth;
  cv::Mat intensity;
  float focalLength;
  float baseline;
  cv::Mat xyzi;
  vector<KeyPoint> keyPoints;
  vector<Eigen::Vector4f> spatialKeyPoints;
  Mat descriptors;
};

void bestFriendsFilter(std::vector<DMatch>& result, 
		       const std::vector<DMatch>& matches1, 
		       const std::vector<DMatch>& matches2){
  for (size_t i=0; i<matches1.size(); i++){
    const DMatch& match1=matches1[i];
    const DMatch& match2=matches2[match1.trainIdx];
    if (match1.queryIdx==match2.trainIdx) {
      result.push_back(match1);
    } 
  }	
} 

// struct Correspondence{
//   Eigen::Vector3f p1;
//   Eigen::Vector3f p2;
//   float strength;
// };

// typedef std::set<Correspondence*> CorrespondenceSet
// typedef std::vector<Correspondence*> CorrespondenceVector;


// struct CorrespondenceDistanceSampler{
//   float pairwiseDistance(const Correspondence& c1, const Correspondence& c2){
//     return fabs((c1.p1-c2.p1).norm()-(c2.p1-c2.p2).norm());
//   }
  
//   bool isValid(const Correspondence* c, const CorrespondenceSet& cset, float threshold){
//     for (CorrespondenceSet::const_iterator it=cset.begin(); it!=cset.end(); it++){
//       Correspondence*  ci=*it;
//       if (pairwiseDistance(*c,*ci)>threshold)
//         return false;
//     }
//     return true;
//   }

//   bool sampleOneMore(CorrespondenceVector& acceptedSet, CorrespondenceVector openSet, float threshold, int n){
//     if (n==0)
//       return true;
//     for (CorrespondenceSet::iterator it=openSet.begin(); it!=openSet.end(); it++) {
//       Correspondence* cnew=*it;
//       openSet.erase(cnew);
//       if (! isValid(cnew, acceptedSet, threshold)){
//         continue;
//       }
//       acceptedSet.insert(cnew);
//       bool validOneMore=sampleOneMore(acceptedSet, openSet, threshold, n-1);
//       if (validOneMore) {
//         return true;
//       } else {
//         acceptedSet.erase(cnew);
//       }
//     }
//     return false;
//   }
 
// };

void help()
{
	printf("\nThis program demonstrates using features2d detector, descriptor extractor and simple matcher\n"
			"Using the SURF desriptor:\n"
			"\n"
			"Usage:\n matcher_simple <image1> <image2>\n");
}

int main(int argc, char** argv)
{
	if(argc != 5)
	{
		help();
		return -1;
	}
	
	Mat img1 = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
	Mat img2 = imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);
	Mat depth1 = imread(argv[3], -1);
	Mat depth2 = imread(argv[4], -1);
	cerr << "channel size of depth image is: " << depth1.elemSize() << endl;
	if(img1.empty() || img2.empty() || depth1.empty() || depth2.empty())
	{
		printf("Can't read one of the images\n");
		return -1;
	}

	IntensityDepthImage intDepth1(img1, depth1, 0.075f, 575.0f);
	IntensityDepthImage intDepth2(img2, depth2, 0.075f, 575.0f);
	
	// detecting keypoints
	SurfFeatureDetector detector(400);
	intDepth1.computeKeyPoints(&detector);
	intDepth2.computeKeyPoints(&detector);

	SurfDescriptorExtractor extractor;
	intDepth1.computeDescriptors(&extractor);
	intDepth2.computeDescriptors(&extractor);


	// matching descriptors
	BFMatcher matcher(NORM_L2);
	vector<DMatch> matches1;
	vector<DMatch> matches2;
	matcher.match(intDepth1.descriptors, intDepth2.descriptors, matches1);
	matcher.match(intDepth2.descriptors, intDepth1.descriptors, matches2);
	
	vector<DMatch> matches;
	bestFriendsFilter(matches, matches1, matches2);

	// drawing the results
	namedWindow("matches", 1);
	Mat img_matches;
	drawMatches(intDepth1.intensity, intDepth1.keyPoints, intDepth2.intensity, intDepth2.keyPoints, matches, img_matches);
	imshow("matches", img_matches);
	
	cerr << "matches:" << endl;
	for (size_t i=0; i<matches.size(); i++){
	  const DMatch& match=matches[i];
	  cerr << i<< ":\t" <<  match.queryIdx << ", ";
	  cerr << intDepth1.spatialKeyPoints[match.queryIdx].x() << " ";
	  cerr << intDepth1.spatialKeyPoints[match.queryIdx].y() << " ";
	  cerr << intDepth1.spatialKeyPoints[match.queryIdx].z() << " -> ";
	  cerr << match.trainIdx << ", ";
	  cerr << intDepth2.spatialKeyPoints[match.trainIdx].x() << " ";
	  cerr << intDepth2.spatialKeyPoints[match.trainIdx].y() << " ";
	  cerr << intDepth2.spatialKeyPoints[match.trainIdx].z() << endl;
	}

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1 (new pcl::PointCloud <pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2 (new pcl::PointCloud <pcl::PointXYZI>);
	/*
	intDepth1.computePointCloud(cloud1);
	intDepth2.computePointCloud(cloud2);
	pcl::io::savePCDFileBinary("left.pcd",  *cloud1);
	pcl::io::savePCDFileBinary("right.pcd", *cloud2);
	*/

	waitKey(0);

	return 0;
}
