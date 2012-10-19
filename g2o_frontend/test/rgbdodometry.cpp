#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "Eigen/Geometry"

#include <cstdio>
#include <iostream>
#include <fstream>

#include <ctime>

using namespace cv;
using namespace std;

static
void cvtDepth2Cloud( const Mat& depth, Mat& cloud, const Mat& cameraMatrix )
{
    const float inv_fx = 1.f/cameraMatrix.at<float>(0,0);
    const float inv_fy = 1.f/cameraMatrix.at<float>(1,1);
    const float ox = cameraMatrix.at<float>(0,2);
    const float oy = cameraMatrix.at<float>(1,2);
    cloud.create( depth.size(), CV_32FC3 );
    for( int y = 0; y < cloud.rows; y++ )
    {
        Point3f* cloud_ptr = (Point3f*)cloud.ptr(y);
        const float* depth_prt = (const float*) depth.ptr(y);
        for( int x = 0; x < cloud.cols; x++ )
        {
            float z = depth_prt[x];
            cloud_ptr[x].x = (x - ox) * z * inv_fx;
            cloud_ptr[x].y = (y - oy) * z * inv_fy;
            cloud_ptr[x].z = z;
        }
    }
}

template<class ImageElemType>
static void warpImage( const Mat& image, const Mat& depth,
                       const Mat& Rt, const Mat& cameraMatrix, const Mat& distCoeff,
                       Mat& warpedImage )
{
    const Rect rect = Rect(0, 0, image.cols, image.rows);

    vector<Point2f> points2d;
    Mat cloud, transformedCloud;

    cvtDepth2Cloud( depth, cloud, cameraMatrix );
    perspectiveTransform( cloud, transformedCloud, Rt );
    projectPoints( transformedCloud.reshape(3,1), Mat::eye(3,3,CV_64FC1), Mat::zeros(3,1,CV_64FC1), cameraMatrix, distCoeff, points2d );

    Mat pointsPositions( points2d );
    pointsPositions = pointsPositions.reshape( 2, image.rows );

    warpedImage.create( image.size(), image.type() );
    warpedImage = Scalar::all(0);

    Mat zBuffer( image.size(), CV_32FC1, FLT_MAX );
    for( int y = 0; y < image.rows; y++ )
    {
        for( int x = 0; x < image.cols; x++ )
        {
            const Point3f p3d = transformedCloud.at<Point3f>(y,x);
            const Point p2d = pointsPositions.at<Point2f>(y,x);
            if( !cvIsNaN(cloud.at<Point3f>(y,x).z) && cloud.at<Point3f>(y,x).z > 0 &&
                rect.contains(p2d) && zBuffer.at<float>(p2d) > p3d.z )
            {
                warpedImage.at<ImageElemType>(p2d) = image.at<ImageElemType>(y,x);
                zBuffer.at<float>(p2d) = p3d.z;
            }
        }
    }
}

Eigen::Isometry3d fromMat(const cv::Mat m){
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  Eigen::Isometry3d T;
  T.setIdentity();
  for (int i=0; i<4; i++) {
    for (int j=0; j<4; j++)
      T.matrix()(i,j)=m.at<double>(i,j);
  }
  return T;
}

int main(int argc, char** argv)
{
    float vals[] = {525., 0., 3.1950000000000000e+02,
                    0., 525., 2.3950000000000000e+02,
                    0., 0., 1.};

    const Mat cameraMatrix = Mat(3,3,CV_32FC1,vals);
    const Mat distCoeff(1,5,CV_32FC1,Scalar(0));

    ofstream os("trajectory.dat");
    if( argc < 2 )
    {
        cout << "Format: num_images [transformationType]" << endl;
        cout << "Depth file must be 16U image stored depth in mm." << endl;
        cout << "Transformation types:" << endl;
        cout << "   -rbm - rigid body motion (default)" << endl;
        cout << "   -r   - rotation rotation only" << endl;
        cout << "   -t   - translation only" << endl;
        return -1;
    }

    int imageNum=0;
    imageNum=atoi(argv[1]);

    int transformationType = RIGID_BODY_MOTION;
    if( argc == 2 )
    {
        string ttype = argv[2];
        if( ttype == "-rbm" )
        {
            transformationType = RIGID_BODY_MOTION;
        }
        else if ( ttype == "-r")
        {
            transformationType = ROTATION;
        }
        else if ( ttype == "-t")
        {
            transformationType = TRANSLATION;
        }
        else
        {
            cout << "Unsupported transformation type." << endl;
            return -1;
        }
    }
    
    Mat previousImage;
    Mat previousDepth;
    Mat previousDepthFlt;
    Eigen::Isometry3d T;
    T.setIdentity();
    for (int i=0; i<imageNum; i++) {
      char iname[40];
      sprintf(iname, "test-%03d-GRAY.pgm",i);
      char dname[40];
      sprintf(dname, "test-%03d-DEPT.pgm",i);
      
      cerr << "****************** Frame: " << i << " ****************** " <<   endl;
      if (i==0){
	previousImage = imread( iname, -1);
	previousDepth = imread( dname, -1 );
	previousDepth.convertTo(previousDepthFlt, CV_32FC1, 1./1000 );
	continue;
      }

      Mat image = imread( iname, -1 );
      Mat depth = imread( dname, -1 );
      Mat depthFlt;
      depth.convertTo(depthFlt, CV_32FC1, 1./1000 );

      TickMeter tm;
      Mat Rt;
      
      vector<int> iterCounts(4);
      iterCounts[0] = 7;
      iterCounts[1] = 7;
      iterCounts[2] = 7;
      iterCounts[3] = 10;
      
      vector<float> minGradMagnitudes(4);
      minGradMagnitudes[0] = 12;
      minGradMagnitudes[1] = 5;
      minGradMagnitudes[2] = 3;
      minGradMagnitudes[3] = 1;
      
      const float minDepth = 0.f; //in meters
      const float maxDepth = 3.f; //in meters
      const float maxDepthDiff = 0.07f; //in meters
      
      tm.start();
      cerr << previousDepthFlt.rows << " " << previousDepthFlt.cols << endl;
      cerr << depthFlt.rows << " " << depthFlt.cols << endl;
      bool isFound = cv::RGBDOdometry( Rt, Mat(),
				       previousImage, previousDepthFlt, Mat(),
				       image, depthFlt, Mat(),
				       cameraMatrix, minDepth, maxDepth, maxDepthDiff,
				       iterCounts, minGradMagnitudes, transformationType );
      tm.stop();
      
      if (isFound){
	cout << "***************** Success ***************** " << endl;
	cout << "Frame: " << i << endl;
	Eigen::Isometry3d delta=fromMat(Rt);
	cout << "Delta: " << endl;
	cout << delta.matrix();
	cout << endl;
	//cout << "Rt = " << Rt << endl;
	T=T*delta;
	cout << "Global: " << endl;
	cout << T.matrix();
	cout << endl;
	cout << "Time = " << tm.getTimeSec() << " sec." << endl << endl;
	os << i 
	   << " " << T.matrix()(0,3) 
	   << " " << T.matrix()(1,3) 
	   << " " << T.matrix()(2,3) << endl;

      } else {
	cout << "***************** Fail ***************** " << endl;
      }

      previousImage=image;
      previousDepth=depth;
      previousDepthFlt=depthFlt;
    }
    return 0;
}
