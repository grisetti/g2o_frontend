#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.hpp>
#include <sstream>
#include <fstream>
#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <queue>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;

struct KinectData
{
  cv::Mat depthImage;
  cv::Mat grayImage;
  geometry_msgs::TransformStamped tf;
  double timeStamp;
};

// Data queue to elaborate
typedef std::queue<KinectData*> DataQueue;

void generateGraph();

// Thread and mutex
boost::thread thrd;
boost::mutex mtx;

DataQueue _queue;

// Open file to write g2o format
ofstream ofG2O("graph.g2o");

int skip = 30;

// Variable to conatin the number of file to write on the g2o file
char name[8];

// Counter variable to numerate files
int num = 0;

// Windows name
static const char DEPTH_WINDOW[] = "Depth View";
static const char GRAY_WINDOW[] = "Gray View";

// Size of the type we get from sensor_msgs/Image type message
int size = sizeof(uint16_t);

// It's where we put the image that we will show
cv_bridge::CvImagePtr depth_image;
cv_bridge::CvImagePtr gray_image;

// Node pointer
ros::NodeHandle *nptr;

// Transformation listener
tf::TransformListener *tf_listener;

// offset to center the clouds
tf::Vector3 offset = tf::Vector3(0.0f, 0.0f, 0.0f);

void saveCb(const ImageConstPtr& raw_gray_image, const ImageConstPtr& raw_depth_image)
{
	bool we_got_transf = false;
	geometry_msgs::TransformStamped msg;
	// Convert the gray and depth image in input to a OpenCV image type
	try
	{
		gray_image = cv_bridge::toCvCopy(raw_gray_image, raw_gray_image->encoding);
		depth_image = cv_bridge::toCvCopy(raw_depth_image, raw_depth_image->encoding);
	}
	catch (cv_bridge::Exception& ex)
	{
		ROS_ERROR("cv_bridge exception: %s", ex.what());
		return;
	}
		
	if(nptr->ok())
	{
		tf::StampedTransform transform;	
		try
		{
		// Get transformation
			(*tf_listener).lookupTransform("/map", 
											"/kinect_camera", 
											raw_depth_image->header.stamp, 
											transform);
			we_got_transf = true;
			tf::transformStampedTFToMsg(transform, msg);
		}
		catch(tf::TransformException & ex)
		{
			ROS_ERROR("%s", ex.what());
		}
	}

	imshow(GRAY_WINDOW, gray_image->image);
	const float scaleFactor = 0.05f;
	Mat show; 
	depth_image->image.convertTo(show, CV_8UC1, scaleFactor);
	imshow(DEPTH_WINDOW, show);
	
	if(we_got_transf)
	{
		KinectData *frame = new KinectData();
		frame->depthImage = depth_image->image;
		frame->grayImage = gray_image->image;
		frame->tf = msg;
		frame->timeStamp = raw_depth_image->header.stamp.toSec();
		{
			mtx.lock();
			_queue.push(frame);
			mtx.unlock();
		}
	}

	char c = waitKey(33);  
	if(c == 'q') 
	{
		// Close g2o graph file.
		ofG2O.close();
		exit(0);
	}
}

void generateGraph()
{
	while(1)
	{
		KinectData *data = 0;
		{
			mtx.lock();
			if(!_queue.empty())
			{
				data = _queue.front();
				_queue.pop();
				cout << "Time: " << data->timeStamp << endl;
			}
			mtx.unlock();
		}
		if(data)
		{
			cv::Mat depth;
			cv::Mat gray;
			sprintf(name, "%05d", num);
			char buf[100];
			sprintf(buf, "%05d_intensity.pgm", num);
			imwrite(buf, gray_image->image);
			sprintf(buf, "%05d_depth.pgm", num);
			imwrite(buf, depth_image->image);
			cout << "Saved frame #"<< num << endl;
			if(num == 0)
			{
				offset = tf::Vector3(data->tf.transform.translation.x, 
									 data->tf.transform.translation.y, 
									 data->tf.transform.translation.z);
			}
			// Traslate all to be centered on the origin
			ofG2O << "VERTEX_SE3:QUAT " 
				<< num << " " 
				<< data->tf.transform.translation.x - offset.getX() << " " 
				<< data->tf.transform.translation.y - offset.getY() << " " 
				<< data->tf.transform.translation.z - offset.getZ() << " "
				<< data->tf.transform.rotation.x << " " 
				<< data->tf.transform.rotation.y << " " 
				<< data->tf.transform.rotation.z << " " 
				<< data->tf.transform.rotation.w 
				<< std::endl;
			if(num % skip == 0)
			{
				ofG2O << "DATA_DEPTH_REGISTERED 0 " 
					  << name << " " 
					  << num << " 0" 
					  << std::endl;
			}
			num++;
		}
		delete(data);
		usleep(20000); 
	}
}

int main(int argc, char* argv[])
{
	// Initialize ros
	ros::init(argc, argv, "data_grabber");
	ros::NodeHandle nh;
	nptr = &nh;
	
	// Initialize windows
	cv::namedWindow(DEPTH_WINDOW, CV_WINDOW_AUTOSIZE);
	cvMoveWindow(DEPTH_WINDOW, 0, 0);
	cv::namedWindow(GRAY_WINDOW, CV_WINDOW_AUTOSIZE);
	cvMoveWindow(GRAY_WINDOW, 641, 0);

	// Subscribe to topics
	message_filters::Subscriber<Image> depth_sub(nh, "/camera/depth/image_raw", 5);
	message_filters::Subscriber<Image> gray_sub(nh, "/camera/rgb/image_mono", 5);
	typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), gray_sub, depth_sub);
	sync.registerCallback(boost::bind(&saveCb, _1, _2));
	tf_listener = new tf::TransformListener;
	thrd = boost::thread(generateGraph);
	ros::spin();
	
	return(0);
}
