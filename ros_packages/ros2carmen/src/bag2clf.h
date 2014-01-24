#ifndef _ROS2CARMEN_H_
#define _ROS2CARMEN_H_

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/transform_listener.h>

/*! \class ROS2CarmenBagReader
 * \brief Reads bag files for offline use of the tracker
 */
class ROS2CarmenBagReader {
public:
  ROS2CarmenBagReader(std::string odom="/odom",
	              std::string scan="/scan",
		      std::string tf="/tf" );

  ~ROS2CarmenBagReader();
  void readBag(std::string filename, std::string outfilename);
private:
  std::string topicOdom, topicScan, topicTf;
  tf::Transformer _transformer;
};


#endif
