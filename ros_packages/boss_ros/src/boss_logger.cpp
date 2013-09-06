#include <iostream>
#include "ros/ros.h"
#include "ros_message_context.h"
#include "ros_message_handler.h"
#include "ros_transform_message_handler.h"
#include "ros_pinholeimagedata_message_handler.h"
#include "ros_laser_message_handler.h"
#include "ros_imu_data_message_handler.h"


#include "g2o_frontend/boss/deserializer.h"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss_logger/bimagesensor.h"
#include "g2o_frontend/boss_logger/blasersensor.h"
#include "g2o_frontend/boss_logger/bimusensor.h"
#include "g2o_frontend/boss_logger/bsynchronizer.h"
using namespace std;


const char* banner[]={
  "boss_logger: listens to ros topics and generates a boss log",
  " Informations you need to tell the system:",
  " - the type and topic you want to save. The following types are supported",
  "   - Images, use the '-image:topic' option, e.g. '-image:/camera_array/camera_image';"
  "   - Laser,  use the '-laser:topic' option  e.g.  '-laser:/front_laser';",
  "   - IMU,    use the '-imu:topic' option    e.g.  '-imu:/top_imu';",
  " - the odom frame id, by using the '-odomReferenceFrame:frame' option e.g.: '-odomReferenceFrame:/odom';",
  "   if unspecified it defaults to '/odom'.",
  " - the base frame id, by using the '-baseReferenceFrame:frame' option e.g.: '-baseReferenceFrame:/base_link';",
  "   if unspecified it defaults to '/base_link'.", 
  "",
  "call it with: boss_logger [arguments] <output filename>",
  "example: rosrun boss_ros boss_logger -image:/kinect/depth_registered/image_raw -image:/kinect/rgb/image_color -imu:/imu/data -laser:/front_scan test.log",
  0,
};

void printBanner (){
  int c=0;
  while (banner[c]){
    cerr << banner [c] << endl;
    c++;
  }
}

struct CommandArg{
  CommandArg(const std::string s){
    size_t i =s.find_first_of(':');
    if (i == std::string::npos)
      throw std::runtime_error("malformed parameter type arg should be \"param:value\" ");
    param = s.substr(0,i);
    value = s.substr(i+1,s.length()-1);
  }

  int asInt() const {
    return atoi(value.c_str());
  }
  float asFloat() const {
    return atof(value.c_str());
  }

  bool asBool() const {
    if (value == "true")
      return true;
    if (value == "false")
      return false;
    throw std::runtime_error("illegal bool value");
  }

  std::string asString() const{
    return value;
  }

  std::string param;
  std::string value;
};

int parseArgs(std::list<CommandArg>& parsedArgs, int argc, char** argv){
  int c = 1;
  parsedArgs.clear();
  while (c<argc){
    if (*argv[c]=='-') {
      CommandArg arg(CommandArg(argv[c]));
      parsedArgs.push_back(arg);
      c++;
    } else {
      return c;
    }
  }
  return c;
}

void processArgs(RosMessageContext* context, std::list<CommandArg>& list){
  for (std::list<CommandArg>::iterator it = list.begin(); it!=list.end(); it++){
    const CommandArg& arg = *it;
    if (arg.param =="-image"){
      context->addHandler("image", arg.asString());
      continue;
    }
    if (arg.param =="-laser"){
      context->addHandler("laser", arg.asString());
      continue;
    }
    if (arg.param =="-imu"){
      context->addHandler("imu", arg.asString());
      continue;
    }
    if (arg.param =="-odomReferenceFrame"){
      context->setOdomReferenceFrameId(arg.asString());
      continue;
    }
    if (arg.param =="-baseReferenceFrame"){
      context->setBaseReferenceFrameId(arg.asString());
      continue;
    }
    throw std::runtime_error("unknown argument");
  }
}

int main(int argc, char** argv){
  std::list<CommandArg> parsedArgs;
  if (argc <2){
    printBanner();
    return 0;
  }
  int c = parseArgs(parsedArgs, argc, argv);
  
  if (c!=argc-1){
    cerr << "no filename provided, exiting" << endl;
    return 0;
  }
  std::string filename = argv[c];
  
  boss::Serializer ser;
  ser.setFilePath(filename);
  
  ros::init(argc, argv, "boss_logger");
  ros::NodeHandle nh;

  cerr << "Started the logger on file [" << filename << "]"  << endl; 
  // Add your handlers
  RosMessageContext context(&nh);

  // tell the context what is the odom frame and the base link frame
  context.setOdomReferenceFrameId("/odom");
  context.setBaseReferenceFrameId("/base_link");

  processArgs(&context, parsedArgs);
  
  // cmd line:
  // rosrun boss_ros boss_logger -image:/kinect/depth_registered/image_raw -image:/kinect/rgb/image_color -imu:/imu/data -laser:/front_scan
  // // register the necessary handlers
  // context.addHandler("laser","/front_scan");
  // context.addHandler("imu","/imu/data");
  // context.addHandler("image","/kinect/depth_registered/image_raw");
  // context.addHandler("image","/kinect/rgb/image_color");

  // start the machine
  context.init();

  bool confReady=false;
  while (ros::ok()){
    ros::spinOnce();
    // cerr << "tf: " << tfReferenceFrameMap.size() 
    // 	 << ", sensors: "<<sensorTopicMap.size()
    // 	 << ", messages: "<<dataQueue.size() << endl;
    bool isReady = context.configReady();
    // we got all transforms and sensors, so we can dump the configuration
    if (! confReady && isReady){
      cerr << endl << "CONF IS NOW READY!!!, STARTING WRITING" << endl;
      context.serializeInternals(ser);
      boss::RobotConfiguration conf = context;
      conf.serializeInternals(ser);
      ser.writeObject(conf);
      // for (boss::StringReferenceFrameMap::iterator it = context.frameMap().begin(); it!=context.frameMap().end(); it++)
      // 	ser.writeObject(*it->second);
      // for (boss::StringSensorMap::iterator it = context.sensorMap().begin(); it!=context.sensorMap().end(); it++)
      // 	ser.writeObject(*it->second);
      confReady = isReady;
    }
    if (confReady){
      while (context.messageQueue().size()){
	boss::BaseSensorData* data = dynamic_cast<boss::BaseSensorData*>(context.messageQueue().front());
	if (! data){
	  cerr << "fatal error, inconsistent data" << endl; 
	  return 0;
	}
	context.messageQueue().pop_front();
	boss::ReferenceFrame* f = data->robotReferenceFrame();
	if (f){
	  ser.writeObject(*f);
	}
	ser.writeObject(*data);
	if (f)
	  delete f;
	delete data;
	  cerr << '.';
      }
    }
    // start emptying the queue of the writable objects
  }
}
