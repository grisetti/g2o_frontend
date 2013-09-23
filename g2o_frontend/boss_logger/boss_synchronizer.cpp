#include <list>
#include <set>
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"
#include "bframe.h"
#include "bframerelation.h"
#include "bimagesensor.h"
#include "blasersensor.h"
#include "bimusensor.h"
#include "bsynchronizer.h"
#include "brobot_configuration.h"
#include "g2o_frontend/pwn_boss/pwn_sensor_data.h"


using namespace boss_logger;
using namespace boss;
using namespace std;

pwn_boss::PWNSensorData data;


const char* banner[]={
  "boss_synchronizer: takes a raw boss log and aggregates messages into single frames",
  " based on time constraints.",
  " If you want to make a frame out of two topics that have a time distance of 0.01 sec you should use",
  " the -sync option.",
  "   -sync:topic1:topic2:time",
  " Adding multiple sync  options results in having frames containing multiple informations.",
  " Messages that do not match the constraints are dropped."
  "",
  "usage: boss_synchronizer [options] filein fileout",
  "example: boss_synchronizer \\",
  " -sync:/kinect/rgb/image_color:/kinect/depth_registered/image_raw:0.05 \\",
  " -sync:/kinect/rgb/image_color:/imu/data:0.1 \\",
  " -sync:/kinect/rgb/image_color:/front_scan:0.1 \\",
  " test.log test_sync.log", 0
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
    size_t i = 0;
    std::string trimmed=s;
    do{
      i =trimmed.find_first_of(':');
      values.push_back(trimmed.substr(0,i));
      trimmed = trimmed.substr(i+1,std::string::npos);
    } while (i != std::string::npos);

    cerr << "arg" << endl;
    for (i=0; i<values.size(); i++)
      cerr <<" " << values[i] << endl; 
  }

  int asInt(int i) const {
    return atoi(values[i].c_str());
  }
  float asFloat(int i) const {
    return atof(values[i].c_str());
  }

  bool asBool(int i) const {
    if (values[i] == "true")
      return true;
    if (values[i] == "false")
      return false;
    throw std::runtime_error("illegal bool value");
  }

  std::string asString(int i) const{
    return values[i];
  }

  std::vector<std::string> values;
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

void handleParsedArgs(Synchronizer* sync, std::list<CommandArg> args){
  for(std::list<CommandArg>::iterator it = args.begin(); it!=args.end(); it++){
    CommandArg& arg = *it;
    if (arg.asString(0)!="-sync")
      continue;
  // if (arg.values.size()!=4)
  //     throw runtime_error("options must be in the form -sync:topic1:topic2:time");
    std::string topic1 = arg.asString(1);
    std::string topic2 = arg.asString(2);
    double time = arg.asFloat(3);
    cerr << "sync: " << topic1 << " " << topic2 << " " << time << endl;
    sync->addSyncTimeCondition(topic1,topic2,time);
  }
 
}

int main(int argc, char** argv) {
  std::list<CommandArg> parsedArgs;
  if (argc == 1) {
    printBanner();
    return 0;
  }

  int c = parseArgs(parsedArgs, argc, argv);
  if (c<argc-2) {
    printBanner();
    return 0;
  }
    
  

  // create a synchronizer
  Synchronizer sync;

  handleParsedArgs(&sync, parsedArgs);
  std::string filein = argv[c];
  std::string fileout = argv[c+1];



  Deserializer des;
  des.setFilePath(filein.c_str());

  Serializer ser;
  ser.setFilePath(fileout.c_str());
  
  cerr <<  "running logger with arguments: filein[" << filein << "] fileout: [" << fileout << "]" << endl;

  // create a reframer object, that once all messages have been put together sets them to a unique frame
  Synchronizer::Reframer reframer;
  sync.addOutputHandler(&reframer);

  // create a writer object that dumps on the disk each block of synchronized objects
  Synchronizer::Writer writer(&ser);
  sync.addOutputHandler(&writer);

  // create a deleter object that polishes the memory after writing
  Synchronizer::Deleter deleter;
  sync.addOutputHandler(&deleter);

  std::vector<BaseSensorData*> sensorDatas;
  RobotConfiguration* conf = readLog(sensorDatas, des);
  cerr << "# frames: " << conf->frameMap().size() << endl;
  cerr << "# sensors: " << conf->sensorMap().size() << endl;
  cerr << "# sensorDatas: " << sensorDatas.size() << endl;

  conf->serializeInternals(ser);
  ser.writeObject(*conf);

  TSCompare comp;
  std::sort(sensorDatas.begin(), sensorDatas.end(), comp);

  for (size_t i = 0; i< sensorDatas.size(); i++){
    BaseSensorData* data = sensorDatas[i];
    sync.addSensorData(data);
  }

}
