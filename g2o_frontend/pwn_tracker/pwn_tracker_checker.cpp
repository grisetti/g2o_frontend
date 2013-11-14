
#include "g2o_frontend/boss_logger/bframe.h"
#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/aligner.h"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"

#include "opencv2/highgui/highgui.hpp"
#include "g2o_frontend/boss_map/boss_map_g2o_reflector.h"
#include <fstream>
#include <iostream>
#include <queue>
#include "pwn_tracker.h"
#include "g2o_frontend/boss_map/boss_map_utils.h"
#include "cache.h"

using namespace std;
using namespace pwn;
using namespace boss;
using namespace boss_map;
using namespace pwn_tracker;

#include "pwn_closer.h"

MapManager* load(std::vector<Serializable*>& objects,
		 Deserializer& des){
  Serializable* o=0;
  boss_map::MapManager* manager = 0;
  while( (o=des.readObject()) ){
    objects.push_back(o);
    boss_map::MapManager* m = dynamic_cast<boss_map::MapManager*>(o);
    if (m) {
      return m;
    }
  }
  return 0;
}

int main(int argc, char** argv){
  if (argc<3) {
    cerr << " u should provide a config file and an input file" << endl;
    return 0;
  }
  pwn::Aligner* aligner;
  pwn::DepthImageConverter* converter;
  std::vector<Serializable*> instances = readConfig(aligner, converter, argv[1]);
  cerr << "config loaded" << endl;
  cerr << " aligner:" << aligner << endl;
  cerr << " converter:" << converter << endl;
  
  Deserializer des;
  Serializer  ser;
  des.setFilePath(argv[2]);
  // load the log
  std::vector<boss::Serializable*> objects;

  MapManager* manager = load(objects, des);
  if (! manager) 
    return 0;
  
  PwnCloser* closer = new PwnCloser(aligner,converter,manager);
  closer->setScale(4);
  
  boss::Serializable* o=0;
  while( (o=des.readObject()) ){
    objects.push_back(o);
    PwnTrackerFrame* f = dynamic_cast<PwnTrackerFrame*>(o);
    if (f) {
      closer->addFrame(f);
    }
    PwnTrackerRelation* r = dynamic_cast<PwnTrackerRelation*>(o);
    if (r) {
      closer->addRelation(r);
    }
  }
  
  cerr << "end, detected closures:" << closer->results().size() << endl;
  for (size_t i =0; i<closer->results().size(); i++){
    MatchingResult& result = closer->results()[i];
    char fname[1024];
    sprintf(fname, "match-%05d-%05d.pgm",result.from->seq, result.to->seq);
    cerr << "i: " << i <<  " [" << fname << "]" <<  endl;
    cv::Mat m=result.diffRegistered;
    cv::imwrite(fname, m);
  }
  return 0;
}
