
#include "g2o_frontend/boss_logger/bframe.h"
#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/aligner.h"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/stuff/macros.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "opencv2/highgui/highgui.hpp"
#include "g2o_frontend/boss_map/boss_map_g2o_reflector.h"
#include <fstream>
#include <iostream>
#include <queue>
#include "pwn_tracker.h"
#include "g2o_frontend/boss_map/boss_map_utils.h"
#include "cache.h"
#include "pwn_closer.h"
#include "pwn_tracker_g2o_wrapper.h"

using namespace std;
using namespace g2o;
using namespace pwn;
using namespace boss;
using namespace boss_map;
using namespace pwn_tracker;


MapManager* load(std::vector<Serializable*>& objects,
		 Deserializer& des){
  Serializable* o=0;
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
  pwn::Aligner* aligner;  pwn::DepthImageConverter* converter;
  std::vector<Serializable*> instances = readConfig(aligner, converter, argv[1]);
  cerr << "config loaded" << endl;
  cerr << " aligner:" << aligner << endl;
  cerr << " converter:" << converter << endl;
  
  Deserializer des;
  des.setFilePath(argv[2]);
  // load the log
  std::vector<boss::Serializable*> objects;

  MapManager* manager = load(objects, des);
  if (! manager) 
    return 0;

  // install the optimization wrapper
  G2oWrapper* wrapper = new G2oWrapper(manager);

  int scale = 4;
  // create a cache for the frames
  PwnCache* cache  = new PwnCache(converter, scale, 100);
  PwnCacheHandler* cacheHandler = new PwnCacheHandler(manager, cache);
  manager->actionHandlers().push_back(cacheHandler);
  cacheHandler->init();

  // create a closer
  PwnCloser* closer = new PwnCloser(aligner,converter,manager,cache);
  closer->setScale(scale);
  
  // install a closure criterion to select nodes in the clser
  DistancePoseAcceptanceCriterion criterion(manager);
  criterion.setRotationalDistance(M_PI/4);
  criterion.setTranslationalDistance(1);
  closer->setCriterion(&criterion);


  // sequentially process the data
  boss::Serializable* o=0;
  int count=0;

  while( (o=des.readObject()) ){
    objects.push_back(o);
    PwnTrackerFrame* f = dynamic_cast<PwnTrackerFrame*>(o);
    if (f) {
      //cache->addEntry(f);
      closer->addFrame(f);
      count++;
    }
    PwnTrackerRelation* r = dynamic_cast<PwnTrackerRelation*>(o);
    if (r) {
      closer->addRelation(r);
    }
    int committedRelations = closer->committedRelations().size();
    if (committedRelations ){
      char fname[100];
      wrapper->optimize();
      sprintf(fname, "out-%05d.g2o", count);
      wrapper->save(fname);
    }
  }

  cerr << "writing out" << endl;
  Serializer ser;
  ser.setFilePath("output.log");
  ser.writeObject(*manager);
  cerr << "writing out fixed input" << endl;
  for (size_t i = 0; i<objects.size(); i++)
    ser.writeObject(*objects[i]);
  return 0;
}
