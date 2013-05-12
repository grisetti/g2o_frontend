#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>
#include <fstream>
#include <string>
#include <set>
#include <sstream>

#include "g2o/stuff/command_args.h"
#include "depthimage.h"
#include "depthimageconverter.h"
#include "pinholepointprojector.h"

#include "linearizer.h"
#include "correspondencegenerator.h"
#include "aligner.h"


using namespace Eigen;
using namespace g2o;
using namespace std;

set<string> readDir(std::string dir){
  DIR *dp;
  struct dirent *dirp;
  struct stat filestat;
  std::set<std::string> filenames;
  dp = opendir( dir.c_str() );
  if (dp == NULL){
    return filenames;
  }
  
  while ((dirp = readdir( dp ))) {
    string filepath = dir + "/" + dirp->d_name;

    // If the file is a directory (or is in some way invalid) we'll skip it 
    if (stat( filepath.c_str(), &filestat )) continue;
    if (S_ISDIR( filestat.st_mode ))         continue;

    filenames.insert(filepath);
  }

  closedir( dp );
  return filenames;
}


int
 main(int argc, char** argv){
  string dirname;
  
  PinholePointProjector projector;
  HomogeneousPoint3fStatsGenerator statsGenerator;
  PointOmegaGenerator pointOmegaGenetator;
  NormalOmegaGenerator normalOmegaGenerator;
  DepthImageConverter converter(&projector, 
				&statsGenerator, 
				&pointOmegaGenetator, 
				&normalOmegaGenerator);
  
  g2o::CommandArgs arg;
  arg.paramLeftOver("dirname", dirname, "", "", true);
  arg.parseArgs(argc, argv);
  cerr << "dirname " << dirname << endl;

  std::vector<string> filenames;
  std::set<string> filenamesset = readDir(dirname);
  for(set<string>::const_iterator it =filenamesset.begin(); it!=filenamesset.end(); it++) {
    filenames.push_back(*it);
  }

  Eigen::Matrix3f cameraMatrix;
  cameraMatrix << 
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;

  Eigen::Matrix3f scaledCameraMatrix = cameraMatrix* 0.5;
  cameraMatrix(2,2) = 1.0f;
  
  projector.setCameraMatrix(scaledCameraMatrix);
  
  DepthImage depthImage, scaledDepthImage;
  cerr << "there are " << filenames.size() << " files  in the pool" << endl; 

  Linearizer linearizer;
  CorrespondenceGenerator correspondenceGenerator;
  Aligner aligner;
  aligner.setProjector(&projector);
  projector.setCameraMatrix(scaledCameraMatrix);
  aligner.setLinearizer(&linearizer);
  linearizer.setAligner(&aligner);
  aligner.setCorrespondenceGenerator(&correspondenceGenerator);
  aligner.setOuterIterations(10);
  aligner.setInnerIterations(1);

  HomogeneousPoint3fScene* previousScene=0;

  for (size_t i=0; i<filenames.size(); i++){
    cerr << endl << endl << endl;
    cerr << ">>>>>>>>>>>>>>>>>>>>>>>> PROCESSING " << filenames[i] << " <<<<<<<<<<<<<<<<<<<<" <<  endl;
    if (!depthImage.load(filenames[i].c_str())){
      cerr << " skipping " << filenames[i] << endl;
      continue;
    }
    HomogeneousPoint3fScene* scene=new HomogeneousPoint3fScene();
    DepthImage::scale(scaledDepthImage, depthImage, 2);
    converter.compute(*scene, scaledDepthImage);
    int numNormals = 0;
    for (size_t i = 0; i<scene->normals().size(); i++){
      numNormals += (scene->normals()[i].squaredNorm()>0);
    }
    correspondenceGenerator.setSize(scaledDepthImage.rows(), scaledDepthImage.cols());
    cerr << "# points"  << scene->points().size() << " normals: " << numNormals << endl;
    if (previousScene){
      aligner.setInitialGuess(Eigen::Isometry3f::Identity());
      aligner.setSensorOffset(Eigen::Isometry3f::Identity());
      aligner.setReferenceScene(previousScene);
      aligner.setCurrentScene(scene);
      cerr << "align" << endl;
      aligner.align();
      delete previousScene;
    }
    previousScene=scene;
  }
}
