#ifndef _PWN_VIEWER_STATE_H_
#define _PWN_VIEWER_STATE_H_

#include "g2o_frontend/basemath/bm_se3.h"
#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/informationmatrixfinder.h"
#include "g2o_frontend/pwn2/statsfinder.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/aligner.h"
#include "pwn_gui_main_window.h"
#include "drawable_frame.h"


#include "g2o_frontend/sensor_data/laser_robot_data.h"
#include "g2o_frontend/sensor_data/rgbd_data.h"
#include "g2o_frontend/sensor_data/imu_data.h"
#include "g2o/core/sparse_optimizer.h"


namespace pwn{

struct ViewerState{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  PWNGuiMainWindow* pwnGMW;
  QGraphicsScene *refScn, *currScn;
  g2o::SparseOptimizer* graph;
  std::vector<RGBDData*> listAssociations;

  PinholePointProjector* projector;
  StatsFinder* statsFinder;
  PointInformationMatrixFinder* pointInformationMatrixFinder;
  NormalInformationMatrixFinder* normalInformationMatrixFinder;
  DepthImageConverter* converter;
  TraversabilityAnalyzer* traversabilityAnalyzer;


  Matrix3f cameraMatrix;
  Eigen::Isometry3f sensorOffset;
  
  CorrespondenceFinder* correspondenceFinder;
  Linearizer* linearizer;

  Aligner* aligner;

  bool newCloudAdded, wasInitialGuess;
  bool *initialGuessViewer, *optimizeViewer, *addCloud, *clearLast, *clearAll;
  bool *mergePressed;
  int *stepViewer, *stepByStepViewer;
  float *pointsViewer, *normalsViewer, *covariancesViewer, *correspondencesViewer;
  QListWidgetItem* listItem;
  QListWidget* listWidget;

  std::vector<DrawableFrame*> drawableFrameVector;
  Isometry3f initialGuess;
  Isometry3f globalT;
  DrawableFrameParameters* drawableFrameParameters;
  int imageRows, imageCols;

  int reduction; // inverse integer scaling for the images

  float ng_scale;
  float ng_worldRadius;
  float ng_curvatureThreshold;
  int ng_minImageRadius;
  float if_curvatureThreshold;
  int al_innerIterations;
  int al_outerIterations;
  int vz_step;
  
  bool _meHasNewFrame;

  bool continuousMode;

  ViewerState(PWNGuiMainWindow* mwin);

  // sets up the structures and constructs the graph
  void init();
  // refreshes the variables used to communicate with the gui
  void refreshFlags();
  // loads a graph and updates the list of things displayed;
  void load(const std::string& filename);
  // clears the view
  void clear();  
  // updates the glviwers according to the display parameters
  void updateDrawableParameters();
  // handles the initial guess action
  void initialGuessSelected();
  // handles the optimize action
  void optimizeSelected();
  // handles the addCloud action
  void addCloudSelected();
  // adds the selected frame and optimizes it
  void addNextAndOptimizeSelected();
  // handles the clearLast action
  void clearLastSelected();
  // handles the processing of the commands (to be called within the event handling loop 
  void processCommands();



  void polishOldThings();
};

} // end namespace
#endif
