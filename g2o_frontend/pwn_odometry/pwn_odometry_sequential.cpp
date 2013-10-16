#include <QApplication>
#include <QMainWindow>
#include <QHBoxLayout>

#include "g2o/stuff/command_args.h"

#include "g2o_frontend/pwn2/frame.h"

#include "g2o_frontend/pwn_viewer/pwn_qglviewer.h"
#include "g2o_frontend/pwn_viewer/drawable_frame.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_frame.h"
#include "g2o_frontend/pwn_viewer/drawable_trajectory.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_trajectory.h"

#include "pwn_odometry_sequential_controller.h"

using namespace std;
using namespace Eigen;
using namespace g2o;
using namespace pwn;

int main (int argc, char** argv) {
  // Handle input
  int vz_applyTransform, vz_pointStep;
  int pwn_chunkStep;
  float vz_pointSize, vz_alpha;
  float pwn_scaleFactor, pwn_scale;

  string pwn_configFilename, pwn_logFilename, pwn_sensorType;
  string oc_benchmarkFilename, oc_trajectoryFilename, oc_groundTruthFilename, oc_associationsFilename;

  string directory;

  g2o::CommandArgs arg;
  arg.param("vz_pointSize", vz_pointSize, 1.0f, "Size value of the points to visualize");
  arg.param("vz_transform", vz_applyTransform, 1, "Apply absolute transform to the point clouds");
  arg.param("vz_alpha", vz_alpha, 1.0f, "Alpha channel value of the points to visualize");
  arg.param("vz_pointStep", vz_pointStep, 1, "Step value at which points are drawn");
  
  arg.param("pwn_chunkStep", pwn_chunkStep, 30, "Number of cloud composing a registered scene");
  arg.param("pwn_scaleFactor", pwn_scaleFactor, 0.001f, "Scale factor at which the depth images were saved");
  arg.param("pwn_scale", pwn_scale, 4.0f, "Scale factor the depth images are loaded");
  arg.param("pwn_configFilename", pwn_configFilename, "pwn.conf", "Specify the name of the file that contains the parameter configurations of the pwn structures");
  arg.param("pwn_logFilename", pwn_logFilename, "pwn.log", "Specify the name of the file that will contain the trajectory computed in boss format");
  arg.param("pwn_sensorType", pwn_sensorType, "kinect", "Sensor type: xtion640/xtion320/kinect/kinectFreiburg1/kinectFreiburg2/kinectFreiburg3");  
  arg.param("oc_benchmarkFilename", oc_benchmarkFilename, "pwn_benchmark.txt", "Specify the name of the file that will contain the results of the benchmark");
  arg.param("oc_trajectoryFilename", oc_trajectoryFilename, "pwn_trajectory.txt", "Specify the name of the file that will contain the trajectory computed");
  arg.param("oc_associationsFilename", oc_associationsFilename, "pwn_associations.txt", "Specify the name of the file that contains the images associations");
  arg.param("oc_groundTruthFilename", oc_groundTruthFilename, "groundtruth.txt", "Specify the name of the file that contains the ground truth trajectory");

  arg.paramLeftOver("directory", directory, ".", "Directory where the program will find the input needed files and the subfolders depth and rgb containing the images", true);

  arg.parseArgs(argc, argv);
  
  // Create GUI
  QApplication application(argc,argv);
  QWidget* mainWindow = new QWidget();
  mainWindow->setWindowTitle("PWN Odometry Sequential");
  QHBoxLayout* hlayout = new QHBoxLayout();
  mainWindow->setLayout(hlayout);
  PWNQGLViewer* viewer = new PWNQGLViewer(mainWindow);
  hlayout->addWidget(viewer);
  viewer->init();
  viewer->setAxisIsDrawn(true);
  viewer->show();
  mainWindow->showMaximized();
  mainWindow->show();
  
  // Init odometry controller
  PWNOdometrySequentialController *pwnOdometrySequentialController = new PWNOdometrySequentialController(pwn_configFilename.c_str(), pwn_logFilename.c_str());
  pwnOdometrySequentialController->fileInitialization(oc_groundTruthFilename.c_str(), oc_associationsFilename.c_str(),
						      oc_trajectoryFilename.c_str(), oc_benchmarkFilename.c_str());
  pwnOdometrySequentialController->setScaleFactor(pwn_scaleFactor);
  pwnOdometrySequentialController->setScale(pwn_scale);
  pwnOdometrySequentialController->setSensorType(pwn_sensorType);
  pwnOdometrySequentialController->setChunkStep(pwn_chunkStep);

  // Compute odometry
  bool sceneHasChanged = false;
  Frame *frame = 0, *groundTruthReferenceFrame = 0;
  Isometry3f groundTruthPose;	
  GLParameterTrajectory *groundTruthTrajectoryParam = new GLParameterTrajectory(0.02f, Vector4f(0.0f, 1.0f, 0.0f, 0.6f));
  GLParameterTrajectory *trajectoryParam = new GLParameterTrajectory(0.02f, Vector4f(1.0f, 0.0f, 0.0f, 0.6f));
  std::vector<Isometry3f> trajectory, groundTruthTrajectory;
  std::vector<Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > trajectoryColors, groundTruthTrajectoryColors;
  DrawableTrajectory *drawableGroundTruthTrajectory = new DrawableTrajectory(Isometry3f::Identity(), 
									    groundTruthTrajectoryParam, 
									    &groundTruthTrajectory, 
									    &groundTruthTrajectoryColors);
  DrawableTrajectory *drawableTrajectory = new DrawableTrajectory(Isometry3f::Identity(), 
								 trajectoryParam, 
								 &trajectory, 
								 &trajectoryColors);
  viewer->addDrawable(drawableGroundTruthTrajectory);
  viewer->addDrawable(drawableTrajectory);
  bool finished = false;
  while (mainWindow->isVisible()) {
    if (sceneHasChanged)
      viewer->updateGL();
    application.processEvents();

    sceneHasChanged = false;

    if (finished) {
      continue;
    }
    if (pwnOdometrySequentialController->loadFrame(frame)) {    
      if(pwnOdometrySequentialController->counter() == 1) {
	pwnOdometrySequentialController->getGroundTruthPose(groundTruthPose, atof(pwnOdometrySequentialController->timestamp().c_str()));
	// Add first frame to draw
	groundTruthReferenceFrame = new Frame();
	*groundTruthReferenceFrame = *frame;
	GLParameterFrame *groundTruthReferenceFrameParams = new GLParameterFrame();
	groundTruthReferenceFrameParams->setStep(vz_pointStep);
	groundTruthReferenceFrameParams->setShow(true);	
	groundTruthReferenceFrameParams->parameterPoints()->setColor(Vector4f(1.0f, 0.0f, 1.0f, vz_alpha));
	DrawableFrame *drawableGroundTruthReferenceFrame = new DrawableFrame(groundTruthPose, groundTruthReferenceFrameParams, groundTruthReferenceFrame);

	GLParameterFrame *frameParams = new GLParameterFrame();
	frameParams->setStep(vz_pointStep);
	frameParams->setShow(true);	
	DrawableFrame *drawableFrame = new DrawableFrame(pwnOdometrySequentialController->globalPose(), frameParams, frame);

	viewer->addDrawable(drawableGroundTruthReferenceFrame);
	viewer->addDrawable(drawableFrame);

	sceneHasChanged = true;
      }
      // Compute current transformation
      if (pwnOdometrySequentialController->processFrame()) {
	pwnOdometrySequentialController->getGroundTruthPose(groundTruthPose, atof(pwnOdometrySequentialController->timestamp().c_str()));	
	
	// Add frame to draw
	GLParameterFrame *frameParams = new GLParameterFrame();
	frameParams->setStep(vz_pointStep);
	frameParams->setShow(true);	
	DrawableFrame *drawableFrame = new DrawableFrame(pwnOdometrySequentialController->globalPose(), frameParams, frame);
	viewer->addDrawable(drawableFrame);
	
	// Add trajectory pose
	groundTruthTrajectory.push_back(groundTruthPose);
	groundTruthTrajectoryColors.push_back(Vector4f(0.0f, 1.0f, 0.0f, 0.6f));
	trajectory.push_back(pwnOdometrySequentialController->globalPose());
	trajectoryColors.push_back(Vector4f(1.0f, 0.0f, 0.0f, 0.6f));
	drawableGroundTruthTrajectory->updateTrajectoryDrawList();
	drawableTrajectory->updateTrajectoryDrawList();
      
	// Write results
	pwnOdometrySequentialController->writeResults();
      
	sceneHasChanged = true;
      }
 
      // Remove old frames
      if (viewer->drawableList().size() > 23) {
	DrawableFrame *d = dynamic_cast<DrawableFrame*>(viewer->drawableList()[3]);
	if (d) {
	  Frame *f = d->frame();
	  if (pwnOdometrySequentialController->referenceFrame() != f &&
	      pwnOdometrySequentialController->currentFrame() != f) {
	    viewer->erase(3);
	    delete d;
	    delete f; 
	  }
	}
      }

      frame = 0;
    }
    else {
      finished = true;
    }
  }

  return 0;
}
