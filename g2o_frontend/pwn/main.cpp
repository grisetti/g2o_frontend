#include <iostream>
#include <qapplication.h>
#include "pwn_normals.h"
#include "pwn_cloud.h"
#include "pwn_qglviewer.h"

using namespace std;

int main(int argc, char** argv)
{
  // If wrong number of parameters print the help menù.
  if (argc!=5){
    cout << "To launch the program write: \"./nord-odometry file.g2o sensorId numImage0 numImage1\"" << endl
	 << "  file.g2o\tname of the file .g2o to open" << endl
	 << "  sensorId\tid of the sensor used to save the rgbd images" << endl
	 << "  numImage0\tnumber of the image to be aligned" << endl
	 << "  numImage1\tnumber of the target image" << endl;
    return 0;
  }

  QApplication application(argc, argv);

  // Create camera matrix.
  Matrix3f cameraMatrix;
  cameraMatrix << 525.0f, 0.0f, 319.5f,
                  0.0f, 525.0f, 239.5f,
                  0.0f, 0.0f, 1.0f;

  // Open file .g2o.
  ifstream ifG2O(argv[1]);
  
  // Get sensor and images identifiers.
  int sensorId = atoi(argv[2]);
  int numImage0 = atoi(argv[3]);
  int numImage1 = atoi(argv[4]);

  /************************************************************************************
   *                                                                                  *
   *  Read depth images.                                                              *
   *                                                                                  *
   ************************************************************************************/
  MatrixXus image0, image1;
  char imageFilename[50];
  string baseFilename(argv[1]);
  FILE *file;
  baseFilename = baseFilename.substr(0, baseFilename.length()-4);
  baseFilename = baseFilename + "_rgbd_";

  sprintf(imageFilename, "%s%d_%05d_depth.pgm", &baseFilename[0], sensorId, numImage0);
  file = fopen(&imageFilename[0], "rb");
  if (!readPgm(image0, file)){
    cout << "Error while reading first depth image." << endl;
    exit(-1);
  }
  fclose(file);
  sprintf(imageFilename, "%s%d_%05d_depth.pgm", &baseFilename[0], sensorId, numImage1);
  file = fopen(&imageFilename[0], "rb");
  if (!readPgm(image1, file)){
    cout << "Error while reading second depth image." << endl;
    exit(-1);
  }
  fclose(file);

  // Get rows and columns for the input images.
  int rows = image0.rows();
  int cols = image0.cols();

  /************************************************************************************
   *                                                                                  *
   *  Compute 3D points from the depth images.                                        *
   *                                                                                  *
   ************************************************************************************/
  // Cast images to float type.
  MatrixXf depth0(rows, cols), depth1(rows, cols);
  img2depth(depth0, image0);
  img2depth(depth1, image1);

  // Create 3D point clouds with normals.
  Vector6fVector cloud0, cloud1;
  depth2cloud(cloud0, depth0, cameraMatrix);
  depth2cloud(cloud1, depth1, cameraMatrix);
  
  /************************************************************************************
   *                                                                                  *
   *  Compute normals and curvature of the 3D points.                                 *
   *                                                                                  *
   ************************************************************************************/
  // Create matrices of pointers.
  MatrixXf curvature0(rows, cols), curvature1(rows, cols);
  MatrixXf zBuffer(rows, cols);
  Vector6fPtrMatrix cloud0Ptr(rows, cols), cloud1Ptr(rows, cols);
  cloud2mat(cloud0Ptr,
            cloud0,
            Isometry3f::Identity(), cameraMatrix,
	    zBuffer);
  cloud2mat(cloud1Ptr,
            cloud1,
            Isometry3f::Identity(), cameraMatrix,
	    zBuffer);
 
  // Compute normals.
  float r = 0.1f;
  float d = 100.0f; 
  computeNormals(cloud0Ptr, curvature0, cameraMatrix, r, d);
  computeNormals(cloud1Ptr, curvature1, cameraMatrix, r, d);

  // Close file stream.
  ifG2O.close();

  /************************************************************************************
   *                                                                                  *
   *  Draw 3D points with normals.                                                    *
   *                                                                                  *
   ************************************************************************************/
  PWNQGLViewer viewer;
  viewer.setPoints(&cloud0);
  viewer.setWindowTitle("Viewer");

  // Make the viewer window visible on screen.
  viewer.show();

  // Run main loop.
  return application.exec();
    
  return 0;
}
