#include "pwn_normals.h"
#include "pwn_cloud.h"
// to remove
//#include <pcl/features/integral_image_normal.h>
//#include <pcl/visualization/cloud_viewer.h>

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
  MatrixXus depth0us, depth1us;
  char imageFilename[50];
  string baseFilename(argv[1]);
  FILE *file;
  baseFilename = baseFilename.substr(0, baseFilename.length()-4);
  baseFilename = baseFilename + "_rgbd_";

  sprintf(imageFilename, "%s%d_%05d_depth.pgm", &baseFilename[0], sensorId, numImage0);
  file = fopen(imageFilename, "rb");
  if (!readPgm(depth0us, file)){
    cout << "Error while reading first depth image." << endl;
    exit(-1);
  }
  fclose(file);
  sprintf(imageFilename, "%s%d_%05d_depth.pgm", &baseFilename[0], sensorId, numImage1);
  file = fopen(&imageFilename[0], "rb");
  if (!readPgm(depth1us, file)){
    cout << "Error while reading second depth image." << endl;
    exit(-1);
  }
  fclose(file);

  // Get rows and columns for the input images.
  int rows = depth0us.rows();
  int cols = depth0us.cols();

  /************************************************************************************
   *                                                                                  *
   *  Compute 3D points from the depth images.                                        *
   *                                                                                  *
   ************************************************************************************/
  // Cast images to float type.
  MatrixXf depth0(rows, cols), depth1(rows, cols);
  depth0 = depth0us.cast<float>() / 1000.0f;
  depth1 = depth1us.cast<float>() / 1000.0f;

  // Create 3D point clouds with normals.
  Vector6fVector cloud0, cloud1;
  depth2cloud(cloud0, depth0, cameraMatrix);
  depth2cloud(cloud1, depth1, cameraMatrix);

  /************************************************************************************
   *                                                                                  *
   *  Compute normals and curvature of the 3D points.                                 *
   *                                                                                  *
   ************************************************************************************/
  MatrixXf curvature0, curvature1;
  float r = 0.03f;
  float d = 100.0f;
    /*computeNormals(cloud0, curvature0, cameraMatrix, r, d);
    computeNormals(cloud1, curvature1, cameraMatrix, r, d);
    
    // Save curvature images.
    MatrixXus curvature0Image(rows, cols), curvature1Image(rows, cols);
    curvature0Image = (curvature0 * 65535.0f).cast<int>();
    file = fopen("curvature0.pgm", "wb");
    writePgm(curvature0Image, file);
    fclose(file);
    file = fopen("curvature0.pgm", "wb");
    curvature1Image = (curvature1 * 65535.0f).cast<int>();
    writePgm(curvature1Image, file);
    fclose(file);

    // Close file stream.
    ifG2O.close();*/

    /**********************************************************
    *                   TESTING UNITS                         *
    *                                                         *
    ***********************************************************/
    /*Vector6fPtrMatrix pointsImage(rows, cols);
    Matrix6fPtrMatrix informationImage(rows, cols);
    Vector6fVector points;
    Vector6fVector pointsT;
    Matrix6fVector omegas;
    Vector6f initialGuess;
    points.reserve(rows * cols);
    omegas.reserve(rows * cols);
    Matrix6f omega = Matrix6f::Zero();
    omega(0, 0) = 1;
    omega(1, 1) = 1;
    omega(2, 2) = 1;
    omega(3, 3) = 1000;
    omega(4, 4) = 1000;
    omega(5, 5) = 1000;
    Vector6f _v;
    _v << 100.f, 150.f, -20.f, 0.2f, 0.7f, -0.1f;
//    _v << 0.2f, 0.1f, -0.1f, 0.5f, 0.5f, 0.5f;
    Isometry3f T = v2t(_v);
    for(int i = 0; i < normals0.rows(); i++)
    {
        for(int j = 0; j < normals0.cols(); j++)
        {
            Vector3f point = points0(i, j);
            if(point[0] == 0.0f && point[1] == 0.0f && point[2] == 0.0f)
                continue;
            Vector3f normal = normals0(i, j);
            Vector6f p;
            p << point[0], point[1], point[2], normal[0], normal[1], normal[2];
            Vector6f pT = remapPoint(T, p);
            points.push_back(p);
            pointsT.push_back(pT);
            omegas.push_back(omega);
        }
    }
    float error = 0;
    Isometry3f isoResult;

    Vector6f** _refPoints = new Vector6f*[points.size()];
    Vector6f** _currPoints = new Vector6f*[points.size()];
    Matrix6f** _omegas = new Matrix6f*[points.size()];
    for(size_t i = 0; i < points.size(); i++)
    {
        _refPoints[i] = &points[i];
        _currPoints[i] = &pointsT[i];
        _omegas[i] = &omegas[i];
    }

    isoResult = Isometry3f::Identity();

    for(int i = 0; i < 10; i++)
    {
        clock_t start = getMilliSecs();
        int inl = pwn_iteration(error, isoResult,
                                _refPoints,
                                _currPoints,
                                _omegas,
                                points.size(),
                                isoResult,
                                std::numeric_limits<float>::max(),
                                0);
        cout << "i: " << i << " " << inl << " " << error << " " << endl << t2v(isoResult*T) << endl;
        cout << "Time elapsed: " << getMilliSecs() - start << " ms" << endl;
        cout << "---------------------------------------------------------------" << endl;

    }


    MatrixXf zBuffer;
    cloud2img(pointsImage, informationImage,
              points, omegas,
              v2t(initialGuess), cameraMatrix,
              zBuffer);
    MatrixXi zBufferImage = (zBuffer * 65535.0f/5.0f).cast<int>();
    writePgm("zBuffer.pgm", zBufferImage);

    MatrixXi depth0Image = (depth0 * 65535.0f/5.0f).cast<int>();
    writePgm("depth0.pgm", depth0Image);*/

    // Create clouds with normals.
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//    for(int i = 0; i < normals0.rows(); i++)
//    {
//        for(int j = 0; j < normals0.cols(); j++)
//        {
//            Vector3f currentPoint = points0(i, j);
//            Vector3f currentNormal = normals0(i, j);
//            pcl::PointXYZ currentPointPCL(currentPoint[0], currentPoint[1], currentPoint[2]);
//            pcl::Normal currentNormalPCL(currentNormal[0], currentNormal[1], currentNormal[2]);
//            normals->push_back(currentNormalPCL);
//            cloud->push_back(currentPointPCL);
//        }
//    }

//    // Visualize normals.
//    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
//    viewer.setBackgroundColor(0.0, 0.0, 0.5);
//    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals);
//    viewer.addPointCloud<pcl::PointXYZ>(cloud);
//    while(!viewer.wasStopped())
//    {
//        viewer.spinOnce();
//    }

    return 0;
}
