#include <Eigen/Geometry>
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
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"
#include "g2o_frontend/data/point_cloud_data.h"
#include "g2o_frontend/sensor_data/rgbd_data.h"

using namespace Eigen;
using namespace g2o;


void printVector4dAsRow(Vector4d &vector,int terminatore=0)
{
    if(terminatore==0)
    {
        cout << vector.coeff(0) << "\t"
             << vector.coeff(1) << "\t"
             << vector.coeff(2) << "\t"
             << vector.coeff(3);
    }
    else
    {
        cout << vector.coeff(0) << "\t"
             << vector.coeff(1) << "\t"
             << vector.coeff(2) << "\t"
             << vector.coeff(3)<<endl;

    }
}

void printVector6dAsRow(Vector6d &vector,int terminatore=0)
{
    if(terminatore==0)
    {
        cout << vector.coeff(0) << "\t"
             << vector.coeff(1) << "\t"
             << vector.coeff(2) << "\t"
             << vector.coeff(3) << "\t"
             << vector.coeff(4) << "\t"
             << vector.coeff(5);
    }
    else
    {
        cout << vector.coeff(0) << "\t"
             << vector.coeff(1) << "\t"
             << vector.coeff(2) << "\t"
             << vector.coeff(3) << "\t"
             << vector.coeff(4) << "\t"
             << vector.coeff(5)<<endl;

    }
}
