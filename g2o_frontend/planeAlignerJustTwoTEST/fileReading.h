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

void fillPlanes(char* filename,int size,Vector4d* planes)
{
    std::ifstream p1("p1.dat");
    //leggo i piani
    for(int i =0;i<size;i++)
    {
        for(int j=0;j<4;j++)
        {
            p1 >> planes[i][j];
        }
    }
}


void fillTransform(char* filename,Isometry3d &t)
{
    std::ifstream fileReader(filename);
    t.setIdentity(); //security reasons
    Vector6d transform;
    for(int i=0;i<6;i++)
    {
        fileReader >>transform[i];
    }
    t=g2o::internal::fromVectorMQT(transform);

}
