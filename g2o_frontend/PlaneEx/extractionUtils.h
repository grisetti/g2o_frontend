#include <signal.h>
#include <ncurses.h>
#include "g2o/stuff/macros.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"
#include "g2o/types/slam3d/types_slam3d.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"


#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"
#include "g2o_frontend/sensor_data/rgbd_data.h"
#include <Eigen/Geometry>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>

#define fx_d 5.9421434211923247e+02
#define fy_d 5.9104053696870778e+02
#define cx_d 3.3930780975300314e+02
#define cy_d 2.4273913761751615e+02
#define k1_d -2.6386489753128833e-01
#define k2_d 9.9966832163729757e-01
#define p1_d -7.6275862143610667e-04
#define p2_d 5.0350940090814270e-03
#define k3_d -1.3053628089976321e+00


using namespace std;
using namespace g2o;
using namespace Slam3dAddons;
using namespace cv;
using namespace Eigen;

RGBDData imageOriginal;

Vector3d kinectRay(0,0,1);

//PLANE
//***********************************************
struct plane
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    Vec3f color;
    plane()
    {

        this->color[0]=((float)rand())/RAND_MAX;
        this->color[1]=((float)rand())/RAND_MAX;
        this->color[2]=((float)rand())/RAND_MAX;
    }
};



//VOXELIZATION
//***********************************************

struct voxelAcc
{
    Vec3f acc;
    int i;

    voxelAcc()
    {
        acc[0]=acc[1]=acc[2]=0;
        i=0;
    };

    void add(Vec3f & v)
    {
        i++;
        acc+=v;
    };

    Vec3f average() const
    {
        float f=1./i;
        return acc*f;
    }
};

struct something
{
    int i[3];
    bool operator < (const something & s) const
    {
        if(i[0]<s.i[0]) return true;
        if(i[0]==s.i[0] && i[1]<s.i[1]) return true;
        if(i[1]==s.i[1] && i[2]<s.i[2]) return true;
        return false;
    }
};
//***********************************************

typedef std::map<something, voxelAcc> accumulatorMap;

//***********************************************


//read from the image imageToProcess and fill the Vec3f Cloud
void create_cloud(std::vector<Vec3f> &Cloud,const Mat &imageToProcess,int maxDist)
{
    int rows=imageToProcess.rows;
    int cols=imageToProcess.cols;
    const unsigned short * dptra=imageToProcess.ptr<unsigned short>(0);
    for(int i = 0; i < rows; i++)
    {

        for(int j = 0; j < cols; j++)
        {
            unsigned short d = *dptra;
            if(d != 0)
            {

                Vec3f cloudElem;
                if(d<maxDist) //2000 is max distance
                {
                    cloudElem[0]=(double)((double)j-(double)cx_d)*(double)d/(double)fx_d;
                    cloudElem[1]=(double)((double)i-(double)cy_d)*(double)d/(double)fy_d;
                    cloudElem[2]=d;

                    Cloud.push_back(cloudElem);

                }
            }
            dptra++;
        }
    }
}


void voxelize(std::vector<Vec3f> &theCloud,float res)
{
    cout << "\t\t starting voxelization"<<endl;

    accumulatorMap accMap;
    float ires=1./res;


    for(int i=0;i<(int)theCloud.size();i++)
    {

        something s;
        s.i[0]=(int)(theCloud)[i][0]*ires;
        s.i[1]=(int)(theCloud)[i][1]*ires;
        s.i[2]=(int)(theCloud)[i][2]*ires;

        accumulatorMap::iterator it=accMap.find(s);
        if(it==accMap.end())
        {
            voxelAcc vac;
            vac.acc[0]=(theCloud)[i][0];
            vac.acc[1]=(theCloud)[i][1];
            vac.acc[2]=(theCloud)[i][2];
            vac.i=1;
            accMap.insert(make_pair(s,vac));
        }
        else
        {
            voxelAcc & vac = it->second;
            vac.add((theCloud)[i]);
        }

    }



    theCloud.clear();
    for(accumulatorMap::iterator it=accMap.begin();it!=accMap.end();it++ )
    {
        voxelAcc &aMap=it->second;
        Vec3f tmp = aMap.average();
        theCloud.push_back(tmp);
    }
    cout << "\t\t voxelized in "<<theCloud.size()<<" points"<<endl;
    cout << "\t\t Voxelization finished..."<<endl;
}
