
#include <qapplication.h>
#include <QDomElement>
#include <QtOpenGL/QGLWidget>
#include "opencv2/opencv.hpp"
#include <qobject.h>
#include "interface.h"
#include "CloudUtils.h"

using namespace std;
using namespace cv;

#define fx_d 5.9421434211923247e+02
#define fy_d 5.9104053696870778e+02
#define cx_d 3.3930780975300314e+02
#define cy_d 2.4273913761751615e+02
#define k1_d -2.6386489753128833e-01
#define k2_d 9.9966832163729757e-01
#define p1_d -7.6275862143610667e-04
#define p2_d 5.0350940090814270e-03
#define k3_d -1.3053628089976321e+00

std::vector<Vec3f> Cloud;
std::vector<Vec3f> OriginalCloud;
Mat tmp;
int allPoints;
int skippedPoints;
float voxelParam;
int numpointParam;
int treshParam;
int maxDistanceParam;

void reproject(Mat & image);

int main(int argc, char** argv)
{
    if(argc<=2)
    {
        cout << "usage: QTPlaneExtractor image.pgm [int]voxelParam [int]minimumPlanePointsParam [int]planeTreshParam [int]maxdistParam "<<endl;
        exit(0);
    }
    numpointParam = atoi(argv[3]);
    treshParam = atoi(argv[4]);
    voxelParam = atof(argv[2]);
    maxDistanceParam = atoi(argv[5]);

    tmp = imread(argv[1], CV_16UC1);
    int rows=tmp.rows;
    int cols=tmp.cols;
    //cout << rows << " " << cols<<endl;
    reproject(tmp);

    QApplication app(argc, argv);


    CloudUtils * utils = new CloudUtils(&Cloud,&OriginalCloud);
    ViewerInterface *dialog = new ViewerInterface(utils,&Cloud,&OriginalCloud);
    dialog->viewer->getDataPointer(&Cloud);
    dialog->show();
    return app.exec();
}

void reproject(Mat &image)
{
    int rows=image.rows;
    int cols=image.cols;
    unsigned short * dptra=tmp.ptr<unsigned short>(0);
    for(int i = 0; i < rows; i++)
    {

        for(int j = 0; j < cols; j++)
        {
            unsigned short d = *dptra;
            if(d != 0)
            {
                //float x=(j-cx_d)*d/fx_d;
                //float y=(i-cy_d)*d/fy_d;
                //float z=d;
                Vec3f p;
                if(d<maxDistanceParam)
                {
                p[0]=(double)((double)j-(double)cx_d)*(double)d/(double)fx_d;
                p[1]=(double)((double)i-(double)cy_d)*(double)d/(double)fy_d;
                p[2]=d;

                Cloud.push_back(p);
                OriginalCloud.push_back(p);
                }
            }
            dptra++;
        }
    }
    cout <<" Reprojection done!"<<endl;
}




