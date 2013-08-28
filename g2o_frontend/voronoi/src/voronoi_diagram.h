#ifndef VORONOI_DIAGRAM_H
#define VORONOI_DIAGRAM_H

#include <Eigen/Core>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <deque>
#include <fstream>
#include <iostream>
#include <sys/time.h>
#include <vector>

#include "voronoi_vertex.h"


typedef Eigen::Matrix<Eigen::Vector2f, Eigen::Dynamic, Eigen::Dynamic> LookUpTable;

class VoronoiDiagram
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VoronoiDiagram(std::istream&, int, int, float);
    ~VoronoiDiagram();

    inline double get_time()
    {
        struct timeval ts;
        gettimeofday(&ts,0);
        return ts.tv_sec + ts.tv_usec*1e-6;
    }


    void titSeeker(const Eigen::Vector2i& mouse_coords);
    void proxySetter();
    void spikeFinder();

    void pruner();

    void newQueueFiller();
    void queueFiller();

    void newDistmap();
    void distmap();

    void distmap2image();
    void distmap2voronoi();

    void eroded2eigen();

    void fillLookUpTable(int dTh, float dRho, int maxDist);
    void testLookUpTable(int dTh, float dRho, int maxDist);

    void cvmat2eigenmat();

    void vmap2eigenmat();
//    void eigenmat2cvmat(cv::Mat& out, const Eigen::MatrixXf& in);

    void newLoad();
    void loadPGM();
    void savePGM(const char *filename, const Eigen::MatrixXf& image_);

    int _squaredResolution;
    int _thetaRes;
    float _rhoRes;

    VertexMap _vMap;

    Eigen::MatrixXf _drawableDistmap;
    Eigen::MatrixXf _drawableVoromap;
    Eigen::MatrixXf _drawableEroded;
    Eigen::MatrixXf _testVMap;

    Eigen::MatrixXi _input;
    Eigen::MatrixXi _pmat;
    PositionQueue* _posqueue;


    VoronoiVertex** _dmap;

    cv::Mat* _voro;
    DistanceMap* _distmap;
    DistanceQueue* _distqueue;
    LookUpTable* _lut;
    Eigen::MatrixXf* _tit;

    std::istream& _file;
};
#endif // VORONOI_DIAGRAM_H
