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

    VoronoiDiagram(std::istream&, int);
    ~VoronoiDiagram();

    /** UNUSED CONSTRUCTOR. WILL BE DELETED*/
//    VoronoiDiagram(std::istream&, int, int, float);

    inline double get_time()
    {
        struct timeval ts;
        gettimeofday(&ts,0);
        return ts.tv_sec + ts.tv_usec*1e-6;
    }

    void graph();
    void checkQueue();
    void checkStats();

    void distmap();
    void distmap2image();
    void distmap2voronoi();

    void diagram2graph();

    void filter();

    void queueFiller();

    /** UNUSED FUNCTIONS. THEY WILL BE DELETED*/
//    void fillLookUpTable(int dTh, float dRho, int maxDist);
//    void testLookUpTable(int dTh, float dRho, int maxDist);
//    void titSeeker(const Eigen::Vector2i& mouse_coords);
//    void proxySetter();

    void loadPGM();
    void savePGM(const char *filename, const Eigen::MatrixXf& image_);


    int _squaredResolution;
    std::istream& _file;
    /** UNUSED PARAMETERS. WILL BE DELETED*/
//    int _thetaRes;
//    float _rhoRes;
//    LookUpTable* _lut;
//    Eigen::MatrixXf* _tit;

    VertexMap _vMap;
    VertexMap _candidates;

    Eigen::MatrixXf _drawableDistmap;

    cv::Mat* _voro;
    cv::Mat* _graph;
    DistanceMap* _distmap;
    VoronoiQueue* _vQueue;
};
#endif // VORONOI_DIAGRAM_H
