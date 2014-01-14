#include "voronoi_diagram.h"

using namespace std;
using namespace Eigen;


VoronoiDiagram* vd = 0;


void mouseEvent(int evt, int x, int y, int flags, void *param)
{
    if(evt == CV_EVENT_LBUTTONDOWN)
    {
        cout << "Coords: " << x << ", " << y << endl;
        cout << (*vd->_distmap)(x, y).distance() << endl;
        EdgeSet es = (*vd->_distmap)(x, y).edgeSet();
        for(EdgeSet::iterator it = es.begin(); it != es.end(); it++)
        {
            VoronoiEdge e = *it;
            cout << "(" << e.from()->position().x() << ", " << e.from()->position().y() << ") --> (" << e.to()->position().x() << ", " << e.to()->position().y() << ")" << endl;
        }
    }
}


int main(int argc, char** argv)
{
    if(argc < 2 || argc > 3)
    {
        cerr << "usage: " << argv[0] <<" <pgm map> " << endl;
        exit(-1);
    }

    ifstream is(argv[1]);
    if(!is)
    {
        cerr << "Could not open map file for reading." << endl;
        exit(-1);
    }

    int res = 2500; // res = 2500 for MIT dataset
//    int res = 100; // res = 100 for DIS basement
//    int dt = 256;
//    float dr = 0.125;
//    int max = 20;

//    vd = new VoronoiDiagram(is, res, 256, (float) 0.125);

    vd = new VoronoiDiagram(is, res);

    double a = vd->get_time();
    vd->loadPGM();
    double b = vd->get_time();
    cout << "IMAGE LOADED: " << b-a << endl;
    vd->queueFiller();
    double c = vd->get_time();
    cout << "FILLING QUEUE: " << c-b << endl;
    vd->distmap();
    vd->distmap2image();
    vd->checkStats();
    vd->savePGM("distance_map.pgm", vd->_drawableDistmap);

    double d = vd->get_time();
    vd->distmap2voronoi();
    double e = vd->get_time();
    cout << "VORO time: " << e-d << endl;
    cv::imwrite("voronoi.pgm", (*(vd->_voro)).t());
    vd->checkStats();
//    vd->checkQueue();
//    cout << "PRE FILTERING" << endl;
//    vd->filter();
//    cout << "POST FILTERING" << endl;
    vd->checkStats();
    double fc = vd->get_time();
    vd->diagram2graph();
    double fd = vd->get_time();
    cout << "DIAGRAM2GRAPH TIME: " << fd - fc << endl;
    vd->checkStats();
    vd->graph();
    cv::imwrite("graph.pgm", (*(vd->_graph)).t());


    /** LOOK-UP TABLE ANALYSIS. DEAD END */
//    vd->fillLookUpTable(dt, dr, max);
//    vd->proxySetter();

    cv::namedWindow("voronoi", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
    cv::namedWindow("graph", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
    cv::setMouseCallback("graph", mouseEvent, 0);

    //load and display an image
    cv::Mat img = cv::imread("voronoi.pgm", CV_LOAD_IMAGE_UNCHANGED);
    cv::imshow("voronoi", img);

    cv::Mat img2 = cv::imread("graph.pgm", CV_LOAD_IMAGE_UNCHANGED);
    cv::imshow("graph", img2);

    cv::waitKey(0);

    //cleaning up
    cv::destroyWindow("voronoi");
    cv::destroyWindow("graph");
    img.release();
    img2.release();

    exit(0);
}
