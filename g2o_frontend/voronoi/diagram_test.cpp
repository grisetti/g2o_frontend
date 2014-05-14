#include "voronoi_diagram.h"
#include "simple_timer.h"


using namespace std;
using namespace Eigen;


void plotGraph(cv::Mat &img, vector<cv::Point2f> &nodes, vector< vector<int> > &edges,
               bool draw_edges = true, float scale = 1.0f)
{
    for(size_t i = 0; i < nodes.size(); i++)
    {
        cv::Point p1(cvRound(scale*nodes[i].x), cvRound(scale*nodes[i].y));
        cv::circle(img, p1, 2, cv::Scalar(0,0,255), 2);
        if(draw_edges)
        {
            vector<int> &neighbors = edges[i];
            for(size_t j = 0; j < neighbors.size(); j++)
            {
                cv::Point p2(cvRound(scale*nodes[neighbors[j]].x), cvRound(scale*nodes[neighbors[j]].y));
                cv::line(img, p1, p2, cv::Scalar(255,0,0));
            }
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

    //    int res = 2500; // res = 2500 for MIT dataset
    int res = 100; // res = 100 for DIS basement

    cv::Mat input = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    if(!input.data)
    {
        cerr << "Could not open map file for reading." << endl;
        exit(-1);
    }

    VoronoiDiagram* vd = new VoronoiDiagram(input, res);
    SimpleTimer timer;

    timer.reset();
    vd->distmapExtraction();
    cout << "Time distmapExtraction: " << (unsigned long) timer.elapsedTimeUs() << endl;

    vd->distmap2image();
    vd->savePGM("distance_map.pgm", vd->_drawableDistmap);

    timer.reset();
    vd->voronoiExtraction();
    cout << "Time voronoiExtraction: " << (unsigned long) timer.elapsedTimeUs() << endl;

    cv::imwrite("voronoi.pgm", (*(vd->_voro)));

    cv::namedWindow("voronoi", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
    //    cv::setMouseCallback("graph", mouseEvent, 0);

    //load and display an image
    cv::Mat img = cv::imread("voronoi.pgm", CV_LOAD_IMAGE_UNCHANGED);
    cv::imshow("voronoi", img);

    timer.reset();
    vd->morphThinning();
    cout << "Time morphThinning: " << (unsigned long)timer.elapsedTimeUs() << endl;

//    std::vector<cv::Point2f> nodes;
//    std::vector< std::vector<int> > edges;

//    timer.reset();
//    vd->graphExtraction(nodes, edges, true, 3);
//    cout << "Time graphExtraction: " << (unsigned long) timer.elapsedTimeUs() << endl;

//    voronoi = cv::Mat(vd->_skeleton.size(), CV_8UC3);
//    cv::cvtColor(vd->_skeleton, voronoi, CV_GRAY2BGR);
//    plotGraph(voronoi, nodes, edges, true);
    cv::imshow("src", img);
    cv::imshow("skeleton", vd->_skeleton);
    cv::imwrite("skeleton.pgm", vd->_skeleton);

    vd->skeleton2vmap();

    vd->denseGraphExtraction();
    vd->createObservations();

    ofstream os("prova_grafo.g2o");
    vd->save2g2o(os);
    cout << "Graph written" << endl;

    cv::waitKey(0);
    cv::destroyAllWindows();
    img.release();

    exit(0);
}
