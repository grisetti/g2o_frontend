#include "voronoi_diagram.h"
#include "simple_timer.h"


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


void plotGraph(cv::Mat &img, std::vector<cv::Point2f> &nodes,
               vector< vector<int> > &edges,
               bool draw_edges = true, float scale = 1.0f )
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

    vd = new VoronoiDiagram(input, res);
    double b = vd->get_time();
    vd->fillQueue();
    double c = vd->get_time();
    cout << "FILLING QUEUE: " << c-b << endl;
    vd->distmapExtraction();
    vd->distmap2image();
    vd->checkStats();
    vd->savePGM("distance_map.pgm", vd->_drawableDistmap);

    double d = vd->get_time();
    vd->voronoiExtraction();
    double e = vd->get_time();
    cout << "VORO time: " << e-d << endl;
    cv::imwrite("voronoi.pgm", (*(vd->_voro)));

    cv::namedWindow("voronoi", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
    cv::namedWindow("graph", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
    cv::setMouseCallback("graph", mouseEvent, 0);

    //load and display an image
    cv::Mat img = cv::imread("voronoi.pgm", CV_LOAD_IMAGE_UNCHANGED);
    cv::imshow("voronoi", img);

    cv::Mat img2 = cv::imread("graph.pgm", CV_LOAD_IMAGE_UNCHANGED);
    cv::imshow("graph", img2);





    cv::Mat input_img = img.clone(),
            dilated_input_img, skeleton, voronoi, voronoi2X;

    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,cv::Size( 7,7),cv::Point(3,3));
    cv::dilate(input_img ,dilated_input_img, element );

    SimpleTimer timer;

    timer.reset();
    vd->morphThinning( dilated_input_img, skeleton );
    std::cout<<"Time morphThinning : "<<(unsigned long)timer.elapsedTimeUs()<<std::endl;

    std::vector<cv::Point2f> nodes;
    std::vector< std::vector<int> > edges;

    timer.reset();
    vd->graphExtraction( skeleton, nodes, edges, true, 3 );
    std::cout<<"Time graphExtraction : "<<(unsigned long)timer.elapsedTimeUs()<<std::endl;

    voronoi = cv::Mat(skeleton.size(), CV_8UC3);
    cv::cvtColor(skeleton, voronoi, CV_GRAY2BGR);
    plotGraph( voronoi, nodes, edges,true );
    cv::resize(voronoi, voronoi2X, cv::Size(2*voronoi.cols, 2*voronoi.rows));
    plotGraph( voronoi2X, nodes, edges,true, 2.0f );

    cv::imshow("src",input_img);
    cv::imshow("dilated",dilated_input_img);
    cv::imshow("skeleton",skeleton);
    cv::imshow("voronoi",voronoi);
    cv::imshow("voronoi2X",voronoi2X);

    cv::waitKey(0);
    //    while(cv::waitKey() != 27);
    //cleaning up
    cv::destroyWindow("voronoi");
    cv::destroyWindow("graph");
    img.release();
    img2.release();

    exit(0);
}
