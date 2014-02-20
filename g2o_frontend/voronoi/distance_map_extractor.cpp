#include "voronoi_diagram.h"


using namespace std;



int main(int argc, char** argv)
{
    if(argc < 2 || argc > 3)
    {
        cerr << "usage: " << argv[0] << " <pgm map> " << endl;
        exit(-1);
    }

    cv::Mat input = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    if(!input.data)
    {
        cerr << "Could not open map file for reading." << endl;
        exit(-1);
    }

//    int res = 2500; // res = 2500 for MIT dataset
    int res = 100; // res = 100 for DIS basement

    VoronoiDiagram* vd = new VoronoiDiagram(input, res);    double a = vd->get_time();
    vd->loadPGM();
    double b = vd->get_time();
    cout << "IMAGE LOADED: " << b-a << endl;
    vd->fillQueue();
    double c = vd->get_time();
    cout << "FILLING QUEUE: " << c-b << endl;
    vd->distmapExtraction();
    vd->distmap2image();
    vd->savePGM("distance_map.pgm", vd->_drawableDistmap);

    exit(0);
}
