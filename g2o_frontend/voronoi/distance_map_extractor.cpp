#include "voronoi_diagram.h"

using namespace std;


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

    /*PARAMETER USED FOR THE VORONOI EXTRACTION, USELESS IN THIS CASE*/
    int res = 100;

//    VoronoiDiagram* vd = new VoronoiDiagram(is, res, 256, (float) 0.125);
    VoronoiDiagram* vd = new VoronoiDiagram(is, res);

    double a = vd->get_time();
    vd->loadPGM();
    double b = vd->get_time();
    cout << "IMAGE LOADED: " << b-a << endl;
    vd->queueFiller();
    double c = vd->get_time();
    cout << "FILLING QUEUE: " << c-b << endl;
    vd->distmap();
    vd->distmap2image();
    vd->savePGM("distance_map.pgm", vd->_drawableDistmap);

    exit(0);
}
