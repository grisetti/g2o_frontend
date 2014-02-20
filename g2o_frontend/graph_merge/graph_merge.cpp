#include <QApplication>
#include <vector>
#include "drawable.h"
#include "graph.h"
#include "viewer.h"

#define MARKUSED(x) x=x


using namespace std;



int main(int argc, char ** argv)
{
    vector<Graph*> graphs;
    int c = 1;
    while(c < argc)
    {
        cerr << "loading file " << argv[c] << endl;
        ifstream is(argv[c]);
        Graph* g = new Graph;
        g->initGraph(is);
        graphs.push_back(g);
        c++;
    }

    QApplication* app = new QApplication(argc,argv);
    GraphViewer* viewer = new GraphViewer();
    viewer->show();
    float k = 0.35;
    for(size_t i = 0; i < graphs.size(); i++)
    {
        GraphDrawableItem* gdi = new GraphDrawableItem(graphs[i], viewer);
        gdi->_g = k * (i + 1);
        gdi->_b = k * (i + 1);
        MARKUSED(gdi);
    }

    while(viewer->isVisible())
    {
        app->processEvents();
        usleep(1000);
    }

}
