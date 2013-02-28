#include "map_gui.h"
#include "moc_map_gui.cpp"


using namespace std;



MapGUI::MapGUI(Vertices* verticesVector_, Edges* edgesVector_, QWidget *parent)
{
    setupUi(this);
    QObject::connect(pushButton, SIGNAL(clicked()), this, SLOT(loadGraph()));

    _vertices = verticesVector_;
    _edges = edgesVector_;
}


void MapGUI::loadGraph()
{
}
