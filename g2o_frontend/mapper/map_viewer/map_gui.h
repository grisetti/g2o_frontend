#ifndef MAPGUI_H_
#define MAPGUI_H_

#include "interface.h"
#include <fstream>
#include <Qt/qapplication.h>

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"


typedef std::vector<g2o::VertexSE2*> Vertices;
typedef std::vector<g2o::EdgeSE2*> Edges;


class MapGUI : public QMainWindow, public Ui::MainWindow
{
    Q_OBJECT

public:
    MapGUI(Vertices* verticesVector_, Edges* edgesVector_, QWidget *parent = 0);

		Vertices* _vertices;
		Edges* _edges;


public slots:
        void loadGraph();
};

#endif // MAPGUI_H_
