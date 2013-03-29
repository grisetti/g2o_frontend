#ifndef MAPGUI_H_
#define MAPGUI_H_

#include "interface.h"
#include <fstream>
#include <Qt/qapplication.h>

#include "g2o/core/optimizable_graph.h"



class MapGUI : public QMainWindow, public Ui::MainWindow
{
    Q_OBJECT

public:
    MapGUI();

public slots:
        void loadReferenceGraph();
        void loadCurrentGraph();
        void clearReferenceGraph();
        void clearCurrentGraph();
        void translateCurrent();
};

#endif // MAPGUI_H_
