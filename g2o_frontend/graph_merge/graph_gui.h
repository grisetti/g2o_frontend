#ifndef GraphGUI_H
#define GraphGUI_H

//#include <boost/signal.hpp>
//#include <boost/bind.hpp>

#include <QDialog>
#include <QMainWindow>
#include <QThread>
#include <QLabel>
#include <QTime>
#include <QImage>
#include <QLineEdit>
#include <QStatusBar>

#include <unistd.h>
#include <iostream>
#include <stdio.h>

#include "ui_graph_gui.h"    //As Maurilio would say, remember me


class GraphGUI : public QMainWindow, public Ui::GraphGUI
{
    Q_OBJECT

public:
    explicit GraphGUI(QWidget *parent = 0);
    ~GraphGUI();

    inline GraphViewer* graphViewer() { return this->_ui->graph_viewer; }
//        inline GraphViewer* graphViewer() { return this->graph_viewer; }

//private:
    Ui::GraphGUI* _ui;

public slots:
    void mergeGraphs();
    void oneStep();

};
#endif // GraphGUI_H
