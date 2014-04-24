#include <iostream>
#include <fstream>

#include <QTime>

#include "graph_gui.h"


using namespace std;



GraphGUI::GraphGUI(QWidget *parent) : QMainWindow(parent), _ui(new Ui::GraphGUI)
{
    _ui->setupUi(this);
    QLabel* label = new QLabel(this);
    label->setText("this the status bar");
    this->_ui->statusBar->addPermanentWidget(label, 200);

    QObject::connect(_ui->pushButton, SIGNAL(clicked()), this, SLOT(mergeGraphs()));
    QObject::connect(_ui->pushButton_2, SIGNAL(clicked()), this, SLOT(oneStep()));
}


GraphGUI::~GraphGUI()
{
    delete _ui;
}


void GraphGUI::mergeGraphs()
{
    cout << "FULL GRAPHS MERGE" << endl;
    this->_ui->graph_viewer->updateGL();
}


void GraphGUI::oneStep()
{
    cout << "ONE STEP MERGE" << endl;
    this->_ui->graph_viewer->updateGL();
}
