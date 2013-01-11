/*
 * viewerGUI.h
 *
 *  Created on: Dic 05, 2012
 *      Author: Martina
 */

#include "viewerGUI.h"
#include "moc_viewerGUI.cpp"
#include "RansacEE.h"
#include "SplitMergeEE.h"
#include "line_extraction2d.h"

using namespace std;
using namespace cv;

const std::string noType = "No Line Extraction Algorithm choosen yet.";
const std::string ransacType = "Ransac algorithm.";
const std::string splitMergeType = "Split & Merge algorithm.";

void ViewerGUI::updateVal1(int val)
{

}

void ViewerGUI::updateVal2(int val)
{

}

void ViewerGUI::updateVal3(int val)
{

}

void ViewerGUI::showOriginal()
{
  cout << "#----------------------------------#" << endl;
  cout << "Original LaserRobotData points." << endl;
  cout << "#----------------------------------#" << endl;
  this->viewer->lineFound = false;
  this->viewer->setDataPointer(originalPoints);
  this->viewer->updateGL();

}

void ViewerGUI::lineExtraction()
{
  if (algotype == noType) {
    cout << "WARNING! No type of algorithm extraction choosen yet. Please select one of the available options." << endl;
    return;
  }
	
  int minPoints = 10;
  if (algotype == ransacType || algotype == splitMergeType){
    cout << "#----------------------------------------------#" << endl;
    cout << "Starting extraction with: " << algotype << endl;
    cout << "#------------------------------------------------#" << endl;
	
    //TODO
    //pointslineUtilities::lineextraction()
    //project lines found
    this->lc.clear();
    double maxRangeFinderLimit = laserData->maxRange();
    float angularStep = laserData->fov() / laserData->ranges().size();
    float angle = laserData->firstBeamAngle();
    Point2DClusterer clusterer;
    clusterer._squaredDistance = 0.3*0.3;
    Vector2fVector cartesianPoints = laserData->floatCartesian();
    clusterer.setPoints(cartesianPoints);
    clusterer.compute();
    cerr << "I found " << clusterer.numClusters() << " clusters in the pool" << endl;
    Line2DExtractor lineExtractor;
    for (int i =0; i<clusterer.numClusters(); i++){
      const Point2DClusterer::Cluster& cluster = clusterer.cluster(i);
      int clusterSize = cluster.second - cluster.first;
      cerr << "processing cluster: " << i << " npoints: " << cluster.second - cluster.first;
      if (clusterSize < minPoints) {
	cerr << "IGNORE" << endl;
	continue;
      }
      cerr << "ACCEPT" << endl;
      Vector2fVector::const_iterator first = cartesianPoints.begin() + cluster.first;
      Vector2fVector::const_iterator last = cartesianPoints.begin() + cluster.second;
      Vector2fVector currentPoints(first, last);
      lineExtractor.setPoints(currentPoints);
      lineExtractor._minPointsInLine = 10;
      lineExtractor._splitThreshold = 0.03*0.03;
      lineExtractor.compute();
    }
    
    // //creating a map of all the vertices corrisponding the laserData ranges
    // Vector2fVector cartesianPoints = laserData->floatCartesian();
    
    // VerticesMap map_vertex;
    // Pointslineutilities
    // for (size_t i = 0; i < laserData->ranges().size(); ++i) {
    //   cout << "saving ranges into map_vertices: " << laserData->ranges().size() <<  endl;
    //   const float& rho = laserData->ranges()[i];
    //   const float& theta = angle;
			
    //   Vertex* v = new Vertex(i, rho, theta);
    //   map_vertex.insert(pair<int,Vertex*>(i,v));
			
    //   angle+=angularStep;
    // }
		 
    // cout << "map_vertices dim: " << map_vertex.size() << endl;
    // Edges edges; //edges ia a vector of Edge
    // Edges satEdges;
		
		
    // edgeExtr->purgeBoundaryVertices(map_vertex, satEdges, maxRangeFinderLimit);
    // edgeExtr->step(map_vertex, edges);
    // //edgeExtr->mergeCollinearSegments(edges);
		
    // cout << "*** End of extraction: the number of edges is " << edges.size() << " ***" << endl;
		
    // if (edges.size() == 0) {
    //   this->viewer->lineFound = false;
    //   cout << "WARNING! The algorithm you used haven't found any lines.." << endl;
    // }
    // else {
			
    //   //create the linecontainer structure from the edges inside the vector of edges
    //   line3d l;
    //   for (int i = 0; i < (int)edges.size(); i++) {
				
    // 	cout << "creating linecontainer: " << endl;
				
    // 	l.lineVertices.push_back(Eigen::Vector2d(edges[i]->getVertexFrom().x(),
    // 						 edges[i]->getVertexFrom().y()));
    // 	l.lineVertices.push_back(Eigen::Vector2d(edges[i]->getVertexTo().x(),
    // 						 edges[i]->getVertexTo().y()));
				
    // 	cout << "***NEW LINE FOUND!*** " << edges[i]->getVertexFrom().x() << ", " << edges[i]->getVertexFrom().y() << "to "  << edges[i]->getVertexTo().x() << ", " << edges[i]->getVertexTo().y() << endl;
    // 	this->lc.push_back(l);
    // 	l.lineVertices.clear();
    //   }
    //   cout << "Drawning is starting....." << endl;
    //   this->viewer->lineFound = true;
    //   this->viewer->lineContainer = &(this->lc);
    // }
    this->viewer->updateGL();
  }
}

void ViewerGUI::setAlgorithm()
{
  if(checkBox->isChecked()){
    algotype = ransacType;
    cout << "Choosing " << algotype << endl;
    this->edgeExtr = new RansacEE();
  }
  else if(checkBox_2->isChecked()){
    algotype = splitMergeType;
    cout << "Choosing " << algotype << endl;
    this->edgeExtr = new SplitMergeEE();
  }
}


ViewerGUI::ViewerGUI(LaserRobotData* theLaserData,LaserRobotData::Point2DVector* theLinesPoints, LaserRobotData::Point2DVector* theOriginalPoints, QWidget* parent)
{
  setupUi(this);
  QObject::connect(horizontalSlider, SIGNAL(sliderMoved(int)), this, SLOT(updateVal1(int)));
  QObject::connect(horizontalSlider_2, SIGNAL(sliderMoved(int)), this, SLOT(updateVal2(int)));
  QObject::connect(horizontalSlider_3, SIGNAL(sliderMoved(int)), this, SLOT(updateVal3(int)));
  QObject::connect(pushButton, SIGNAL(clicked()), this, SLOT(lineExtraction()));
  QObject::connect(pushButton_2, SIGNAL(clicked()), this, SLOT(showOriginal()));
  QObject::connect(checkBox, SIGNAL(clicked(bool)),this, SLOT(setAlgorithm()));
  QObject::connect(checkBox_2, SIGNAL(clicked(bool)),this, SLOT(setAlgorithm()));
	
  slider1value = 0;
  slider2value = 0;
  slider3value = 0;
  algotype = noType;
  edgeExtr = 0;
  laserData = theLaserData;
  linesPoints = theLinesPoints;
  originalPoints = theOriginalPoints;
  //utils=theUtils;
	
}


