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

#include <fstream>

#define MIN_POINTS_IN_LINES 10
#define SPLIT_THRESHOLD 0.05*0.05
#define CLUSTER_SQUARED_DISTANCE 0.3*0.3

using namespace std;

const std::string noType = "No Line Extraction Algorithm choosen yet.";
const std::string ransacType = "Ransac algorithm.";
const std::string splitMergeType = "Split & Merge algorithm.";



void ViewerGUI::updateVal1(int val)
{
	cout << "Min point in lines: " << slider1value << endl;
	lineExtractor->_minPointsInLine = val;
	this->lineExtraction();
	this->viewer->updateGL();
}

void ViewerGUI::updateVal2(int val)
{
	cout << "Split threshold: " << slider2value << endl;
	float dist = (float)val * 0.0005;
	if (dist > 0.f) {
		lineExtractor->_splitThreshold = dist*dist;
	}
	else cerr << "Split threshold is 0! Try to move the slider a bit more.." << endl;
	
	this->lineExtraction();
	this->viewer->updateGL();
}

void ViewerGUI::updateVal3(int val)
{
	cout << "Cluster squared distance: " << slider3value << endl;
	float dist = (float)val * 0.005;
	if (dist > 0.f) {
		clusterer->_squaredDistance = dist*dist;
	}
	else cerr << "Distance for clustering the points is 0! Try to move the slider a bit more.." << endl;
	
	this->lineExtraction();
	this->viewer->updateGL();
}

void ViewerGUI::showOriginal()
{
  cout << "#----------------------------------#" << endl;
  cout << "Original LaserRobotData points." << endl;
  cout << "#----------------------------------#" << endl;
	
  this->viewer->lineFound = false;

	LaserDataVector::iterator it = ldvector->begin();
	ld = *(it+numIteration);
	/*this->viewer->setDataPointer(&(ldvector->front().second)); //  this->viewer->setDataPointer(originalPoints);*/
	this->viewer->setDataPointer(&(ld.second));
	this->viewer->updateGL();
}
#if 0
					ofstream osp1PrimaC("points1PrimaC.dat");
					ofstream osp1("points1.dat");
					ofstream os1("lines1.dat");
#endif
void ViewerGUI::lineExtraction()
{
  if (algotype == noType) {
    cout << "WARNING! No type of algorithm extraction choosen yet. Please select one of the available options." << endl;
    return;
  }
	else
	{
		this->lc.clear();
		int minPointsCluster = 10;
		Vector2fVector linePoints;
		LaserDataVector::iterator it = ldvector->begin();
		ld = *(it+numIteration);
		
		if (algotype == splitMergeType){
			cout << "#----------------------------------------------#" << endl;
			cout << "Starting extraction with: " << algotype << endl;
			cout << "#------------------------------------------------#" << endl;			
			
			Vector2fVector cartesianPoints = ld.second;
#if 0				
				for (size_t i =0; i<cartesianPoints.size(); i++){
					osp1PrimaC << cartesianPoints[i].transpose() << endl;
				}
				osp1PrimaC.flush();
#endif
			clusterer->setPoints(cartesianPoints);
			clusterer->compute();
			cerr << "I found " << clusterer->numClusters() << " clusters in the pool" << endl;
			
			for (int i =0; i<clusterer->numClusters(); i++){
				const Point2DClusterer::Cluster& cluster = clusterer->cluster(i);
				int clusterSize = cluster.second - cluster.first;
				cerr << "processing cluster: " << i << " npoints: " << cluster.second - cluster.first;
				
				if (clusterSize < minPointsCluster) {
					cerr << " IGNORE" << endl;
					continue;
				}
				cerr << " ACCEPT" << endl;
				Vector2fVector::const_iterator first = cartesianPoints.begin() + cluster.first;
				Vector2fVector::const_iterator last = cartesianPoints.begin() + cluster.second;
				Vector2fVector currentPoints(first, last);
				
#if 0				
				for (size_t i =0; i<currentPoints.size(); i++){
					osp1 << currentPoints[i].transpose() << endl;
				}
				osp1.flush();
#endif
				
				lineExtractor->setPoints(currentPoints);
				lineExtractor->compute();
				
				cout << "*** End of extraction: the number of lines found is " << lineExtractor->lines().size() << " ***" << endl;
				
				const Line2DExtractor::IntLineMap& linesMap = lineExtractor->lines();
				for (Line2DExtractor::IntLineMap::const_iterator it=linesMap.begin(); it!=linesMap.end(); it++) {
					
// 					cout << "creating linecontainer: " << endl;
					
					const Line2D& line = it->second;
					const Vector2f& p0 = lineExtractor->points()[line.p0Index];
					const Vector2f& p1 = lineExtractor->points()[line.p1Index];
					linePoints.push_back(Eigen::Vector2f(p0.x(), p0.y()));					
					linePoints.push_back(Eigen::Vector2f(p1.x(), p1.y()));
					
#if 0
// 					os1 << p0.transpose() << endl;
// 					os1 << p1.transpose() << endl;
					// or
					os1 << linePoints[0].x() << " " << linePoints[0].y() << endl;
					os1 << linePoints[1].x() << " " << linePoints[1].y() << endl;
					os1 << endl;
					os1 << endl;
#endif
// 					cout << "***NEW LINE FOUND!*** " << p0.x() << ", " << p0.y() << " to "  << p1.x() << ", " << p1.y() << endl;
					this->lc.push_back(linePoints);
					linePoints.clear();
				}
#if 0
					os.flush();
#endif
			}
			cout << "Drawning is starting....." << endl;
			this->viewer->lineFound = true;			
		}
		else if (algotype == ransacType) 
		{
			cout << "#----------------------------------------------#" << endl;
			cout << "Starting extraction with: " << algotype << endl;
			cout << "#------------------------------------------------#" << endl;
			LaserRobotData* laserData = ld.first;
// 		LaserRobotData* laserData = ldvector->front().first;
			double maxRangeFinderLimit = laserData->maxRange();
			float angularStep = laserData->fov() / laserData->ranges().size();
			float angle = laserData->firstBeamAngle();
			
			//creating a map of all the vertices corrisponding the laserData ranges    
			VerticesMap map_vertex;
			for (size_t i = 0; i < laserData->ranges().size(); ++i) {
			  cout << "saving ranges into map_vertices: " << laserData->ranges().size() <<  endl;
			  const float& rho = laserData->ranges()[i];
			  const float& theta = angle;
				
			  Vertex* v = new Vertex(i, rho, theta);
			  map_vertex.insert(pair<int,Vertex*>(i,v));
				
			  angle+=angularStep;
			}
			
// 			cout << "map_vertices dim: " << map_vertex.size() << endl;
			Edges edges; //edges ia a vector of Edge
			Edges satEdges;
			
			
			edgeExtr->purgeBoundaryVertices(map_vertex, satEdges, maxRangeFinderLimit);
			edgeExtr->step(map_vertex, edges);
			//edgeExtr->mergeCollinearSegments(edges);
			
			cout << "*** End of extraction: the number of lines found is " << edges.size() << " ***" << endl;
			
			if (edges.size() == 0) {
			  this->viewer->lineFound = false;
			  cout << "WARNING! The algorithm you used haven't found any lines.." << endl;
			}
			else {
				
			  //create the linecontainer structure from the edges inside the vector of edges
			 
			  for (int i = 0; i < (int)edges.size(); i++) {
					
// 				cout << "creating linecontainer: " << endl;
					
				linePoints.push_back(Eigen::Vector2f(edges[i]->getVertexFrom().x(),
																							edges[i]->getVertexFrom().y()));
				linePoints.push_back(Eigen::Vector2f(edges[i]->getVertexTo().x(),
																							edges[i]->getVertexTo().y()));
					
// 				cout << "***NEW LINE FOUND!*** " << edges[i]->getVertexFrom().x() << ", " << edges[i]->getVertexFrom().y() << " to "  << edges[i]->getVertexTo().x() << ", " << edges[i]->getVertexTo().y() << endl;
				this->lc.push_back(linePoints);
				linePoints.clear();
			  }
			cout << "Drawning is starting....." << endl;
			this->viewer->lineFound = true;
			}
		}
		this->viewer->lContainer = &(this->lc);
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
		this->lineExtractor = new Line2DExtractor();
		this->lineExtractor->_minPointsInLine = MIN_POINTS_IN_LINES;
		this->lineExtractor->_splitThreshold = SPLIT_THRESHOLD;
		this->clusterer = new Point2DClusterer();
		this->clusterer->_squaredDistance = CLUSTER_SQUARED_DISTANCE;
//     this->edgeExtr = new SplitMergeEE(); //needed for the old/complicated algorithm
  }
}

void ViewerGUI::setIdIteration()
{
	numIteration+=1;
	this->showOriginal();
}


ViewerGUI::ViewerGUI(LaserDataVector* theLdVector, QWidget* parent)
{
  setupUi(this);
  QObject::connect(horizontalSlider, SIGNAL(sliderMoved(int)), this, SLOT(updateVal1(int)));
  QObject::connect(horizontalSlider_2, SIGNAL(sliderMoved(int)), this, SLOT(updateVal2(int)));
  QObject::connect(horizontalSlider_3, SIGNAL(sliderMoved(int)), this, SLOT(updateVal3(int)));
  QObject::connect(pushButton, SIGNAL(clicked()), this, SLOT(lineExtraction()));
  QObject::connect(pushButton_2, SIGNAL(clicked()), this, SLOT(showOriginal()));	
  QObject::connect(pushButton_3, SIGNAL(clicked()), this, SLOT(setIdIteration()));
  QObject::connect(checkBox, SIGNAL(clicked(bool)),this, SLOT(setAlgorithm()));
  QObject::connect(checkBox_2, SIGNAL(clicked(bool)),this, SLOT(setAlgorithm()));

	
  slider1value = 0;
  slider2value = 0;
  slider3value = 0;
  algotype = noType;
  edgeExtr = 0;
	lineExtractor = 0;
	clusterer = 0;
	ldvector = theLdVector;
	numIteration = 0;

}

// ViewerGUI::ViewerGUI(LaserRobotData* theLaserData/*, Vector2fVector* theLinesPoints*/, Vector2fVector* theOriginalPoints, QWidget* parent)
// {
//   setupUi(this);
//   QObject::connect(horizontalSlider, SIGNAL(sliderMoved(int)), this, SLOT(updateVal1(int)));
//   QObject::connect(horizontalSlider_2, SIGNAL(sliderMoved(int)), this, SLOT(updateVal2(int)));
//   QObject::connect(horizontalSlider_3, SIGNAL(sliderMoved(int)), this, SLOT(updateVal3(int)));
//   QObject::connect(pushButton, SIGNAL(clicked()), this, SLOT(lineExtraction()));
//   QObject::connect(pushButton_2, SIGNAL(clicked()), this, SLOT(showOriginal()));
//   QObject::connect(checkBox, SIGNAL(clicked(bool)),this, SLOT(setAlgorithm()));
//   QObject::connect(checkBox_2, SIGNAL(clicked(bool)),this, SLOT(setAlgorithm()));
// 	
//   slider1value = 0;
//   slider2value = 0;
//   slider3value = 0;
//   algotype = noType;
//   edgeExtr = 0;
// 	lineExtractor = 0;
// 	clusterer = 0;
//   laserData = theLaserData;
// //   linesFoundPoints = theLinesPoints;
//   originalPoints = theOriginalPoints;
// }