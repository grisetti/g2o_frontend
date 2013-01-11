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
// 		float alpha0 = 0.1;
// 		float rho0 = 2.;
// 		Vertex* v0 = new Vertex(0, rho0, alpha0);
// 		cout << "v0: " << v0->x() << " , " << v0->y() << endl;
// 		float alpha1 = 2*alpha0;
// 		float rho1 = rho0/cos(alpha0);
// 		Vertex* v1 = new Vertex(1, rho1, alpha1);
// 		cout << "v1: " << v1->x() << " , " << v1->y() << endl;
// 		float alpha2 = 3*alpha0;
// 		float rho2 = rho0/cos(alpha1);
// 		Vertex* v2 = new Vertex(2, rho2, alpha2);
// 		
// 		float alpha3 = 4*alpha0;
// 		float rho3 = rho0/cos(alpha2);
// 		Vertex* v3 = new Vertex(3, rho3, alpha3);
// 		
// 		float alpha4 = 5*alpha0;
// 		float rho4 = rho0/cos(alpha3);
// 		Vertex* v4 = new Vertex(4, rho4, alpha4);
// 		
// 		float alpha5 = 6*alpha0;
// 		float rho5 = rho0/cos(alpha4);
// 		Vertex* v5 = new Vertex(5, rho5, alpha5);
// 		
// 		float alpha6 = 7*alpha0;
// 		float rho6 = rho0/cos(alpha5);
// 		Vertex* v6 = new Vertex(6, rho6, alpha6);
// 		
// 		float alpha7 = 8*alpha0;
// 		float rho7 = rho0/cos(alpha6);
// 		Vertex* v7 = new Vertex(7, rho7, alpha7);
// 		
// 		map_vertex.insert(pair<int, Vertex*>(0,v0));
// 		map_vertex.insert(pair<int, Vertex*>(1,v1));
// 		map_vertex.insert(pair<int, Vertex*>(2,v2));
// 		map_vertex.insert(pair<int, Vertex*>(3,v3));
// 		map_vertex.insert(pair<int, Vertex*>(4,v4));
// 		map_vertex.insert(pair<int, Vertex*>(5,v5));
// 		map_vertex.insert(pair<int, Vertex*>(6,v6));
// 		map_vertex.insert(pair<int, Vertex*>(7,v7));
		
		 
		cout << "map_vertices dim: " << map_vertex.size() << endl;
		Edges edges; //edges ia a vector of Edge
		Edges satEdges;
		
		
		edgeExtr->purgeBoundaryVertices(map_vertex, satEdges, maxRangeFinderLimit);
		edgeExtr->step(map_vertex, edges);
		//edgeExtr->mergeCollinearSegments(edges);
		
		cout << "*** End of extraction: the number of edges is " << edges.size() << " ***" << endl;
		
		if (edges.size() == 0) {
			this->viewer->lineFound = false;
			cout << "WARNING! The algorithm you used haven't found any lines.." << endl;
		}
		else {
			
			//create the linecontainer structure from the edges inside the vector of edges
			line3d l;
			for (int i = 0; i < (int)edges.size(); i++) {
				
				cout << "creating linecontainer: " << endl;
				
				l.lineVertices.push_back(Eigen::Vector2d(edges[i]->getVertexFrom().x(),
																								 edges[i]->getVertexFrom().y()));
				l.lineVertices.push_back(Eigen::Vector2d(edges[i]->getVertexTo().x(),
																								 edges[i]->getVertexTo().y()));
				
				cout << "***NEW LINE FOUND!*** " << edges[i]->getVertexFrom().x() << ", " << edges[i]->getVertexFrom().y() << "to "  << edges[i]->getVertexTo().x() << ", " << edges[i]->getVertexTo().y() << endl;
				this->lc.push_back(l);
				l.lineVertices.clear();
			}
			cout << "Drawning is starting....." << endl;
			this->viewer->lineFound = true;
			this->viewer->lineContainer = &(this->lc);
		}
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


