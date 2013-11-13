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

#include "g2o_frontend/basemath/bm_se2.h"

#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam2d_addons/vertex_line2d.h"
#include "g2o/types/slam2d_addons/edge_line2d_pointxy.h"
#include "g2o/types/slam2d_addons/edge_se2_line2d.h"
#include "g2o_frontend/g2o_line_addons/g2o_line_addons.h"


#define MIN_POINTS_IN_LINES 10
#define SPLIT_THRESHOLD 0.03*0.03
#define CLUSTER_SQUARED_DISTANCE 0.3*0.3

using namespace std;

const std::string noType = "No Line Extraction Algorithm choosen yet.";
const std::string ransacType = "Ransac algorithm.";
const std::string splitMergeType = "Split & Merge algorithm.";



void ViewerGUI::updateVal1(int val)
{
	if (algotype == noType) {
		cout << "WARNING! No type of algorithm extraction choosen yet. Please select one of the available options." << endl;
		return;
	}
	cout << "Min point in lines: " << val << endl;
	lineExtractor->_minPointsInLine = val;
	this->lineExtraction();
	this->viewer->updateGL();
}

void ViewerGUI::updateVal2(int val)
{
	if (algotype == noType) {
		cout << "WARNING! No type of algorithm extraction choosen yet. Please select one of the available options." << endl;
		return;
	}
	cout << "Split threshold: " << val << endl;
	float dist = (float)val * 0.0005;
	if (dist > 0.f) {
		lineExtractor->_splitThreshold = dist*dist;
        cerr << "FUUUUCK  " << lineExtractor->_splitThreshold << endl;
	}
	else cerr << "Split threshold is 0! Try to move the slider a bit more.." << endl;
	
	this->lineExtraction();
	this->viewer->updateGL();
}

void ViewerGUI::updateVal3(int val)
{
	if (algotype == noType) {
		cout << "WARNING! No type of algorithm extraction choosen yet. Please select one of the available options." << endl;
		return;
	}
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

	//changing this....
// 	LaserDataVector::iterator it = ldvector->begin();
// 	ld = *(it+numIteration);
// 	this->viewer->setDataPointer(&(ld.second));
	//in this...
	VertexDataVector::iterator it = vldvector->begin();
    if(numIteration > (int)vldvector->size()-1)
    {
        cout << "!!!Graph Ended!!!" << endl;
        return;
    }
	vld = *(it+numIteration);
	// TODO changing from SE3:
// 	g2o::VertexSE3* v = vld.first;
// 	Eigen::Isometry3d T = v->estimate()*offset;
	//in SE2:
	g2o::VertexSE2* v = vld.first;
	Eigen::Isometry2d vEstimate;
	Vector3d ev = v->estimate().toVector();
	vEstimate.linear() = Rotation2Dd(ev.z()).matrix();
	vEstimate.translation() = ev.head<2>();
	Eigen::Isometry2d T = vEstimate*offset;
	
	for (size_t i = 0; i < vld.second.size(); i++) {
		Vector2d dpoints(vld.second[i].x(), vld.second[i].y());
		dpoints = T*dpoints;
		Vector2f lpoints(dpoints.x(),dpoints.y());
		vld.second[i] = lpoints;
	}	
	this->viewer->setDataPointer(&(vld.second));
	this->viewer->updateGL();
}

#if 1
// 					ofstream osp1PrimaC("points1PrimaC.dat");
// 					ofstream osp1("points1.dat");
//                    ofstream os("myCurrentLines.dat");
                    ofstream oslines("lines_likelihood.dat");
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
		this->lAdjacentVector.clear();
		
		int minPointsCluster = 10;
        Vector3fVector linePoints;
		//changing this..
// 		LaserDataVector::iterator it = ldvector->begin();
// 		ld = *(it+numIteration);	
		//in this..
		VertexDataVector::iterator it = vldvector->begin();
        if(numIteration > (int)vldvector->size()-1)
        {
            cout << "!!!Graph Ended!!!" << endl;
            return;
        }
		vld = *(it+numIteration);

		
		//TODO changing in SE2:
// 		g2o::VertexSE3* v = vld.first;
// 		Eigen::Isometry3d T = v->estimate()*offset;
		g2o::VertexSE2* v = vld.first;
		Eigen::Isometry2d vEstimate;
		Vector3d ev = v->estimate().toVector();
		vEstimate.linear() = Rotation2Dd(ev.z()).matrix();
		vEstimate.translation() = ev.head<2>();
// 		vEstimate.linear() = v->estimate().rotation().toRotationMatrix();
// 		vEstimate.translation() = v->estimate().translation();
		Eigen::Isometry2d T = vEstimate*offset;
		
// 		cerr << T.translation().transpose() << endl;
        cout << endl << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<" << endl;
		if (algotype == splitMergeType){
            cout << "#------------------------------------------------#" << endl;
			cout << "Starting extraction with: " << algotype << endl;
			cout << "#------------------------------------------------#" << endl;			
			
			//changing this...
			//Vector2fVector cartesianPoints = ld.second;
			//in this..
			Vector2fVector cartesianPoints = vld.second;
#if 0				
				for (size_t i =0; i<cartesianPoints.size(); i++){
					osp1PrimaC << cartesianPoints[i].transpose() << endl;
				}
				osp1PrimaC.flush();
#endif
			clusterer->setPoints(cartesianPoints);
			clusterer->compute();
			cout << "I found " << clusterer->numClusters() << " clusters in the pool" << endl;
			
            //for adjacent lines
			LinesAdjacent la;
			
			for (int i =0; i< clusterer->numClusters(); ++i){
				const Point2DClusterer::Cluster& cluster = clusterer->cluster(i);
				int clusterSize = cluster.second - cluster.first;
                cout << "!!!!!! Processing NEW CLUSTER: " << i << ", npoints: " << cluster.second - cluster.first;
				
				if (clusterSize < minPointsCluster) {
                    cout << ": CLUSTER IGNORED" << endl;
					continue;
				}
                cout << ": CLUSTER ACCEPTED" << endl;
				Vector2fVector::const_iterator first = cartesianPoints.begin() + cluster.first;
				Vector2fVector::const_iterator last = cartesianPoints.begin() + cluster.second + 1;
				Vector2fVector currentPoints(first, last);				
#if 0				
				for (size_t i =0; i<currentPoints.size(); i++){
					osp1 << currentPoints[i].transpose() << endl;
				}
				osp1.flush();
#endif
				lineExtractor->setPoints(currentPoints);
				lineExtractor->compute();
				cout << endl; 
				cout << endl;
				cout << "***********************************************************************************" << endl;
				cout << "*** End of extraction: the number of lines found is " << lineExtractor->lines().size() << " ***" << endl;
				cout << "***********************************************************************************" << endl;
				
				if(lineExtractor->lines().size() == 0)
					continue;
				const Line2DExtractor::IntLineMap& linesMap = lineExtractor->lines();
				for (Line2DExtractor::IntLineMap::const_iterator it=linesMap.begin(); it!=linesMap.end(); it++) {	
					const Line2D& line = it->second;					
					const Vector2f& p0 = lineExtractor->points()[line.p0Index];
                    const Vector2f& p1 = lineExtractor->points()[line.p1Index];

                    Vector2d p0_g(p0.x(), p0.y()); //remember the z field [0.f] if in SE3
					Vector2d p1_g(p1.x(), p1.y());
                    p0_g = T*p0_g;
                    p1_g = T*p1_g;

#if 0
					os << p0_g.transpose() << endl;
					os << p1_g.transpose() << endl;
					os << endl;
					os << endl;
					os.flush();
#endif

                    //creating structure to draw lines (considering the odometry (T) pf the robot)
                    //the third value is the default likelihood to be an extreme of the line
                    linePoints.push_back(Eigen::Vector3f(p0_g.x(), p0_g.y(), 0.5));
                    linePoints.push_back(Eigen::Vector3f(p1_g.x(), p1_g.y(), 0.5));
					
					this->lc.push_back(linePoints);
					linePoints.clear();
				}
				
				/** saving some information about the line extracted and its line adjacent, if exists **/	
				Line2DExtractor::IntLineMap::const_iterator bit=linesMap.begin();
                const Line2D& linea = bit->second;
                const Vector2f& lp0 = lineExtractor->points()[linea.p0Index];
                const Vector2f& lp1 = lineExtractor->points()[linea.p1Index];
                line2DwithPoints lps;
                lps.l = linea;
                lps.p0 = lp0;
                lps.p1 = lp1;
                la.push_back(lps);

                /// printing some info about the current extraction
                cout << endl;
                cout << "\tInfo about the lines of this cluster: " << endl;
                cout << endl;
                while(bit != linesMap.end()) {
                    const Line2D& line = bit->second;

                    linesInfoExtraction(bit, linesMap, currentPoints);
					
					Line2DExtractor::IntLineMap::const_iterator tmp = bit;
					if((++tmp) != linesMap.end()) {
                        const Line2D& lineRight = tmp->second;
                        const Vector2f& lrp0 = lineExtractor->points()[lineRight.p0Index];
                        const Vector2f& lrp1 = lineExtractor->points()[lineRight.p1Index];
                        line2DwithPoints lrps;
                        lrps.l = lineRight;
                        lrps.p0 = lrp0;
                        lrps.p1 = lrp1;
						if(line.p1Index == lineRight.p0Index)
						{
                            la.push_back(lrps);
//                            cout << "line with points " << la[la.size()-1].p0 << "and" << la[la.size()-1].p1 << endl;
							bit++;
						}
						else {
							lAdjacentVector.push_back(la);
							la.clear();
							bit++;
						}
					}
					else {
						lAdjacentVector.push_back(la);
						cout << "Number of lines adjacent set: " << lAdjacentVector.size() << endl;
						la.clear();
						bit++;
					}
				}
				cout << endl; 
				cout << endl;
			}
			cout << "Drawing is starting....." << endl;
			this->viewer->lineFound = true;			
		}
		else if (algotype == ransacType) 
		{
            cout << "#----------------------------------------------#" << endl;
            cout << "Starting extraction with: " << algotype << endl;
            cout << "#------------------------------------------------#" << endl;
            cout << "RANSAC METHODS IS NOT WORKING ANYMORE, CHOOSE THE OTHER OPTION! " << endl;

//            //changing this...
//// 			LaserRobotData* laserData = ld.first;
//            //in this...
//            g2o::OptimizableGraph::Data* d = v->userData();
//            LaserRobotData* laserData = dynamic_cast<LaserRobotData*>(d);
			
//            double maxRangeFinderLimit = laserData->maxRange();
//            float angularStep = laserData->fov() / laserData->ranges().size();
//            float angle = laserData->firstBeamAngle();
			
//            //creating a map of all the vertices corrisponding the laserData ranges
//            VerticesMap map_vertex;
//            for (size_t i = 0; i < laserData->ranges().size(); ++i) {
//              cout << "saving ranges into map_vertices: " << laserData->ranges().size() <<  endl;
//              const float& rho = laserData->ranges()[i];
//              const float& theta = angle;
				
//              Vertex* v = new Vertex(i, rho, theta);
//              map_vertex.insert(pair<int,Vertex*>(i,v));
				
//              angle+=angularStep;
//            }
//// 			cout << "map_vertices dim: " << map_vertex.size() << endl;
//            Edges edges; //edges ia a vector of Edge
//            Edges satEdges;
//            edgeExtr->purgeBoundaryVertices(map_vertex, satEdges, maxRangeFinderLimit);
//            edgeExtr->step(map_vertex, edges);
//            //edgeExtr->mergeCollinearSegments(edges);
			
//            cout << "*** End of extraction: the number of lines found is " << edges.size() << " ***" << endl;
			
//            if (edges.size() == 0) {
//              this->viewer->lineFound = false;
//              cout << "WARNING! The algorithm you used haven't found any lines.." << endl;
//            }
//            else {
//              //create the linecontainer structure from the edges inside the vector of edges
//              for (int i = 0; i < (int)edges.size(); i++) {
//                    linePoints.push_back(Eigen::Vector3f(edges[i]->getVertexFrom().x(),	edges[i]->getVertexFrom().y(), 0.5));
//                    linePoints.push_back(Eigen::Vector3f(edges[i]->getVertexTo().x(), edges[i]->getVertexTo().y(), 0.5));
//    // 				cout << "***NEW LINE FOUND!*** " << edges[i]->getVertexFrom().x() << ", " << edges[i]->getVertexFrom().y() << " to "  << edges[i]->getVertexTo().x() << ", " << edges[i]->getVertexTo().y() << endl;
//                    this->lc.push_back(linePoints);
//                    linePoints.clear();
//              }
//            cout << "Drawing is starting....." << endl;
//            this->viewer->lineFound = true;
//			}
        }

        /// finding the extreme likelihood, for each lines:

        Vector3fVector prev;
        prev.reserve(lc[0].size());
        prev.push_back(Vector3f(-1e9, -1e9, 0.5));//prev left [p0]
        prev.push_back(Vector3f(-1e9, -1e9, 0.5));//prev right [p1]

        float thetaP1, rhoP1, thetaPrev2, rhoPrev2 = -1e9;

        bool commonVertex = false;

        float dist_point = -1e9;
        float pdiV_common, pdiV_notCommon = -1;

        for (int i = 0; i < (int)lc.size(); i++)
        {
            Vector3fVector l = lc[i];
            Vector2d p1(l[0].x(), l[0].y());

            thetaP1 = atan2(p1.y(), p1.x());
            rhoP1 = p1.x()/sin(thetaP1);
            float dist_min = 0.10; //??? it depends on the laser(alpha angular step = 0.0004..)
            float thetaPointTh = 0.0086; //joint discontinuity:hokuyo angularstep is 0.0043..

            if(i==0)
            {
                commonVertex = false;
                dist_point = rhoP1;
            }
            else
            {
                //controlling if this line have a common vertex with the previous one(the same point or really closed one to each other..)
                prev = lc[i-1];
                Vector2d prev1d(prev[0].x(), prev[0].y());
                Vector2d prev2d(prev[1].x(), prev[1].y());
                thetaPrev2 = atan2(prev2d.y(), prev2d.x());
                rhoPrev2 = prev2d.x()/sin(thetaPrev2);
                dist_point = (rhoP1 < rhoPrev2 ? rhoP1 : rhoPrev2);
                cout << "dist nearest point: "  << dist_point << endl;
                //TODO
                float distPointTh = 4*rhoP1*sin(0.5*fabs(thetaP1-thetaPrev2)); //distance threshold (0.3)
                cout << "distPointTh " << distPointTh << endl;
                if((p1-prev2d).squaredNorm() <= distPointTh)
                {
                    commonVertex = true;
                    if((p1-prev2d).squaredNorm() == 0)
                    {
                        cerr << "[COMMON VERTEX]" << endl;
                        pdiV_common = 1;
                    }
                    else
                    {
                        cerr << "[ALMOST COMMON VERTEX]" << endl;
                        pdiV_common = 0.5/(1+(fabs(dist_point)-dist_min)) + 0.5;
                    }
                    lc[i][0].z() = pdiV_common;
                    lc[i-1][1].z() = pdiV_common;
                    cout << " - PdiV_common " << pdiV_common << " !!!!!!!!!!!must be equal to" << lc[i][0].z() << endl;
                }
                else
                    commonVertex = false;
            }
            if(!commonVertex)
            {
                cerr << "[NO COMMON VERTEX]" << endl;
                pdiV_notCommon = 0.5/(1+(fabs(dist_point)-dist_min)) + 0.5;
                cout << " - PdiV_NotCommon " << pdiV_notCommon << endl;
                if(i == 0)
                    lc[i][0].z() = pdiV_notCommon;
                else
                {
                    if(rhoPrev2 <= rhoP1)
                    {
                        lc[i-1][1].z() = pdiV_notCommon;
                        lc[i][0].z() = 0.25; //for now
                        cerr << "<<<<<<< this line has a possibly fake extreme point(LEFT): " << lc[i][0].transpose() << " with pdiV = " << lc[i][0].z() << endl;

                    }
                    else if(rhoP1 < rhoPrev2)
                    {
                        lc[i-1][1].z() = 0.25; //for now
                        lc[i][0].z() = pdiV_notCommon;
                        cerr << "<<<<<<< the previous line has a possibly fake extreme point(RIGHT): " << lc[i-1][1].transpose() << " with pdiV = " << lc[i-1][1].z() << endl;
                    }
                }
            }
            //save the current line points as previous
            prev = l;
        }

        //file for Matlab alignment
#if 1
        //in lc there are the line already trasformed with the odometry
        for(int i=0; i<lc.size(); i++)
        {
            Vector3fVector line = lc[i];
            oslines << "% Frame n " << numIteration << " with odometry [x y theta]" << endl;
            oslines << "% " << t2v_2d(T).transpose()/*.matrix()*/ << endl;
            oslines << line[0].x() << " " << line[0].y() << " " << line[0].z() << endl;
            oslines << line[1].x() << " " << line[1].y() << " " << line[1].z() << endl;
            oslines << endl;
            oslines << endl;
        }
        oslines.flush();
#endif
        this->viewer->lContainer = &(this->lc);
        this->viewer->updateGL();
	}
}

/**FOR EACH line
	* save coordinates of the point with max distance from the line;
	* save p0 as right extreme;
	* save p1 as left extreme;
	* save num points in line;
	* save maxerror (line.max dist(p));
**/
void ViewerGUI::linesInfoExtraction(Line2DExtractor::IntLineMap::const_iterator it, const Line2DExtractor::IntLineMap& linesMap, Vector2fVector& currentPoints) const
{
	const Line2D& line = it->second;					
	const Vector2f& p0 = lineExtractor->points()[line.p0Index];
	const Vector2f& p1 = lineExtractor->points()[line.p1Index];
	Vector2fVector::const_iterator f = currentPoints.begin() + line.p0Index;
	Vector2fVector::const_iterator l = currentPoints.begin() + line.p1Index + 1;
	Vector2fVector tmpPoints(f, l);
	int imax=-1;
	float maxDistance=-1;
	Vector2f& v = tmpPoints[0];
	for (size_t i = 0; i< tmpPoints.size(); i++){
		float d = line.squaredDistance(tmpPoints[i]);
		if (d>maxDistance){
				imax = i; 
				maxDistance = d;
				v = tmpPoints[imax];
		}
	}
	int numPointsInLine = (line.p1Index - line.p0Index) + 1; //tmpPoints.size();
    cout << "-->NEW LINE! Point with max distance from the line:"  <<  v.x() << " ," << v.y() << endl;
	cout << "------------------" << endl;
	cout << "Line extreme points: " << endl;
	cout << "\tLEFT: " << p0.x() << ", " << p0.y() << endl; cout << "\tRIGHT: " << p1.x() << ", " << p1.y() << endl;
	cout << "------------------" << endl;
	cout << "Number of points in line: " << numPointsInLine <<endl;
	cout << "------------------" << endl;
	cout << "Line max Error: " << maxDistance << endl;		
	cout << "------------------" << endl;
	Line2DExtractor::IntLineMap::const_iterator tmpit1 = it;
	if (it!=linesMap.begin()){
		const Line2D& lineLeft = (--tmpit1)->second;
		if (line.p0Index == lineLeft.p1Index){
			cout << "It has an adjacent line with in common its left vertex: " << p0.x() << ", " << p0.y() << endl;
		}
		else cout << "No adjacent line on the left" << endl;
	}
	Line2DExtractor::IntLineMap::const_iterator tmpit2 = it;
	if ((++tmpit2)!=linesMap.end()){
		const Line2D& lineRight = tmpit2->second;
		if (line.p1Index == lineRight.p0Index){
			cout << "It has an adjacent line with in common its right vertex: " << p1.x() << ", " << p1.y() << endl;
		}
		else cout << "No adjacent line on the right" << endl;
	}
	cout << "***********************************************************************************" << endl;
	cout << "***********************************************************************************" << endl;
	cout << "***********************************************************************************" << endl;
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
    if(numIteration <= (int)vldvector->size()-1)
        this->showOriginal();
    else {
        cout << "!!!Graph Ended!!!" << endl;
    }
}


Vector2d pointsToLine(const Vector2d& p1, const Vector2d& p2){
	Vector2d dp= p2-p1;
	Vector2d mp = (p2+p1)*.5;
	dp.normalize();
	Vector2d n(dp.y(), -dp.x());
	return Vector2d(atan2(n.y(), n.x()), n.dot(mp));
}

/** Compute the Line extraction for the entire graph adding vertices and edges related to the line data**/
void ViewerGUI::ComputeAll()
{
	if (algotype == noType) {
		cout << "WARNING! No type of algorithm extraction choosen yet. Please select one of the available options." << endl;
		return;
	}
	g2o::OptimizableGraph* graph;
	
	int count = 0;
	int step = 1;
	for (VertexDataVector::iterator it = vldvector->begin(); it != vldvector->end() && count < vldvector->size(); it+=step)
	{
		numIteration = count;
		this->lineExtraction();
		count+=step;

		VertexData vld = *(it);
		g2o::VertexSE2* v = vld.first;
		
		// we need T to transform points in gloabl frame
		Eigen::Isometry2d vEstimate;
		Vector3d ev = v->estimate().toVector();
		vEstimate.linear() = Rotation2Dd(ev.z()).matrix();
		vEstimate.translation() = ev.head<2>();
		Eigen::Isometry2d T = vEstimate*offset;
		Eigen::Isometry2d iT = T.inverse();
		
		graph = v->graph();
		int id1, id2, lid,  id1_oldId2, tmpId2 = -1;
        g2o::VertexExtremePointXY* vp1, *vp2, *vprev2, *vp1_oldVp2, *tmpVp2 = 0;
		g2o::VertexLine2D* vl;

        Vector3fVector prev;
        prev.reserve(lc[0].size());
        prev.push_back(Vector3f(-1e9, -1e9, 0.5));//prev left [p0]
        prev.push_back(Vector3f(-1e9, -1e9, 0.5));//prev right [p1]

        float thetaP1, rhoP1, thetaPrev2, rhoPrev2 = -1e9;

        bool commonVertex = false;

        float distPointTh = -1e9;
        double pdiV_common, pdiV_notCommon = -1;

        int id = (int)graph->vertices().size() - 1;
        cout << ">>>>>>>>>> NEW FRAME: saving new info in the graph: (id ultimo: " << id << ")" << endl;

        //for each line
		for (int i = 0; i < lc.size(); i++)
		{
            Vector3fVector l = lc[i];
            Vector2d p1(l[0].x(), l[0].y());
            Vector2d p2(l[1].x(), l[1].y());
            Vector2d lp1 = iT * p1;
            Vector2d lp2 = iT * p2;

            thetaP1 = atan2(p1.y(), p1.x());
            rhoP1 = p1.x()/sin(thetaP1);

            if(i==0)
                commonVertex = false;
            else
            {
                //controlling if this line have a common vertex with the previous one(the same point or really closed one to each other..)
//                Vector2d prev1d(prev[0].x(), prev[0].y());
                Vector2d prev2d(prev[1].x(), prev[1].y());

                thetaPrev2 = atan2(prev2d.y(), prev2d.x());
                rhoPrev2 = prev2d.x()/sin(thetaPrev2);

                distPointTh = 4*rhoP1*sin(0.5*fabs(thetaP1-thetaPrev2)); //distance threshold (0.3)
                if((p1-prev2d).squaredNorm() <= distPointTh)
                {
                    cerr << "[COMMON VERTEX]" << endl;
                    commonVertex = true;
                    pdiV_common = lc[i-1][1].z();
                    vprev2 = dynamic_cast<g2o::VertexExtremePointXY*>(graph->vertex(tmpId2));
                    if(vprev2)
                        vprev2->setIsExtremePoint(pdiV_common);
                }
                else
                    commonVertex = false;
            }

            if(!commonVertex)
            {
                cerr << "[NO COMMON VERTEX]" << endl;
                pdiV_notCommon = lc[i][0].z();

                //first current vertex
                vp1 = new g2o::VertexExtremePointXY();
                id1 = ++id;
                vp1->setId(id1);
                vp1->setEstimate(p1); //p1 in local frame (già trasformati, non moltiplico per T)
                if(!vprev2)                
                    vp1->setIsExtremePoint(pdiV_notCommon);
                else
                {
                    if(rhoPrev2 <= rhoP1)
                    {
                        vprev2->setIsExtremePoint(pdiV_notCommon);
                        vp1->setIsExtremePoint(0.25); //for now
                        cerr << "<<<<<<< this line has a possibly fake extreme point(LEFT): " << vp1->estimate().transpose() << " with pdiV = " << vp1->isExtremePoint() << endl;

                    }
                    else if(rhoP1 < rhoPrev2)
                    {
                        // add this point as a fake extreme point of the line
                        vp1->setIsExtremePoint(pdiV_notCommon);
                        vprev2->setIsExtremePoint(0.25); //for now
                        cerr << "<<<<<<< the previous line has a possibly fake extreme point(RIGHT): " << vprev2->estimate().transpose() << " with pdiV = " << vprev2->isExtremePoint() << endl;
                    }
                }

                graph->addVertex(vp1);

                //edge between v and  first vertex
                g2o::EdgeSE2PointXY* erp1 = new g2o::EdgeSE2PointXY();
                erp1->setVertex(0,v);
                erp1->setVertex(1,vp1);
                erp1->setMeasurement(lp1); //p1 in global frame (not transformed, moltiplico per iT)
                Eigen::Matrix2d info1;
                info1 << 1000, 0, 0, 1000;
                erp1->setInformation(info1);
                graph->addEdge(erp1);
            }

            /*TODO
             * MODIFY THE KEEPING COMMON VERTEX:
             *  NOT THE PROVIOUS ONE BY DEFAULT
             *  BUT THE ONE WITH SMALLEST DISTANCE
            */

            //second current vertex
            vp2 = new g2o::VertexExtremePointXY();
			id2 = ++id;
			vp2->setId(id2);
            vp2->setEstimate(p2); //p2 in local frame (già trasformati, non moltiplico per T)
			graph->addVertex(vp2);
			vp1_oldVp2 = tmpVp2;
			id1_oldId2 = tmpId2;
            tmpVp2 = vp2;
			tmpId2 = id2;

			//edge between v and second vertice
            g2o::EdgeSE2PointXY* erp2 = new g2o::EdgeSE2PointXY();
			erp2->setVertex(0,v);
			erp2->setVertex(1,vp2);
            erp2->setMeasurement(lp2); //p2 in global frame (not transformed, moltiplico per iT)
			Eigen::Matrix2d info2;
			info2 << 1000, 0, 0, 1000;
			erp2->setInformation(info2);
			graph->addEdge(erp2);

            //line vertex
			vl = new g2o::VertexLine2D();
			lid = ++id;
			vl->setId(lid);
            vl->setEstimate(pointsToLine(p1,p2)); //line in local frame
			if(!commonVertex)
				vl->p1Id = id1;
			else
				vl->p1Id = id1_oldId2;
            vl->p2Id = id2;
			graph->addVertex(vl);

			//Edge between v and vl
			g2o::EdgeSE2Line2D* erl = new g2o::EdgeSE2Line2D();
			erl->setMeasurement(pointsToLine(lp1,lp2));
			erl->setVertex(0,v);
			erl->setVertex(1,vl);
			Eigen::Matrix2d infovl;
			infovl << 1000, 0, 0, 1000;
			erl->setInformation(infovl);
			graph->addEdge(erl);

			//Edge between vl and vp1
			g2o::EdgeLine2DPointXY* elp1 = new g2o::EdgeLine2DPointXY();
			elp1->setVertex(0,vl);
			if(!commonVertex)
				elp1->setVertex(1,vp1);
			else
				elp1->setVertex(1,vp1_oldVp2);
			elp1->setMeasurement(0);
			Eigen::Matrix<double, 1, 1> infolp1;
			infolp1(0,0) = 1e3;
			elp1->setInformation(infolp1);
			graph->addEdge(elp1);
		
			//Edge between vl and vp2				
			g2o::EdgeLine2DPointXY* elp2 = new g2o::EdgeLine2DPointXY;
			elp2->setVertex(0,vl);
			elp2->setVertex(1,vp2);
			elp2->setMeasurement(0);
			Eigen::Matrix<double, 1, 1> infolp2;
			infolp2(0,0) = 1e3;
			elp2->setInformation(infolp2);
			graph->addEdge(elp2);

            //save the current line points as previous
            prev = l;
        }
	}
	
	ofstream ofG2OLine(outfilename.c_str());/*, ios::app*/
	graph->save(ofG2OLine);
	ofG2OLine.flush();
	ofG2OLine.close();
}

// TODO
ViewerGUI::ViewerGUI(VertexDataVector* theVLdVector, Eigen::Isometry2d TheOffset, QWidget* parent)
{
  setupUi(this);
  QObject::connect(horizontalSlider, SIGNAL(sliderMoved(int)), this, SLOT(updateVal1(int)));
  QObject::connect(horizontalSlider_2, SIGNAL(sliderMoved(int)), this, SLOT(updateVal2(int)));
  QObject::connect(horizontalSlider_3, SIGNAL(sliderMoved(int)), this, SLOT(updateVal3(int)));
  QObject::connect(pushButton, SIGNAL(clicked()), this, SLOT(lineExtraction()));
  QObject::connect(pushButton_2, SIGNAL(clicked()), this, SLOT(showOriginal()));	
  QObject::connect(pushButton_3, SIGNAL(clicked()), this, SLOT(setIdIteration()));
  QObject::connect(pushButton_4, SIGNAL(clicked()), this, SLOT(ComputeAll()));
  QObject::connect(checkBox, SIGNAL(clicked(bool)),this, SLOT(setAlgorithm()));
  QObject::connect(checkBox_2, SIGNAL(clicked(bool)),this, SLOT(setAlgorithm()));

	
  slider1value = 0;
  slider2value = 0;
  slider3value = 0;
  algotype = noType;
  edgeExtr = 0;
  lineExtractor = 0;
  clusterer = 0;
  vldvector = theVLdVector;
  offset = TheOffset;
  numIteration = 0;
}

// ViewerGUI::ViewerGUI(LaserDataVector* theLdVector, QWidget* parent)
// {
//   setupUi(this);
//   QObject::connect(horizontalSlider, SIGNAL(sliderMoved(int)), this, SLOT(updateVal1(int)));
//   QObject::connect(horizontalSlider_2, SIGNAL(sliderMoved(int)), this, SLOT(updateVal2(int)));
//   QObject::connect(horizontalSlider_3, SIGNAL(sliderMoved(int)), this, SLOT(updateVal3(int)));
//   QObject::connect(pushButton, SIGNAL(clicked()), this, SLOT(lineExtraction()));
//   QObject::connect(pushButton_2, SIGNAL(clicked()), this, SLOT(showOriginal()));	
//   QObject::connect(pushButton_3, SIGNAL(clicked()), this, SLOT(setIdIteration()));
//   QObject::connect(checkBox, SIGNAL(clicked(bool)),this, SLOT(setAlgorithm()));
//   QObject::connect(checkBox_2, SIGNAL(clicked(bool)),this, SLOT(setAlgorithm()));
// 
// 	
//   slider1value = 0;
//   slider2value = 0;
//   slider3value = 0;
//   algotype = noType;
//   edgeExtr = 0;
// 	lineExtractor = 0;
// 	clusterer = 0;
// 	ldvector = theLdVector;
// 	numIteration = 0;
// }
