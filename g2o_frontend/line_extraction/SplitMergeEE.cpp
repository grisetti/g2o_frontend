#include "SplitMergeEE.h"
#include <utility>
#include <Eigen/Dense>

SplitMergeEE::SplitMergeEE(){
}
SplitMergeEE::~SplitMergeEE(){
}



void SplitMergeEE::step(VerticesMap &map_vertex, Edges &edges){
	
	vector<VerticesMap*> VectorOfVerticesMap;
	vector<Edge*> VectorEdges;
	
	float th = 0.2;
	float dth = 0.1;

	VerticesMap::iterator itv;
	itv = map_vertex.begin();
	int kprev = (*itv).first-1;
	int  k = 0;
	VerticesMap * vm = new VerticesMap();
	Vertex * vt  = (*itv).second;
	
	// divide the entire set in subsets between the saturated areas and close enough points
	while(itv != map_vertex.end()){
	  
		k = (*itv).first;
			  
		if (k == kprev + 1 && map_vertex[k]->dist(vt) < 0.2){ 
			vt = (*itv).second;
			(*vm)[k] = vt;
			kprev = k;
		}
		else{	 
			VectorOfVerticesMap.push_back(vm);
			vm = new VerticesMap();
			vt = (*itv).second;
			(*vm)[k] = vt;
			kprev = k;
		}
		itv++;
	}
	VectorOfVerticesMap.push_back(vm);

	// cout<<"SM - #Initial Sets : "<<VectorOfVerticesMap.size()<<endl;
	
	int i = 0;
	while( i < VectorOfVerticesMap.size() ){
	  	  
		VerticesMap currVertices = *(VectorOfVerticesMap[i]);

		VerticesMap::iterator itf;
		itf = currVertices.begin();
		Vertex * from = new Vertex(*(*itf).second);
		int id = (*itf).first;
		Vertex & toTemp =  *(currVertices.rbegin()->second);
		Vertex * to = new Vertex(toTemp);
 
		Edge * e = 0;
		e = new Edge(id, from, to);
		
		if(e != 0){
			// find vertex of maximum distance from the edge we found
			float dMax = 0;
			float d = 0;
			Vertex* vMax;

			VerticesMap::iterator itm;
			itm = currVertices.begin();
			
			while( itm != currVertices.end() ){
			  
				d = distPE( itm->second, e);
				if(d > dMax){
					vMax = itm->second;
					dMax = d;
				}
				itm++;
			}
			
			// if maximum distance is less then a threshold
			if(dMax < th) {	VectorEdges.push_back(e);}
			else {
				vector<VerticesMap*> partitions = split(currVertices, e, vMax);
				
				if(((*partitions[0]).size()) > 1){  
					VectorOfVerticesMap.push_back(partitions[0]);
				}
				if(((*partitions[1]).size()) > 1){
					VectorOfVerticesMap.push_back(partitions[1]);
				}
			}
			i++;
		}
	}
	
	//cout <<"SM - # Edges Found : "<< VectorEdges.size()<<endl;
	Edges MergedVectorEdges = merge(VectorEdges, dth);
	
	sort(MergedVectorEdges);
	edges = MergedVectorEdges;
	
	//cout<<"SM - # Merged Edges : "<< edges.size() <<endl;
}

float SplitMergeEE::distPE (Vertex* v, Edge* e){
  
	float x1, y1, x2, y2;

	x1 = e->getVertexFrom()._x;
	y1 = e->getVertexFrom()._y;
	x2 = e->getVertexTo()._x;
	y2 = e->getVertexTo()._y;

	// ax+by+c = 0 retta per due punti
	float a = -(y2-y1);
	float b = (x2-x1);
	//float c = (y2-y1)*x1 - y1*(x2-x1);
	float c = x1*y2 - y1*x2;

	//distanza punto retta
	return fabs(v->x()*a + v->y()*b + c)/sqrt(a*a + b*b);
}


vector<VerticesMap*> SplitMergeEE::split(VerticesMap curr, Edge* e, Vertex* v){

	float ax = e->getVertexFrom()._x;
	float ay = e->getVertexFrom()._y;
	float bx = e->getVertexTo()._x;
	float by = e->getVertexTo()._y;
	float vx = v->_x;
	float vy = v->_y;
	Vertex * proj_v = new Vertex(0);

	Eigen::Vector2d ab( bx - ax, by - ay ); // vector from A to B
	float ab_squared = (ab.x() * ab.x() + ab.y() * ab.y());	// squared distance from A to B
	
	if( fabs(ab_squared) < 0.001) { proj_v->setX(ax); proj_v->setY(ay); }
	
	else { 
		Eigen::Vector2d av( vx - ax, vy - ay );

		// Consider the line extending the segment, parameterized as A + t (B - A)
		// We find projection of point p onto the line. 
		// It falls where t = [(p-A) . (B-A)] / |B-A|^2
		
		float t = (ab.x() * av.x() + ab.y() * av.y()) / ab_squared;
		
		if (t < 0.0){
			// "Before" A on the line, just return A
			proj_v->setX(ax);
			proj_v->setY(ay);
		}
		else if (t > 1.0){ 
			// "After" B on the line, just return B
			proj_v->setX(bx);
			proj_v->setY(by);
		}
		else{	
			// projection lines "inbetween" A and B on the line
			//proj = A + t * AB;
			proj_v->setX(ax + t*ab.x());
			proj_v->setY(ay + t*ab.y());
		}
	}
	
	VerticesMap * part_1 = new VerticesMap();
	VerticesMap * part_2 = new VerticesMap();
	
	VerticesMap::iterator it;
	it = curr.begin();
	
	float sel = 0;
	
	while (it != curr.end()){
		
		Vertex * v_curr = it->second;  
		sel = (proj_v->x() - vx)*(v_curr->y() - vy) - (proj_v->y() - vy)*(v_curr->x() - vx);
		
		if(sel > 0) { (*part_1)[it->first] = v_curr; }
		else if(sel < 0){ (*part_2)[it->first] = v_curr; }
		
		it++;
	}
	
	vector<VerticesMap*> parts;
	
	parts.push_back(part_2);
	parts.push_back(part_1);
	
	return parts;    
}


 Edges SplitMergeEE::merge(Edges VectorEdges, float th){
  
	vector<Edge*> mergedEdges;	
	bool bl = false;
	Edges::iterator iti;
	Edges::iterator itj;
	iti = VectorEdges.begin();
	int i = 0;
 	while(iti != VectorEdges.end()){
		
		Edge* a = *iti;
		
		itj = VectorEdges.begin();
		int j = 0;
		while(itj != VectorEdges.end()){
		  
			if(iti != itj){
		  
				Edge* b =*itj;
				
				float s1 = a->getSlopeInRad(); 
				float s2 = b->getSlopeInRad();

				if(fabs(s1-s2) < 0.1){
				  
					float x1f, y1f, x1t, y1t, m1, q1;

					x1f = a->getVertexFrom()._x;
					y1f = a->getVertexFrom()._y;
					x1t = a->getVertexTo()._x;
					y1t = a->getVertexTo()._y;					
					
					float x2f, y2f, x2t, y2t, m2, q2;

					x2f = b->getVertexFrom()._x;
					y2f = b->getVertexFrom()._y;
					x2t = b->getVertexTo()._x;
					y2t = b->getVertexTo()._y;
				  
					float  tf = sqrt(pow(x1t-x2f,2)+pow(y1t-y2f,2));
					float  tt = sqrt(pow(x1t-x2t,2)+pow(y1t-y2t,2));
					float  ff = sqrt(pow(x1f-x2f,2)+pow(y1f-y2f,2));
					float  ft = sqrt(pow(x1f-x2t,2)+pow(y1f-y2t,2));
					
					if (tf < th || tt < th || ff < th || ft < th) {
						
						VerticesMap vm;
						Vertex* af = new Vertex(0,x1f,y1f);
						Vertex* at = new Vertex(0,x1t,y1t);
						Vertex* bf = new Vertex(0,x2f,y2f);
						Vertex* bt = new Vertex(0,x2t,y2t);
						vm[0] = af;
						vm[1] = at;
						vm[2] = bf;
						vm[3] = bf;

						//delete(af);
						//delete(at);
						//delete(bf);
						//delete(bt);
						
						Edge* e = fitLine(vm,a->id,1000);

						VectorEdges.erase(itj);
						VectorEdges.erase(iti);
						
						//cout<<"indices eliminated: "<< i <<"-"<< j <<endl;

						VectorEdges.push_back(e);
						bl = true; 						
						break;
					}
				}
			}
			j++;
			itj++;
		}
	if(bl){
		iti = VectorEdges.begin();
		i = 0;
		bl = false;
		}
	else{
		iti++;
		i++;
		}
	}
	
	return VectorEdges;
}

