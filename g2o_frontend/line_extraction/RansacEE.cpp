#include "RansacEE.h"
#include <cmath>
#include <bits/algorithmfwd.h>

RansacEE::RansacEE(){
}

RansacEE::~RansacEE(){
}

vector<int> RansacEE::randIndices(int range){
	
	vector<int> indices;
	
	int a = 0;
	int b = 0;
	if (range > 1){
		while( a == b){
		  
			a = rand() % range ;
			b = rand() % range ;
		}
	}
	//cout << "**Ransac** # a value: "<< a <<endl;
	//cout << "**Ransac** # b value: "<< b <<endl;
	
	indices.push_back(a);
	indices.push_back(b);
	  
	return indices;
}

float RansacEE::distPE (Vertex* v, Edge* e){
  
	float x1, y1, x2, y2, m, q;

	x1 = e->getVertexFrom()._x;
	y1 = e->getVertexFrom()._y;
	x2 = e->getVertexTo()._x;
	y2 = e->getVertexTo()._y;

	// y=mx+q: straight line crossing 2 points
	m = (y2-y1)/(x2-x1);
	q = -(y2-y1)*x1/(x2-x1)+y1;

	//distanza punto retta

	return fabs((v->y()) - m*(v->x()) - q)/sqrt(1 + m*m);
}

VerticesMap RansacEE::getInlierSet(Edge* e, VerticesMap map, float th){
	
	VerticesMap InlierSet;
	InlierSet.clear();
	
	if ( e->getLength() < 0.05) { return InlierSet;}
	
	VerticesMap::iterator it;
	it =  map.begin();
	int kprev = 0;
	int  k = 0;
	
	while(it != map.end()){
		
		float d = 0;
		d = distPE((*it).second, e);

		if(d < th ){ 
			  
			  k = it->first;
			  if (kprev == 0 || kprev == k-1){ 
				//cout << "**Ransac** Inlier key: "<< k <<endl;
				InlierSet[k] = (*it).second;
				kprev = k;
			  }
			  else return InlierSet;
		}
		it++;
	}
	return InlierSet;
}




void RansacEE::step(VerticesMap &map_vertex, Edges &edges){
  
  //optimazable params
	float th = 0.02;
	int minNrInliers = 4;
	int minNrOfElems = 4;
	
	vector<Edge*> VectorEdges;
	
	int h = 0;
	int w = 0;
	while(map_vertex.size() > minNrOfElems && h < 1000){
	  
		//cout << "**Ransac** Map Size: "<< map_vertex.size() <<endl;
	  
		vector<int> IndicesVector = randIndices(map_vertex.size());
		
		//cout << "**Ransac** Vector Size: "<< IndicesVector.size() <<endl;
		
		Vertex* vf = new Vertex(h);
		Vertex* vt = new Vertex(h);
		
		int a = IndicesVector[0];
		int b = IndicesVector[1];
		
		VerticesMap::iterator it;
		it =  map_vertex.begin();
		int i = 0;
		int id1 = 0;
		int id2 = 0;
		while(it != map_vertex.end()){
		  
			if(i == a){
			  
				vf =  (it->second);  // non  avendo tutte le chiavi sequenziali anziche prendere la chiave a prendo l'elemento a-esimo
				//cout << "**Ransac** # a value: "<< (*it).first <<endl;
				id1 = (it->first);
			}
			if(i == b){
			  
				vt = (it->second);
				//cout << "**Ransac** # b value: "<< (*it).first <<endl;
				id2 = (it->first);
			}
			
			i++;
			it++;
		}

		Edge* e  = new Edge(0);
		
		if(id2 > id1){
			e->setVertexFrom(*vf);
			e->setVertexTo(*vt);
		}
		else{
			e->setVertexFrom(*vt);
			e->setVertexTo(*vf);
		}

		int id = 0;
		if(id1 < id2 ){ id = id1;}
		else id = id2;
		
		//cout << "xf: "<<e->getVertexFrom().x()<<" yf: "<<e->getVertexFrom().y()<<endl;
		//cout << "xf: "<<e->getVertexTo().x()<<" yf: "<<e->getVertexTo().y()<<endl;
		
		//cout << "**Ransac** id of init edge: "<< h <<endl;
		
		VerticesMap InlierSet = getInlierSet(e, map_vertex, th);
		
		//cout << "**Ransac** # of Inliers: "<< InlierSet.size() <<endl;
		//cout << "**Ransac** Map Size: "<< map_vertex.size() <<endl;
		
		if(InlierSet.size() > minNrInliers ){  
		  
			Edge* ed = fitLine(InlierSet,id, 0.003); 
			
			// se l'errore del fitting non supera una certa soglia
			if(ed != 0 && ed->getLength() > 0.08){
			  
				VectorEdges.push_back(ed);
				//cout << "**Ransac** # of edges found: "<< VectorEdges.size() <<endl;
				// elimino gli inliners dal set originario
				VerticesMap::iterator it;
				it = InlierSet.begin();
				int k = 0;
				while(it != InlierSet.end()){
				  
					k = (*it).first;
					map_vertex.erase(k);
					it++;
				}
			}
		}
		
		h++;
	}
	
	//cout << "**Ransac** out of while: "<< Edges.size() <<endl;
	//Edges mergedEdges = merge(VectorEdges, 0.05);
	sort(VectorEdges);
	edges = VectorEdges; 
	//cout << "ITERATIONS: "<< h <<endl;
	
}

 Edges RansacEE::merge(Edges VectorEdges, float th){
  
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
				//cout<<"   index j : "<<j<<endl;
				float x1f, y1f, x1t, y1t, m1, q1, s1;

				x1f = a->getVertexFrom()._x;
				y1f = a->getVertexFrom()._y;
				x1t = a->getVertexTo()._x;
				y1t = a->getVertexTo()._y;

				s1 = a->getSlopeInRad(); 
				
				float x2f, y2f, x2t, y2t, m2, q2, s2;

				x2f = b->getVertexFrom()._x;
				y2f = b->getVertexFrom()._y;
				x2t = b->getVertexTo()._x;
				y2t = b->getVertexTo()._y;

				s2 = b->getSlopeInRad(); 
				
				if(fabs(s1-s2) < 0.5){
				  
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

						delete(af);
						delete(at);
						delete(bf);
						delete(bt);
						
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
