#include "Eigen/Core"
#include "Eigen/Geometry"
#include <map>
#include <set>
#include <iostream>
#include <fstream>
#include <sstream>
#include <QGLViewer/qglviewer.h>
#include "GL/gl.h"
#include <QApplication>
#include <stdexcept>

using namespace Eigen;
using namespace std;


Eigen::Vector3d t2v(const Eigen::Isometry2d& iso) {
  Eigen::Vector3d t;
  t.x() = iso.translation().x();
  t.y() = iso.translation().y();
  Eigen::Rotation2Dd r(0);
  r.fromRotationMatrix(iso.linear());
  t.z() = r.angle();
  return t;
}


Eigen::Isometry2d v2t(const Eigen::Vector3d& v) {
  Isometry2d iso;
  iso.translation() = v.head<2>();
  iso.linear() = Rotation2Dd(v.z()).toRotationMatrix();
  return iso;
}

struct Graph;
struct Node;
struct Edge;


typedef std::set<Edge*> EdgeSet;
typedef std::set<Node*> NodeSet;
typedef std::map<Node*, EdgeSet> NodeEdgeMap;
typedef std::map<int, Edge*> EdgeMap;
typedef std::map<int, Node*> NodeMap;

struct GraphElement {
  GraphElement(int id = -1, Graph* graph=0) {
    this->id = id;
    this->graph = graph;
  }
  int id;
  Graph* graph;
};

struct Node : public GraphElement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  Node(int id = -1, Graph* graph=0): GraphElement(id, graph) {
    pose.setIdentity();
  }

  const EdgeSet& edges();

  Eigen::Isometry2d pose;
};

struct Edge : public GraphElement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Edge(int id = -1, Graph* graph=0): GraphElement(id, graph) {
    transform.setIdentity();
    from = 0;
    to = 0;
  }
  Node* from, * to;
  Eigen::Isometry2d transform;
};

struct Graph {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Graph() {
  }
  Node* node(int id) {
    NodeMap::iterator it = nodes.find(id);
    if (it!=nodes.end())
      return it->second;
    return 0;
  }

  bool addNode(Node* n) {
    Node* n2 = node(n->id);
    if (n2)
      return false;
    nodes.insert(make_pair(n->id, n));
    nodeEdges.insert(make_pair(n, EdgeSet()));
    return true;
  }

  Edge* edge(int id) {
    EdgeMap::iterator it = edges.find(id);
    if (it!=edges.end())
      return it->second;
    return 0;
  }

  bool addEdge(Edge* e){
    Edge* e2 = edge(e->id);
    if (e2)
      return false;
    edges.insert(make_pair(e->id, e));
    nodeEdges[e->from].insert(e);
    nodeEdges[e->to].insert(e);
    return true;
  }

  const EdgeSet& edgesOf(Node* n) {
    NodeEdgeMap::iterator it = nodeEdges.find(n);
    if (it==nodeEdges.end())
      throw std::runtime_error("unable to find an entry in the node edge map");
    return it->second;
  }

  NodeEdgeMap nodeEdges;
  EdgeMap edges;
  NodeMap nodes;
};

const EdgeSet& Node::edges() { return graph->edgesOf(this); }

void loadGraph(Graph* graph, istream& is){
  std::string tag;
  const int maxDim= 32000;
  int eid = 0;
  while (is) {
    char line[maxDim];
    is.getline(line, maxDim);
    //  cerr << "line: " << line << endl;
    istringstream ls(line);
    string tag;
    ls >> tag;
    //  cerr <<  "tag: [" << tag << "]" << endl;
    if (tag == "VERTEX_SE2"){
      int id;
      Vector3d v;
      ls >> id >> v.x() >> v.y() >> v.z();
      Node* n = new Node(id, graph);
      n->pose = v2t(v);
      graph->addNode(n);
    } else if (tag == "EDGE_SE2"){
      //  cerr << "e";
      int id1, id2;
      Vector3d v;
      ls >> id1 >> id2 >> v.x() >> v.y() >> v.z();
      Edge* e = new Edge(eid++, graph);
      e->from = graph->node(id1);
      e->to = graph->node(id2);
      e->transform = v2t(v);
      graph->addEdge(e);
    } else {
      //  cerr << "tag: " << tag << endl;
    }
  }
}


  class StandardCamera : public qglviewer::Camera {
  public:
    StandardCamera() : _standard(true) {}
  
    float zNear() const {
      if(_standard) 
	return 0.001f; 
      else 
	return Camera::zNear(); 
    }

    float zFar() const {  
      if(_standard) 
	return 10000.0f; 
      else 
	return Camera::zFar();
    }

    bool standard() const { return _standard; }
  
    void setStandard(bool s) { _standard = s; }

  protected:
    bool _standard;
  };

class DumbViewer;

struct DrawableItem {
  DrawableItem(DumbViewer* viewer);
  virtual ~DrawableItem();
  virtual void draw() = 0;
  DumbViewer* viewer;
};

  struct GraphDrawableItem: public DrawableItem {
    float r,g,b;
    GraphDrawableItem( Graph* g, DumbViewer* v): DrawableItem(v) {
      graph = g;
      this->r = 0.8;
      this->g = 0;
      this->b = 0;
    }
    virtual void draw();
    Graph* graph;
  };

  void GraphDrawableItem::draw() {
    cerr << "g: " << graph << endl;
    glColor3f(r,g,b);
    glPointSize(3);
    glBegin(GL_LINES);
    int numEdges = 0;
    for (EdgeMap::iterator it = graph->edges.begin(); it!=graph->edges.end(); it++){
      Edge* e = it->second;
      Node* n1 = e->from;
      Node* n2 = e->to;
      glVertex3f(n1->pose.translation().x(), n1->pose.translation().y(), 0);
      glVertex3f(n2->pose.translation().x(), n2->pose.translation().y(), 0);
      numEdges ++;
    }
    glEnd();
  }


  class DumbViewer: public QGLViewer{
  public:
    DumbViewer(QWidget *parent = 0,  const QGLWidget *shareWidget = 0, Qt::WFlags flags = 0);
    virtual ~DumbViewer();
    virtual void init();
    void addItem(DrawableItem* d){
      items.insert(d);
    }
    void removeItem(DrawableItem* d){
      items.erase(d);
    }
    void draw();
    Graph* graph;
    std::set<DrawableItem*> items;
  };


DrawableItem::DrawableItem(DumbViewer* viewer) {
  viewer->addItem(this);
  this->viewer = viewer;
}

DrawableItem::~DrawableItem() {
  if (viewer)
    viewer->removeItem(this);
}


  DumbViewer::DumbViewer(QWidget *parent, 
			 const QGLWidget *shareWidget, 
			 Qt::WFlags flags): QGLViewer(parent,shareWidget,flags){}

  DumbViewer::~DumbViewer() {}

  void DumbViewer::init(){
    // Init QGLViewer.
    QGLViewer::init();
    // Set background color light yellow.
    setBackgroundColor(QColor::fromRgb(255, 255, 194));

    // Set some default settings.
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND); 
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_FLAT);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Don't save state.
    setStateFileName(QString::null);

    // Mouse bindings.
    setMouseBinding(Qt::RightButton, CAMERA, ZOOM);
    setMouseBinding(Qt::MidButton, CAMERA, TRANSLATE);

    // Replace camera.
    qglviewer::Camera *oldcam = camera();
    qglviewer::Camera *cam = new StandardCamera();
    setCamera(cam);
    cam->setPosition(qglviewer::Vec(0.0f, 0.0f, 10.0f));
    cam->setUpVector(qglviewer::Vec(1.0f, 0.0f, 0.0f));
    cam->lookAt(qglviewer::Vec(0.0f, 0.0f, 0.0f));
    delete oldcam;
  }

  
void DumbViewer::draw(){
  cerr << "s: " <<items.size() << endl;
  for (std::set<DrawableItem*>::iterator it=items.begin(); it!=items.end(); it++){
    DrawableItem* item = *it;
    item->draw();
  }
}

/*
struct RecursiveMatcher{
  Graph* g1, *g2;
  double visitDistance;
 

  std::set<Edge*> matches;
  std::set<Node*> visited1, visited2;

  double nodeDistance, nodeRotation;

  matchChildren(Node* n1, Node* n2) {
    Eigen::Isometry2d n2pose= n2->pose;
    const EdgeSet& n1Set = n1->edges();
    const EdgeSet& n2Set = n2->edges();
    for (EdgeSet::const_iterator it = n1Set.begin(); it!=n1Set.end(); it++) {
      Edge* e1=*it;
      Node* n1Other = (n1 == e1->from)? e1->to : e1->from;
      if (visited1.count(n1Other))
	continue;
      visited1.insert(n1Other);
      
      Node* bestN2 = 0;
      float bestChi = 1e9;
      
      Eigen::Isometry2d n1OtherInverse = n1->pose.inverse();
      for (EdgeSet::const_iterator it = n2Set.begin(); it!=n2Set.end(); it++) {
	Edge* e2=*it;
	Eigen::Isometry2d t = e2->transform;
	Node* n2Other = e2->to;
	if(n2 == e2->to){
	  t = t.inverse();
	  n2Other = e2->from;
	}
	if (visited2.count(n2Other))
	  continue;
	n2Other.pose = n2.pose*t;
	
	Vector3d delta = t2v(n1OtherInverse*n2Other.pose);
	if (delta.squaredNorm()<bestChi){
	  bestN2 = n2Other;
	  bestChi = delta;
	}
      }
      if (bestN2)
	visited2.insert(bestN2);
      visited1.erase(n1Other);
    }
  }
  
}
*/
#define MARKUSED(x) x=x

int main(int argc, char ** argv) {
  std::vector<Graph*> graphs;
  int c = 1;
  while(c<argc) {
    cerr << "loading file " << argv[c] << endl;
    ifstream is(argv[c]);
    Graph* g = new Graph;
    loadGraph(g, is);
    graphs.push_back(g);
    c++;
  }

  QApplication* app = new QApplication(argc,argv);
  DumbViewer *viewer = new DumbViewer();
  viewer->show();
  for  (size_t i = 0; i<graphs.size(); i++){
    GraphDrawableItem* gdi=new GraphDrawableItem(graphs[i], viewer);
    MARKUSED(gdi);
  }

  while(viewer->isVisible()){
    app->processEvents();
    usleep(1000);
  }

}
