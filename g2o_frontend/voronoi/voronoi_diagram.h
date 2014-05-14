#ifndef VORONOI_DIAGRAM_H
#define VORONOI_DIAGRAM_H

#include <Eigen/Core>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <deque>
#include <fstream>
#include <iostream>
#include <sys/time.h>
#include <vector>

#include "voronoi_vertex.h"

typedef std::set<VoronoiVertex*> VertexSet;

struct Component
{
    Component()
    {
        _vset = new VertexSet;
        _id = -1;
    }

    inline void add(VoronoiVertex* v) { _vset->insert(v); }

    inline VertexSet* vset() { return _vset; }
    inline const VertexSet* vset() const { return _vset; }

    inline void setId(const int id){ _id = id; }
    inline int& id() { return _id; }
    inline const int& id() const { return _id; }

    VertexSet* _vset;
    int _id;
};
typedef std::map<int, Component*> ComponentMap;

class VoronoiDiagram
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VoronoiDiagram(const cv::Mat& input, const int& vres, const float& _mapResolution= 0.05);
    ~VoronoiDiagram();

    bool addVertex(VoronoiVertex* v);
    bool addEdge(VoronoiEdge* e);

    void createObservations();

    void distmapExtraction();
    void distmap2image();
    void voronoiExtraction();

    void reconnect();

    /**
     * @brief Provides a graph consisting of voronoi vertices.
     *        Denser than a standard Voronoi Graph.
     */
    void denseGraphExtraction();

    void save2g2o(std::ostream& os, bool sparse = true);
    bool saveEdge(std::ostream& os, VoronoiEdge* e);
    bool saveData(std::ostream& os, VoronoiData* d);
    bool saveVertex(std::ostream& os, VoronoiVertex* v);

    void skeleton2vmap();
    void vmap2image();

    /**
     * @brief Simple thinning algorithm
     *
     * @param src Input image: it should be a binary (1 and 0 values) 8 bits single channel image.
     *            In case, use the binarize flag.
     * @param dst Ouput (binary) image
     * @param binarize true if you want to binarize the input image, defaults to true.
     * @param thresh Threshold for the binarization, defaults to 0.
     *
     * Implementation of the thinning algorithm presented in the paper
     * "A Fast Parallel Algorithm for Thinning Digital Patterns"
     * T. Y. Zhang and C. Y. Suen
     * Communications of the ACM March 1984 Volume 27 Number 3
     */
    void morphThinning(bool binarize = true, uchar thresh = 0);


    /**
     * @brief Extract a (possibly not connected) graph from a binary skeleton
     *
     * @param skeleton Input skeleton image: it should be a general, 0 and X values binary
     *                 8 bits single channel image.
     * @param nodes Output graph nodes
     * @param edges Output graph edges, by means of a vecto of vectors of index
     * @param find_leafs true if you want to extract also the leaf nodes, defaults to true
     * @param min_dist minimum distance between nodes, it should be >= 1, default to 1
     */
    void graphExtraction(std::vector<cv::Point2f> &nodes,
                         std::vector< std::vector<int> > &edges,
                         bool find_leafs = true, int min_dist = 1);


    void init(const cv::Mat& img_);
    void newinit(const cv::Mat& img_);
    void loadPGM();
    void savePGM(const char *filename, const Eigen::MatrixXf& image_);
    void shrinkIntersections(cv::Mat &skeleton, cv::Mat &dst);
    static void findNeighborNodes(int src_node_idx, int x, int y, cv::Mat &mask, cv::Mat &index_mat,
                                  std::vector<cv::Point2f> &nodes, std::vector< std::vector<int> > &edges);

    EdgeSet vertexEdges(VoronoiVertex* v);
    int _squaredResolution;
    float _mapResolution;

    int _rows, _cols;

    VertexMap _vertices;
    VertexMap _candidates;
    EdgeSet _edges;

    Eigen::MatrixXf _drawableDistmap;

    cv::Mat _map;
    cv::Mat _skeleton;
    cv::Mat _prova;
    cv::Mat* _voro;
    cv::Mat* _graph;

    VoronoiVertex* _dmap;
    VoronoiQueue* _vQueue;
    ComponentMap* _regions;

protected:
    void fillQueue();
};
#endif // VORONOI_DIAGRAM_H
