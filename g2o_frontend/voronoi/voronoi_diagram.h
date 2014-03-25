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


class VoronoiDiagram
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VoronoiDiagram(const cv::Mat& input, int);
    ~VoronoiDiagram();

    void checkQueue();
    void checkStats();

    void distmapExtraction();
    void distmap2image();
    void voronoiExtraction();

    void fillQueue();

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
    void morphThinning(cv::Mat &src, cv::Mat &dst, bool binarize = true, uchar thresh = 0);


    /**
     * @brief Extract a (possibly not connected) graph from a binary skeleton
     *
     * @param skeleton Input skeleton image: it should be a general, 0 and X values binary
     *                 8 bits single channel image.
     * @param nodes Outout graph nodes
     * @param edges Outout graph edges, by means of a vecto of vectors of index
     * @param find_leafs true if you want to extract also the leaf nodes, defaults to true
     * @param min_dist minimum distance between nodes, it should be >= 1, default to 1
     */
    void graphExtraction(cv::Mat &skeleton, std::vector<cv::Point2f> &nodes,
                         std::vector< std::vector<int> > &edges,
                         bool find_leafs = true, int min_dist = 1);

    void init(const cv::Mat& img_);
    void loadPGM();
    void savePGM(const char *filename, const Eigen::MatrixXf& image_);
    static void shrinkIntersections( cv::Mat &skeleton, cv::Mat &dst);
    static void findNeighborNodes(int src_node_idx, int x, int y, cv::Mat &mask, cv::Mat &index_mat,
                                  std::vector<cv::Point2f> &nodes, std::vector< std::vector<int> > &edges);

    int _squaredResolution;
    int _rows, _cols;

    VertexMap _vMap;
    VertexMap _candidates;

    Eigen::MatrixXf _drawableDistmap;

    cv::Mat* _voro;
    cv::Mat* _graph;

    VoronoiVertex* _dmap;
    DistanceMap* _distmap;
    VoronoiQueue* _vQueue;
};
#endif // VORONOI_DIAGRAM_H
