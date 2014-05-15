#include "voronoi_diagram.h"

#define INF 1000000


using namespace std;
using namespace Eigen;



VoronoiDiagram::VoronoiDiagram(const cv::Mat &input, const int& vres, const float& _mapResolution)
    : _squaredResolution(vres), _mapResolution(_mapResolution)
{
    _vQueue = 0;
    _voro = 0;

    _graph = 0;
    _dmap = 0;

    this->init(input);
    this->newinit(input);
}


VoronoiDiagram::~VoronoiDiagram()
{
    delete[] _dmap;
    delete _voro;
    delete _graph;
    delete _vQueue;
}


bool VoronoiDiagram::addVertex(VoronoiVertex* v)
{
    if(!v)
    {
        return false;
    }
    VoronoiVertex* nv = _vertices.find(v->position())->second;
    if(nv)
    {
        return false;
    }
    _vertices.insert(make_pair(v->position(), v));
    return true;
}


void VoronoiDiagram::morphThinning(bool binarize, uchar thresh)
{
    cv::Mat dilated_voro = _voro->clone();
//    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(7, 7),cv::Point(3, 3));
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3, 3),cv::Point(1, 1));
    cv::dilate(*_voro, dilated_voro, element);

    if(dilated_voro.depth() != cv::DataType<uchar>::type || dilated_voro.channels() != 1 || !dilated_voro.rows || !dilated_voro.cols)
    {
        throw std::invalid_argument("Unsupported input image");
    }

    cv::Mat bin_img;
    if(binarize)
    {
        bin_img = cv::Mat(dilated_voro.size(), CV_8UC1);
        cv::threshold(dilated_voro, bin_img, double(thresh), 1, cv::THRESH_BINARY);
    }
    else
    {
        bin_img = dilated_voro;
    }

    _skeleton = bin_img.clone();

    int n_deleted;
    do
    {
        n_deleted = 0;
        for(int iter = 0; iter < 2; iter++)
        {
            for(int y = 1; y < bin_img.rows - 1; y++)
            {
                /* Image patch:
                * p9 p2 p3
                * p8 p1 p4
                * p7 p6 p5
                */
                uchar p1, p2, p3, p4, p5, p6, p7, p8, p9;
                const uchar *row_up_p = bin_img.ptr<uchar>(y-1),
                        *row_p = bin_img.ptr<uchar>(y),
                        *row_down_p = bin_img.ptr<uchar>(y+1);

                uchar *_skeleton_p = _skeleton.ptr<uchar>(y);
                _skeleton_p++;

                p2 = *row_up_p++;
                p1 = *row_p++;
                p6 = *row_down_p++;

                p3 = *row_up_p++;
                p4 = *row_p++;
                p5 = *row_down_p++;

                for(int x = 1; x < bin_img.cols - 1; x++, _skeleton_p++)
                {
                    p9 = p2;
                    p8 = p1;
                    p7 = p6;

                    p2 = p3;
                    p1 = p4;
                    p6 = p5;

                    p3 = *row_up_p++;
                    p4 = *row_p++;
                    p5 = *row_down_p++;

                    if(p1)
                    {
                        int patterns_01  =  (p2 == 0 && p3 ) + (p3 == 0 && p4 ) +
                                (p4 == 0 && p5 ) + (p5 == 0 && p6 ) +
                                (p6 == 0 && p7 ) + (p7 == 0 && p8 ) +
                                (p8 == 0 && p9 ) + (p9 == 0 && p2 );

                        int nonzero_neighbors  =  p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
                        int cond_1 = (iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8));
                        int cond_2 = (iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8));

                        if(patterns_01 == 1 && (nonzero_neighbors >= 2 && nonzero_neighbors <= 6)
                                && cond_1 == 0 && cond_2 == 0)
                        {
                            *_skeleton_p = 0;
                            n_deleted++;
                        }
                    }
                }
            }
            _skeleton.copyTo(bin_img);
        }
    }
    while(n_deleted);

    _skeleton *= 255;
}


void VoronoiDiagram::shrinkIntersections(cv::Mat& skeleton, cv::Mat& dst)
{
    cv::Mat mask = cv::Mat::zeros(skeleton.size(), CV_8UC1);//skeleton.clone();
    dst = skeleton.clone();

    for(int y = 1; y < skeleton.rows - 1; y++)
    {
        /* Image patch:
        * p9 p2 p3
        * p8 p1 p4
        * p7 p6 p5
        */

        const uchar *s1_p, *s2_p, *s3_p, *s4_p, *s5_p, *s6_p, *s7_p, *s8_p, *s9_p;
        const uchar *src_row_up_p = skeleton.ptr<uchar>(y-1),
                *src_row_p = skeleton.ptr<uchar>(y),
                *src_row_down_p = skeleton.ptr<uchar>(y+1);

        uchar *m1_p, *m2_p, *m3_p, *m4_p, *m5_p, *m6_p, *m7_p, *m8_p, *m9_p;
        uchar *mask_row_up_p = mask.ptr<uchar>(y-1),
                *mask_row_p = mask.ptr<uchar>(y),
                *mask_row_down_p = mask.ptr<uchar>(y+1);


        s2_p = src_row_up_p++;
        s1_p = src_row_p++;
        s6_p = src_row_down_p++;

        s3_p = src_row_up_p++;
        s4_p = src_row_p++;
        s5_p = src_row_down_p++;

        m2_p = mask_row_up_p++;
        m1_p = mask_row_p++;
        m6_p = mask_row_down_p++;

        m3_p = mask_row_up_p++;
        m4_p = mask_row_p++;
        m5_p = mask_row_down_p++;


        for(int x = 1; x < skeleton.cols - 1; x++)
        {
            s9_p = s2_p;
            s8_p = s1_p;
            s7_p = s6_p;

            s2_p = s3_p;
            s1_p = s4_p;
            s6_p = s5_p;

            s3_p = src_row_up_p++;
            s4_p = src_row_p++;
            s5_p = src_row_down_p++;

            m9_p = m2_p;
            m8_p = m1_p;
            m7_p = m6_p;

            m2_p = m3_p;
            m1_p = m4_p;
            m6_p = m5_p;

            m3_p = mask_row_up_p++;
            m4_p = mask_row_p++;
            m5_p = mask_row_down_p++;

            if(*s1_p)
            {
                if( *s2_p ) (*m2_p)++;
                if( *s3_p ) (*m3_p)++;
                if( *s4_p ) (*m4_p)++;
                if( *s5_p ) (*m5_p)++;
                if( *s6_p ) (*m6_p)++;
                if( *s7_p ) (*m7_p)++;
                if( *s8_p ) (*m8_p)++;
                if( *s9_p ) (*m9_p)++;
            }
        }
    }

    for(int y = 1; y < skeleton.rows - 1; y++)
    {
        uchar *s_p = skeleton.ptr<uchar>(y);
        uchar *m_p = mask.ptr<uchar>(y);
        uchar *d_p = dst.ptr<uchar>(y);
        s_p++;
        m_p++;
        d_p++;

        for(int x = 1; x < skeleton.cols - 1; x++, s_p++, m_p++, d_p++)
        {
            if(*m_p >= 5)
            {
                if(mask.at<uchar>( y, x - 1) >= 5 || mask.at<uchar>( y, x + 1) >= 5 ||
                        mask.at<uchar>( y - 1, x) >= 5 || mask.at<uchar>( y + 1, x) >= 5 ||
                        mask.at<uchar>( y - 1, x - 1) >= 5 || mask.at<uchar>( y + 1, x + 1) >= 5 ||
                        mask.at<uchar>( y - 1, x + 1) >= 5 || mask.at<uchar>( y + 1, x - 1) >= 5 )
                {
                    *d_p = 0;
                    *m_p = 0;

                    mask.at<uchar>(y, x - 1) = 0;
                    mask.at<uchar>(y, x + 1) = 0;
                    mask.at<uchar>(y - 1, x) = 0;
                    mask.at<uchar>(y + 1, x) = 0;
                    mask.at<uchar>(y - 1, x - 1) = 0;
                    mask.at<uchar>(y + 1, x + 1) = 0;
                    mask.at<uchar>(y - 1, x + 1) = 0;
                    mask.at<uchar>(y + 1, x - 1) = 0;
                }
            }
        }
    }
}


//void VoronoiDiagram::graphExtraction(cv::Mat& skeleton, vector<cv::Point2f>& nodes, vector<vector<int> >& edges, bool find_leafs, int min_dist)
void VoronoiDiagram::graphExtraction(vector<cv::Point2f>& nodes, vector<vector<int> >& edges, bool find_leafs, int min_dist)
{
    if(_skeleton.depth() != cv::DataType<uchar>::type || _skeleton.channels() != 1 || !_skeleton.rows || !_skeleton.cols)
    {
        throw std::invalid_argument("Unsopported input image");
    }

    if(min_dist < 1)
    {
        cout << "graphExtraction() : Warning! min_dist < 1, using default value 1" << endl;
        min_dist = 1;
    }

    cv::Mat index_mat = cv::Mat(_skeleton.size(), CV_32SC1, cvRealScalar(-1)),
            mask = cv::Mat::zeros(_skeleton.size(), CV_8UC1), tmp_skel;

    shrinkIntersections(_skeleton, tmp_skel);

    // Extract nodes
    nodes.clear();
    vector<float> n_nodes;

    for( int y = 1; y < tmp_skel.rows - 1; y++)
    {
        /* Image patch:
        * p9 p2 p3
        * p8 p1 p4
        * p7 p6 p5
        */

        const uchar *s1_p, *s2_p, *s3_p, *s4_p, *s5_p, *s6_p, *s7_p, *s8_p, *s9_p;
        const uchar *src_row_up_p = tmp_skel.ptr<uchar>(y-1),
                *src_row_p = tmp_skel.ptr<uchar>(y),
                *src_row_down_p = tmp_skel.ptr<uchar>(y+1);

        int32_t *index_mat_p = index_mat.ptr<int32_t>(y);
        index_mat_p++;

        s2_p = src_row_up_p++;
        s1_p = src_row_p++;
        s6_p = src_row_down_p++;

        s3_p = src_row_up_p++;
        s4_p = src_row_p++;
        s5_p = src_row_down_p++;

        for(int x = 1; x < tmp_skel.cols - 1; x++, index_mat_p++)
        {
            s9_p = s2_p;
            s8_p = s1_p;
            s7_p = s6_p;

            s2_p = s3_p;
            s1_p = s4_p;
            s6_p = s5_p;

            s3_p = src_row_up_p++;
            s4_p = src_row_p++;
            s5_p = src_row_down_p++;

            if(*s1_p)
            {
                int n_patterns;
                if(*s2_p)
                {
                    n_patterns =    (*s2_p && *s3_p == 0) + (*s3_p && *s4_p == 0) +
                            (*s4_p && *s5_p == 0) + (*s5_p && *s6_p == 0) +
                            (*s6_p && *s7_p == 0) + (*s7_p && *s8_p == 0) +
                            (*s8_p && *s9_p == 0);
                }
                else
                {
                    n_patterns  =   (*s2_p == 0 && *s3_p) + (*s3_p == 0 && *s4_p) +
                            (*s4_p == 0 && *s5_p) + (*s5_p == 0 && *s6_p) +
                            (*s6_p == 0 && *s7_p) + (*s7_p == 0 && *s8_p) +
                            (*s8_p == 0 && *s9_p);
                }
                if(n_patterns >= 3 || (find_leafs && n_patterns == 1))
                {
                    bool add_node = true;
                    int node_index;
                    // Ensure minimum distance between nodes
                    for(int tx = max<int>(0, x - min_dist); tx <= min<int>(tmp_skel.cols, x + min_dist); tx++)
                        for(int ty = max<int>(0, y - min_dist); ty <= min<int>(tmp_skel.rows, y + min_dist); ty++)
                            if( (tx != x || ty != y ) && (node_index = index_mat.at<int32_t>(ty,tx)) >= 0)
                            {
                                *index_mat_p = node_index;
                                nodes[node_index] += cv::Point2f(tx,ty);
                                n_nodes[node_index] += 1.0f;
                                add_node = false;
                            }

                    if(add_node)
                    {
                        *index_mat_p = nodes.size();
                        nodes.push_back(cv::Point2f(x,y));
                        n_nodes.push_back(1.0f);
                    }
                }
            }
        }
    }

    for(size_t i = 0; i < nodes.size(); i++)
    {
        nodes[i].x /= n_nodes[i];
        nodes[i].y /= n_nodes[i];
    }

    // Extract topology (edges)
    mask = tmp_skel.clone();
    edges.clear();
    edges.resize(nodes.size());

    for(size_t i = 0; i < nodes.size(); i++)
    {
        int x = cvRound(nodes[i].x), y = cvRound(nodes[i].y);
        findNeighborNodes(i, x, y, mask, index_mat, nodes, edges);
    }
}


void VoronoiDiagram::findNeighborNodes(int src_node_idx, int x, int y, cv::Mat &mask, cv::Mat &index_mat,
                                       vector<cv::Point2f> &nodes, vector< vector<int> > &edges)
{
    if(!mask.at<uchar>(y,x))
        return;

    mask.at<uchar>(y,x) = 0;
    //bool neighbor_node = false;
    for(int tx = max<int>(0, x - 1); tx <= min<int>(mask.cols, x + 1); tx++)
    {
        for( int ty = max<int>(0, y - 1); ty <= min<int>(mask.rows, y + 1); ty++)
        {
            if( (tx != x || ty != y ) && mask.at<uchar>(ty,tx))
            {
                int neighbor_idx = index_mat.at<int32_t>(ty, tx);
                if( neighbor_idx >= 0 && neighbor_idx != src_node_idx )
                {
                    //neighbor_node = true;
                    bool add_neighbor = true;
                    vector<int>& neighbors = edges[src_node_idx];
                    for(size_t i = 0; i < neighbors.size(); i++)
                    {
                        if(neighbors[i] == neighbor_idx)
                        {
                            add_neighbor = false;
                        }
                    }
                    if(add_neighbor)
                    {
                        edges[src_node_idx].push_back(neighbor_idx);
                        edges[neighbor_idx].push_back(src_node_idx);
                    }
                    return;
                }
            }
        }
    }

    for(int tx = std::max<int>(0, x - 1); tx <= std::min<int>(mask.cols, x + 1); tx++)
    {
        for(int ty = std::max<int>(0, y - 1); ty <= std::min<int>(mask.rows, y + 1); ty++)
        {
            if((tx != x || ty != y ) && mask.at<uchar>(ty,tx))
            {
                //         if( neighbor_node )
                //         {
                //           if( index_mat.at<int32_t>(ty, tx) < 0 )
                //             mask.at<uchar>(ty,tx) = 0;
                //         }
                //         else
                findNeighborNodes(src_node_idx, tx, ty, mask, index_mat, nodes, edges);
            }
        }
    }
}


void VoronoiDiagram::fillQueue()
{
    _vQueue = new VoronoiQueue;

    const short int coords_x[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
    const short int coords_y[8] = {-1,  0,  1, -1, 1, -1, 0, 1};

    for(int c = 0; c < _cols; ++c)
    {
        int cr = c * _rows;
        for(int r = 0; r < _rows; ++r)
        {
            int k = r + cr;
            VoronoiVertex* obstacle = &(_dmap[k]);
            if(obstacle->value() != 0)
            {
                continue;
            }
            const int obstacle_x = obstacle->position().x();
            const int obstacle_y = obstacle->position().y();

            bool counter = false;
            short int neighbors = 0;
            for(short int i = 0; i < 8; ++i)
            {
                int neighbor_x = obstacle_x + coords_x[i];
                int neighbor_y = obstacle_y + coords_y[i];

                // If at least one of the neighbors is a free cell, put the current obstacle into the queue
                int nk = neighbor_x + neighbor_y * _rows;
                if((neighbor_x >= 0) && (neighbor_y >= 0) && (neighbor_x < _rows) && (neighbor_y < _cols) && (_dmap[nk].distance() != 0))
                {
                    counter = true;
                    neighbors++;
                }
            }
            if(counter && (neighbors < 6) && (neighbors > 0))
            {
                _vQueue->push(obstacle);
                obstacle->setPushed();
            }
        }
    }
}


void VoronoiDiagram::distmapExtraction()
{
    this->fillQueue();

    const short int coords_x[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
    const short int coords_y[8] = {-1,  0,  1, -1, 1, -1, 0, 1};

    while(!_vQueue->empty())
    {
        VoronoiVertex* current = _vQueue->top();
        _vQueue->pop();

        // Current coordinates
        const int current_x = current->position().x();
        const int current_y = current->position().y();

        // Iterate over the neighbors
        for(short int i = 0; i < 8; ++i)
        {
            int nx = current_x + coords_x[i];
            int ny = current_y + coords_y[i];

            //Check if into boundaries or not an obstacle
            int nk = nx + ny * _rows;
            if((nx >= 0) && (ny >= 0) && (nx < _rows) && (ny < _cols) && (_dmap[nk].distance() != 0))
            {
                VoronoiVertex* neighbor = &(_dmap[nk]);
                Vector2i nc(nx, ny);

                // If needed, update the neighbor's distance from the obstacle
                double neighbor_to_current_parent_dist = (nc - current->parent()).squaredNorm();
                if(neighbor_to_current_parent_dist < neighbor->distance())
                {
                    neighbor->setDistance(neighbor_to_current_parent_dist);
                    neighbor->setParent(current->parent());
                    _vQueue->push(neighbor);
                }
            }
        }
    }
}


void VoronoiDiagram::voronoiExtraction()
{
    _voro = new cv::Mat(_rows, _cols, CV_8UC1, cv::Scalar(0));

    const short int coords_x[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
    const short int coords_y[8] = {-1,  0,  1, -1, 1, -1, 0, 1};

    for(int c = 0; c < _cols; ++c)
    {
        int cr = c * _rows;
        for(int r = 0; r < _rows; ++r)
        {
            int k = r + cr;
            if(_dmap[k].distance() != 0)
            {
                VoronoiVertex* current = &(_dmap[k]);
                for(short int i = 0; i < 8; ++i)
                {
                    const int neighbor_x = r + coords_x[i];
                    const int neighbor_y = c + coords_y[i];

                    int nk = neighbor_x + neighbor_y * _rows;
                    if((neighbor_x >= 0) && (neighbor_y >= 0) && (neighbor_x < _rows) && (neighbor_y < _cols) && (_dmap[k].distance() != 0))
                    {
                        VoronoiVertex* neighbor = &(_dmap[nk]);
                        float pdist = (current->parent() - neighbor->parent()).squaredNorm();
                        if(pdist > _squaredResolution)
                        {
                            _voro->at<uchar>(r, c) = 255;
                        }
                    }
                }
            }
        }
    }
}


void VoronoiDiagram::distmap2image()
{
    _drawableDistmap = MatrixXf::Zero(_cols, _rows);

    for(int c = 0; c < _cols; ++c)
    {
        int cr = c * _rows;
        for(int r = 0; r < _rows; ++r)
        {
            int k = r + cr;
            _drawableDistmap(c, r) = _dmap[k].distance();
        }
    }
}


void VoronoiDiagram::init(const cv::Mat& img_)
{
    _map = img_.clone();

    _rows = img_.rows;
    _cols = img_.cols;

    cout << "Input image type: " << img_.type() << endl;
    cout << "Input image depth: " << img_.depth() << endl;

    cv::Mat converted;
    img_.convertTo(converted, 0);

    cout << "Converted image type: " << converted.type() << endl;
    cout << "Converted image depth: " << converted.depth() << endl;

    _dmap = new VoronoiVertex[_rows * _cols];
    for(int c = 0; c < _cols; ++c)
    {
        int cr = c * _rows;
        for(int r = 0; r < _rows; ++r)
        {
            int k = r + cr;
            uchar pixel = converted.at<uchar>(r, c);

            _dmap[k].setPosition(r, c);
            _dmap[k].setGraphPose(Vector3d(r * _mapResolution, c * _mapResolution, 0));
            _dmap[k].setValue(pixel);
            _dmap[k].setNearest(r, c);
            if(pixel == 0)
            {
                _dmap[k].setDistance(0);
                _dmap[k].setParent(r, c);
            }
            else
            {
                _dmap[k].setDistance(INF);
                _dmap[k].setParent(INF, INF);
            }
        }
    }
}


void VoronoiDiagram::skeleton2vmap()
{
    _vertices.clear();
    for(int c = 0; c < _skeleton.cols; c++)
    {
        int cr = c * _rows;
        for(int r = 0; r < _skeleton.rows; r++)
        {
            int k = r + cr;
            if(_skeleton.at<uchar>(r,c) == 255)
            {
                VoronoiVertex* v = &_dmap[k];
                this->addVertex(v);
            }
        }
    }
}


void VoronoiDiagram::vmap2image()
{
    _prova = cv::Mat(_rows, _cols, CV_8UC1, cv::Scalar(0));
    cout << "here we are" << endl;
    for(VertexMap::const_iterator it = _vertices.begin(); it != _vertices.end(); it++)
    {
        VoronoiVertex* v = it->second;
        _prova.at<uchar>(v->position().x(), v->position().y()) = 255;
    }
    cv::imwrite("prova.pgm", _prova);
}


void VoronoiDiagram::denseGraphExtraction()
{
    _regions = new ComponentMap;

    const short coords_x[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
    const short coords_y[8] = {-1,  0,  1, -1, 1, -1, 0, 1};

    VoronoiQueue vq;

    int cnt = 0;
    Component* fcomp = new Component;
    fcomp->setId(cnt++);
    bool first = true;
    for(VertexMap::iterator it = _vertices.begin(); it != _vertices.end(); it++)
    {
        VoronoiVertex* v = it->second;
        if(first)
        {
            _candidates.insert(make_pair(v->position(), v));
            vq.push(v);
            first = false;
        }

        // first node of a disconnected component
        if(!v->visited() && !first)
        {
            if(fcomp->vset()->size() > 0)
            {
                _regions->insert(make_pair(fcomp->id(), fcomp));
            }

            fcomp = new Component;
            fcomp->setId(cnt++);

            vq.push(v);
        }
        while(!vq.empty())
        {
            VoronoiVertex* curr = vq.top();
            vq.pop();

            curr->setVisited();

            //Check neighbors
            const int cx = curr->position().x();
            const int cy = curr->position().y();

            for(short int i = 0; i < 8; i++)
            {
                const int nx = cx + coords_x[i];
                const int ny = cy + coords_y[i];

                int nk = nx + (ny *_rows);
                if(nx > 0 && ny > 0 && nx < _rows && ny < _cols && _skeleton.at<uchar>(nx,ny) == 255 && !_dmap[nk].visited() && !_dmap[nk].merged())
                {
                    VoronoiVertex* neighbor = &(_dmap[nk]);

                    //Check distance stuff
                    double dist = (neighbor->nearest() - curr->nearest()).squaredNorm();
                    if(dist < _squaredResolution)
                    {
                        neighbor->setNearest(curr->nearest());
                        neighbor->setMerged();
                    }
                    else
                    {
                        neighbor->setNearest(neighbor->position());
                        fcomp->add(neighbor);
                        neighbor->setComponent(fcomp->id());

                        _candidates.insert(make_pair(neighbor->position(), neighbor));

                        Vector2i nposition = curr->nearest();
                        VoronoiVertex* near = &_dmap[nposition.x() + nposition.y() * _rows];
                        VoronoiEdge* e = new VoronoiEdge(near, neighbor);
                        e->setTransform(near->toIsometry().inverse() * neighbor->toIsometry());
                        _edges.insert(e);
                    }
                    vq.push(neighbor);
                }
            }
        }
    }
    if(_regions->size() > 1)
    {
        this->reconnect();
    }
}


void VoronoiDiagram::reconnect()
{
    // Find the biggest component
    Component* biggest = 0;
    size_t bs = 0;
    // Find the biggest component
    for(ComponentMap::const_iterator it = _regions->begin(); it != _regions->end(); it++)
    {
        Component* cand = it->second;
        if(cand->vset()->size() > bs)
        {
            bs = cand->vset()->size();
            biggest = cand;
        }
    }

    // Every other component must be merged with the biggest one
    _regions->erase(biggest->id());
    bool merged = true;
    while(merged)
    {
        merged = false;
        VertexSet* bset = biggest->vset();

        double best = 1e9;
        VoronoiVertex* b1 = 0;
        VoronoiVertex* b2 = 0;
        for(ComponentMap::const_iterator it = _regions->begin(); it != _regions->end(); it++)
        {
            Component* curr = it->second;
            if(curr == biggest)
            {
                continue;
            }
            VertexSet* cset = curr->vset();
            for(VertexSet::const_iterator fit = cset->begin(); fit != cset->end(); fit++)
            {
                VoronoiVertex* cand = *fit;
                for(VertexSet::const_iterator bit = bset->begin(); bit != bset->end(); bit++)
                {
                    VoronoiVertex* ref = *bit;
                    double dist = (ref->position() - cand->position()).squaredNorm();
                    if(dist < best)
                    {
                        best = dist;
                        b1 = ref;
                        b2 = cand;
                    }
                }
            }
        }
        if(b1 && b2)
        {
            VoronoiEdge* e = new VoronoiEdge(b1, b2);
            _edges.insert(e);

            Component* mergeable = (_regions->find(b2->component()))->second;
            VertexSet* mergset = mergeable->vset();
            for(VertexSet::const_iterator it = mergset->begin(); it != mergset->end(); it++)
            {
                biggest->vset()->insert(*it);
            }
            _regions->erase(mergeable->id());
            merged = true;
        }
    }
    biggest->setId(0);
    _regions->insert(make_pair(biggest->id(), biggest));
}


EdgeSet VoronoiDiagram::vertexEdges(VoronoiVertex* v)
{
    EdgeSet returned;
    for(EdgeSet::iterator it = _edges.begin(); it != _edges.end(); it++)
    {
        VoronoiEdge* e = *it;
        if(e->from() == v || e->to() == v)
        {
            returned.insert(e);
        }
    }
    return returned;
}


void VoronoiDiagram::createObservations()
{
    float sx = 0;
    float sy = 0;
    float cth = 0;
    float sth = 0;
    for(VertexMap::const_iterator it = _candidates.begin(); it != _candidates.end(); it++)
    {
        //I have the vertex and a laser
        VoronoiVertex* v = it->second;
        float vx = v->position().x();
        float vy = v->position().y();

        vector<double> ranges;
        ranges.clear();
        VoronoiLaser* l = new VoronoiLaser;

        cv::Mat mah = _map.clone();
        // ... cast the lines
        float max = (float) l->maxRange();
        float incr = (float) l->angularStep();
        float firstBeamAngle = (float) l->firstBeamAngle();
        float fov = (float) l->fov();

        for(float th = firstBeamAngle; th < fov; th+= incr)
        {
            sx = 0;
            sy = 0;

            cth = cos(th);
            sth = sin(th);

            // Step increment along line direction
            float dx = cth;// It was _mapResolution*cth;
            float dy = sth;// It was _mapResolution*sth;

            // Distance from the initial vertex v
            double distance = 0;
            while(distance < max)
            {
                sx += dx;
                sy += dy;

                int ix = sx + vx/*lrint(sx + vx)*/;
                int iy = sy + vy/*lrint(sy + vy)*/;
                //                if(ix >= 0 && iy >= 0 && ix < _map.rows && iy < _map.cols)
                //                {
                if(mah.at<uchar>(ix, iy) == 0)
                {
                    distance = sqrt(sx*sx + sy*sy)*_mapResolution;
                    break;
                }
                //                }
            }
            ranges.push_back(distance);
        }
        Isometry2d lpose = Isometry2d::Identity();
        lpose.translation() = Vector2d(v->graphPose().x(), v->graphPose().y());
        lpose.linear() = Rotation2Dd(v->graphPose().z()).toRotationMatrix();
        l->setPose(lpose);
        l->setRanges(ranges);

        v->setData(l);
    }
}


void VoronoiDiagram::save2g2o(ostream& os, bool sparse)
{
    if(!sparse)
    {
        this->denseGraphExtraction();
    }

    map<int, VoronoiVertex*> savedVertices;
    set<VoronoiEdge*> savedEdges;

    int id = 0;
    for(VertexMap::iterator it = _candidates.begin(); it != _candidates.end(); it++)
    {
        VoronoiVertex* v = it->second;
        EdgeSet myEdges = vertexEdges(v);
        if(myEdges.size() > 1)
        {
            v->setId(id++);
            savedVertices.insert(make_pair(v->id(), v));
        }
    }

    for(map<int, VoronoiVertex*>::const_iterator it = savedVertices.begin(); it != savedVertices.end(); it++)
    {
        VoronoiVertex* v = it->second;
        EdgeSet ves = vertexEdges(v);
        for(set<VoronoiEdge*>::iterator it2 = ves.begin(); it2 != ves.end(); it2++)
        {
            VoronoiEdge* e = *it2;
            if((savedVertices.find(e->to()->id()) != savedVertices.end()) && (savedVertices.find(e->from()->id()) != savedVertices.end()))
            {
                savedEdges.insert(*it2);
            }
        }
    }

    for(map<int, VoronoiVertex*>::const_iterator it = savedVertices.begin(); it != savedVertices.end(); it++)
    {
        VoronoiVertex* v = it->second;
        saveVertex(os, v);
    }
    for(set<VoronoiEdge*>::const_iterator it = savedEdges.begin(); it != savedEdges.end(); it++)
    {
        VoronoiEdge* e = *it;
        saveEdge(os, e);
    }
}


bool VoronoiDiagram::saveEdge(std::ostream& os, VoronoiEdge* e)
{
    if(e)
    {
        os << "EDGE_SE2 ";
        e->write(os);
        os << endl;
        return os.good();
    }
    return false;
}


bool VoronoiDiagram::saveData(ostream &os, VoronoiData* d)
{
    if(d)
    {
        os << d->_tag << " ";
        d->write(os);
        os << endl;
        return os.good();
    }
    return false;
}


bool VoronoiDiagram::saveVertex(ostream& os, VoronoiVertex* v)
{
    if(v)
    {
        os << "VERTEX_SE2 ";
        v->write(os);
        os << endl;
        if(v->data())
        {
            saveData(os, v->data());
            Vector3d vp = v->graphPose();
            os << "VERTEX_TAG " << v->id() << " 0 0 0 " << vp.x() << " " << vp.y() << " " << vp.z() << " 0.0 hostname 0.0 " << endl;
        }
        return os.good();
    }
    return false;
}


void VoronoiDiagram::loadPGM()
{
    //    string tag;
    //    _file >> tag;
    //    if(tag != "P5")
    //    {
    //        cerr << "Awaiting 'P5' in pgm header, found " << tag << endl;
    //        exit(-1);
    //    }

    //    int size_x, size_y;
    //    while(_file.peek() == ' ' || _file.peek() == '\n')
    //    {
    //        _file.ignore();
    //    }
    //    while(_file.peek() == '#')
    //    {
    //        _file.ignore(255, '\n');
    //    }
    //    _file >> size_x;
    //    while(_file.peek() == '#')
    //    {
    //        _file.ignore(255, '\n');
    //    }
    //    _file >> size_y;
    //    while(_file.peek() == '#')
    //    {
    //        _file.ignore(255, '\n');
    //    }
    //    _file >> tag;
    //    if(tag != "255")
    //    {
    //        cerr << "Awaiting '255' in pgm header, found " << tag << endl;
    //        exit(-1);
    //    }

    //    double a = get_time();
    //    _distmap = new DistanceMap(size_x, size_y);
    //    double b = get_time();
    //    cout << "Creation of _distmap: " << b-a << endl;
    //    for(int y = 0; y < size_y; ++y)
    //    {
    //        for(int x = 0; x < size_x; ++x)
    //        {
    //            float c = _file.get();
    //            (*_distmap)(x, y).setPosition(x, y);
    //            (*_distmap)(x, y).setValue(c);
    //            (*_distmap)(x, y).setNearest(x, y);
    //            if(c == 0)
    //            {
    //                (*_distmap)(x, y).setDistance(0);
    //                (*_distmap)(x, y).setParent(x, y);
    //            }
    //            else
    //            {
    //                (*_distmap)(x, y).setDistance(INF);
    //                (*_distmap)(x, y).setParent(INF, INF);
    //            }
    //        }
    //    }
}


void VoronoiDiagram::savePGM(const char *filename, const MatrixXf& image_)
{
    FILE* F = fopen(filename, "w");
    if(!F)
    {
        cerr << "could not open 'result.pgm' for writing!" << endl;
        return;
    }

    const int size_x = image_.rows();
    const int size_y = image_.cols();

    fprintf(F, "P5\n");
    fprintf(F, "#CREATOR: Voronoi Extractor\n");
    fprintf(F, "%d %d\n", size_x, size_y);
    fprintf(F, "255\n");

    float maxval = 0;
    for(int y = 0; y < size_y; ++y)
    {
        for(int x = 0; x < size_x; ++x)
        {
            maxval = maxval > image_(x,y) ? maxval : image_(x,y);
        }
    }

    float scale = 255/sqrt(maxval);

    for(int y = 0; y < size_y; ++y)
    {
        for(int x = 0; x < size_x; ++x)
        {
            unsigned char c = scale*sqrt(image_(x,y));
            fputc(c, F);
        }
    }
    //    cerr << "MAXVAL:" << maxval << endl;
    fclose(F);
}


















void VoronoiDiagram::newinit(const cv::Mat& img_)
{
    _map = img_.clone();

    _rows = img_.rows;
    _cols = img_.cols;

    cv::Mat converted;
    img_.convertTo(converted, CV_32FC1);

    double min = -1e9;
    double max = 1e9;
    minMaxIdx(converted, &min, &max);

    cout << "min: " << min << endl;
    cout << "max: " << max << endl;

    cv::Mat a = cv::Mat(5, 5, CV_8UC1);
    for(int c = 0; c < a.cols; c++)
    {
        for(int r = 0; r < a.rows; r++)
        {
            a.at<uchar>(r, c) = c * a.rows + r;
            cout << " " << (short int) a.at<uchar>(r, c);
        }
        cout << endl;
    }

    for(int c = 1; c < a.cols-1; c++)
    {
        const uchar* rp = a.ptr<uchar>(c-1);
        const uchar* r = a.ptr<uchar>(c);
        const uchar* rd = a.ptr<uchar>(c+1);

        cout << "rp1: " << (int*) rp << ", " << (int) *rp << endl;
        rp++;
        cout << "rp11: " << (int*) rp << ", " << (int) *rp << endl;
        cout << "rp2: " << (int*) r << ", " << (int) *r << endl;
        cout << "rp3: " << (int*) rd << ", " << (int) *rd << endl;

        for(int r = 1; r < a.rows-1; r++)
        {
//            cout << " " << (short int) a.at<uchar>(r, c);
        }
        cout << endl;
    }

//    for(int y = 1; y < _rows-1; y++)
//    {
//        const float* row_up = converted.ptr<float>(y-1);
//        const float* row = converted.ptr<float>(y);
//        const float* row_down = converted.ptr<float>(y+1);

//        for(int x = 1; x < _cols-1; x++)
//        {
//        }
//    }
}






