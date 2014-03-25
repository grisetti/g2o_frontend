#include "voronoi_diagram.h"


#define INF 1000000


using namespace std;
using namespace Eigen;



VoronoiDiagram::VoronoiDiagram(const cv::Mat& input, int squaredResolution_) : _squaredResolution(squaredResolution_)
{
    _distmap = 0;
    _vQueue = 0;
    _voro = 0;

    _graph = 0;
    _dmap = 0;

    this->init(input);
}


VoronoiDiagram::~VoronoiDiagram()
{
    delete[] _dmap;
    delete _distmap;
    delete _voro;
    delete _graph;
    delete _vQueue;
}


void VoronoiDiagram::checkQueue()
{
    cout << "CHECKING QUEUE SIZE: " << _vQueue->size() << endl;
    for(VertexMap::const_iterator it = _vMap.begin(); it != _vMap.end(); ++it)
    {
        VoronoiVertex* v = it->second;
        _vQueue->push(v);
    }
    VoronoiQueue tmp = *_vQueue;
    while(!tmp.empty())
    {
        VoronoiVertex* v = tmp.top();
        tmp.pop();

        cout << v->distance() << "; " << v->position().x() << ", " << v->position().y() << endl;
    }
}


void VoronoiDiagram::checkStats()
{
    int visited_counter = 0;
    int merged_counter = 0;
    for(VertexMap::const_iterator it = _vMap.begin(); it != _vMap.end(); ++it)
    {
        VoronoiVertex* v = it->second;
        if(v->visited())
        {
            visited_counter++;
        }
        if(v->merged())
        {
            merged_counter++;
        }
    }
    cout << "VMAP VERTICES: " << _vMap.size() << endl;
    cout << "CANDIDATES VERTICES: " << _candidates.size() << endl;
    cout << "TOTAL VISITED: " << visited_counter << endl;
    cout << "TOTAL MERGED: " << merged_counter << endl;
}


void VoronoiDiagram::morphThinning(cv::Mat &src, cv::Mat &dst, bool binarize, uchar thresh)
{
    if(src.depth() != cv::DataType<uchar>::type || src.channels() != 1 || !src.rows || !src.cols)
    {
        throw std::invalid_argument("Unsupported input image");
    }

    cv::Mat bin_img;
    if(binarize)
    {
        bin_img = cv::Mat(src.size(), CV_8UC1);
        cv::threshold(src, bin_img, double(thresh), 1, cv::THRESH_BINARY);
    }
    else
    {
        bin_img = src;
    }

    dst = bin_img.clone();

    int n_deleted;
    do
    {
        n_deleted = 0;
        for( int iter = 0; iter < 2; iter++)
        {
            for( int y = 1; y < bin_img.rows - 1; y++)
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

                uchar *dst_p = dst.ptr<uchar>(y);
                dst_p++;

                p2 = *row_up_p++;
                p1 = *row_p++;
                p6 = *row_down_p++;

                p3 = *row_up_p++;
                p4 = *row_p++;
                p5 = *row_down_p++;

                for(int x = 1; x < bin_img.cols - 1; x++, dst_p++)
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
                            *dst_p = 0;
                            n_deleted++;
                        }
                    }
                }
            }
            dst.copyTo(bin_img);
        }
    }
    while(n_deleted);

    dst *= 255;
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


void VoronoiDiagram::graphExtraction(cv::Mat& skeleton, vector<cv::Point2f>& nodes, vector<vector<int> >& edges, bool find_leafs, int min_dist)
{
    if(skeleton.depth() != cv::DataType<uchar>::type || skeleton.channels() != 1 || !skeleton.rows || !skeleton.cols)
    {
        throw std::invalid_argument("Unsopported input image");
    }

    if(min_dist < 1)
    {
        cout << "graphExtraction() : Warning! min_dist < 1, using default value 1" << endl;
        min_dist = 1;
    }

    cv::Mat index_mat = cv::Mat(skeleton.size(), CV_32SC1, cvRealScalar(-1)),
            mask = cv::Mat::zeros(skeleton.size(), CV_8UC1), tmp_skel;

    shrinkIntersections(skeleton, tmp_skel);

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


void VoronoiDiagram::findNeighborNodes( int src_node_idx, int x, int y, cv::Mat &mask, cv::Mat &index_mat,
                                        std::vector<cv::Point2f> &nodes,
                                        std::vector< std::vector<int> > &edges )
{
    if( !mask.at<uchar>(y,x) )
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

    for( int tx = std::max<int>(0, x - 1); tx <= std::min<int>(mask.cols, x + 1); tx++)
        for( int ty = std::max<int>(0, y - 1); ty <= std::min<int>(mask.rows, y + 1); ty++)
            if( (tx != x || ty != y ) && mask.at<uchar>(ty,tx))
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


void VoronoiDiagram::distmapExtraction()
{
    const short int coords_x[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
    const short int coords_y[8] = {-1,  0,  1, -1, 1, -1, 0, 1};

    while(!_vQueue->empty())
    {
        VoronoiVertex* current = _vQueue->top();
        _vQueue->pop();

        const int current_x = current->position().x();
        const int current_y = current->position().y();

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
            //            if((*_distmap)(x, y).distance() != 0)
            int k = r + cr;
            if(_dmap[k].distance() != 0)
            {
                //                VoronoiVertex* current = &(*_distmap)(x, y);
                VoronoiVertex* current = &(_dmap[k]);
                for(short int i = 0; i < 8; ++i)
                {
                    const int neighbor_x = r + coords_x[i];
                    const int neighbor_y = c + coords_y[i];

                    int nk = neighbor_x + neighbor_y * _rows;
                    if((neighbor_x >= 0) && (neighbor_y >= 0) && (neighbor_x < _rows) && (neighbor_y < _cols) && (_dmap[k].distance() != 0))
                        //                    if((neighbor_x >= 0) && (neighbor_y >= 0) && (neighbor_x < _rows) && (neighbor_y < _cols) && ((*_distmap)(neighbor_x, neighbor_y).distance() != 0))
                    {
                        //                        VoronoiVertex* neighbor = &(*_distmap)(neighbor_x, neighbor_y);
                        VoronoiVertex* neighbor = &(_dmap[nk]);

                        /*GOOD VORONOI TO WORK ON, BAD RECONSTRUCTED MAP*/
                        float pdist = (current->parent() - neighbor->parent()).squaredNorm();
                        if(pdist > _squaredResolution)
                        {
                            //                        cv::line(*_voro, cv::Point(y, x), cv::Point(current->_parent.y(), current->_parent.x()), cv::Scalar(127));
                            //                        for(int p = 0; p < 9; ++p)
                            //                        {
                            //                            int drawable_x = current->_parent.x()+coords_x[p];
                            //                            int drawable_y = current->_parent.y()+coords_y[p];

                            //                            // Draw obstacles
                            //                            if((drawable_x >= 0) && (drawable_y >= 0) && (drawable_x < rows) && (drawable_y < cols))
                            //                            {
                            //                                _voro->at<uchar>(drawable_x, drawable_y) = 50;
                            //                            }
                            //                        }
                            _voro->at<uchar>(r, c) = 255;
                            _vMap.insert(make_pair(current->position(), current));
                        }

                        /*BAD VORONOI TO WORK ON, GOOD RECONSTRUCTED MAP*/
                        //                    float ndist = (current->_position-neighbor->_position).squaredNorm();
                        //                    if(ndist < pdist)
                        //                    {
                        //                        voromap(current->_position.x(), current->_position.y()) = 0;
                        //                        for(int p = 0; p < 9; ++p)
                        //                        {
                        //                            int drawable_x = current->_parent.x()+coords_x[p];
                        //                            int drawable_y = current->_parent.y()+coords_y[p];

                        //                            if((drawable_x >= 0) && (drawable_y >= 0) && (drawable_x < rows) && (drawable_y < cols))
                        //                                voromap(drawable_x, drawable_y) = 80;
                        //                        }
                        //                    }
                    }
                }
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


void VoronoiDiagram::distmap2image()
{
    _drawableDistmap = MatrixXf::Zero(_cols, _rows);

    for(int c = 0; c < _cols; ++c)
    {
        int cr = c * _rows;
        for(int r = 0; r < _rows; ++r)
        {
            int k = r + cr;
            //            _drawableDistmap(x, y) = (*_distmap)(x, y).distance();
            _drawableDistmap(c, r) = _dmap[k].distance();
        }
    }
}


void VoronoiDiagram::init(const cv::Mat& img_)
{
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
