#include "voronoi_diagram.h"


#define INF 1000000


using namespace std;
using namespace Eigen;


VoronoiDiagram::VoronoiDiagram(istream& file_, int squaredResolution_) : _squaredResolution(squaredResolution_), _file(file_)
{
    _distmap = 0;
    _vQueue = 0;
    _voro = 0;
}


/** UNUSED CTOR. WILL BE DELETED*/
//VoronoiDiagram::VoronoiDiagram(istream& file_, int squaredResolution_, int thetaRes_, float rhoRes_) : _squaredResolution(squaredResolution_), _thetaRes(thetaRes_), _rhoRes(rhoRes_), _file(file_)
//{
//    _distmap = 0;
//    _vQueue = 0;
//    _lut = 0;
//    _tit = 0;
//    _voro = 0;
//}


VoronoiDiagram::~VoronoiDiagram(){}


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


/** ALL THE VERTICES ARE NOT VISITED AND NOT MERGED*/
/** AFTER THE EXECUTION OF THIS FUNCTION, ALL THE VERTICES ARE VISITED, EVEN IN PRESENCE OF DISCONNECTED COMPONENTS*/
void VoronoiDiagram::diagram2graph()
{
    const int rows = _distmap->rows();
    const int cols = _distmap->cols();

    const short coords_x[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
    const short coords_y[8] = {-1,  0,  1, -1, 1, -1, 0, 1};

    VoronoiQueue tmp;
    bool firstVertex = true;
    for(VertexMap::const_iterator it = _vMap.begin(); it != _vMap.end(); it++)
    {
        VoronoiVertex* first = it->second;
        if(firstVertex)
        {
            _candidates.insert(VoronoiPair(first->position(), first));
            first->setNearest(first->position());
            firstVertex = false;
        }
        if(!first->visited())
        {
            first->setVisited();
            tmp.push(first);
        }
        while(!tmp.empty())
        {
            VoronoiVertex* current = tmp.top();
            tmp.pop();

            const int cx = current->position().x();
            const int cy = current->position().y();

            for(short int i = 0; i < 8; ++i)
            {
                const int nx = cx + coords_x[i];
                const int ny = cy + coords_y[i];

                if((nx >= 0) && (ny >= 0) && (nx < rows) && (ny < cols) && ((*_voro).at<uchar>(nx, ny) != 0) && (!((*_distmap)(nx, ny).visited())))
                {
                    VoronoiVertex* neighbor = &(*_distmap)(nx, ny);
                    neighbor->setVisited();
                    tmp.push(neighbor);

                    double dist = (neighbor->position() - current->nearest()).squaredNorm();
                    if(dist <= _squaredResolution*5)
                    {
                        neighbor->setNearest(current->nearest());
                    }
                    else
                    {
                        neighbor->setNearest(neighbor->position());
                        _candidates.insert(VoronoiPair(neighbor->position(), neighbor));
                    }
                }
            }
        }
    }
}
//maybe priority_queue is not the best choice
//void VoronoiDiagram::diagram2graph()
//{
//    const int rows = _distmap->rows();
//    const int cols = _distmap->cols();

//    const short coords_x[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
//    const short coords_y[8] = {-1,  0,  1, -1, 1, -1, 0, 1};

//    VoronoiQueue tmp;
//    Vector2i lastPosition;
//    bool firstVertex = true;
//    for(VertexMap::const_iterator it = _vMap.begin(); it != _vMap.end(); it++)
//    {
//        VoronoiVertex* first = it->second;
//        if(firstVertex)
//        {
//            _candidates.insert(VoronoiPair(first->position(), first));
//            lastPosition = first->position();
//            firstVertex = false;
//        }
//        if(!first->visited())
//        {
//            first->setVisited();
//            tmp.push(first);
//        }
//        while(!tmp.empty())
//        {
//            VoronoiVertex* current = tmp.top();
//            tmp.pop();

//            const int cx = current->position().x();
//            const int cy = current->position().y();

//            for(short int i = 0; i < 8; ++i)
//            {
//                const int nx = cx + coords_x[i];
//                const int ny = cy + coords_y[i];

//                if((nx >= 0) && (ny >= 0) && (nx < rows) && (ny < cols) && ((*_voro).at<uchar>(nx, ny) != 0) && (!((*_distmap)(nx, ny).visited())))
//                {
//                    VoronoiVertex* neighbor = &(*_distmap)(nx, ny);
//                    neighbor->setVisited();
//                    tmp.push(neighbor);

//                    double dist = (neighbor->position() - lastPosition).squaredNorm();
//                    if(dist > 3250)
//                    {
//                        _candidates.insert(VoronoiPair(neighbor->position(), neighbor));
//                        lastPosition = neighbor->position();
//                    }
//                }
//            }
//        }
//    }
//}


void VoronoiDiagram::graph()
{
    _graph = new cv::Mat(_distmap->rows(), _distmap->cols(), CV_8UC3, cv::Scalar(0, 0, 0));
    for(VertexMap::iterator it = _candidates.begin(); it != _candidates.end(); ++it)
    {
        VoronoiVertex* current = it->second;
        int cx = current->position().x();
        int cy = current->position().y();

        _graph->at<cv::Vec3b>(cv::Point(cy, cx)) = cv::Vec3b(255, 255, 255);
    }
}


void VoronoiDiagram::filter()
{
    const int rows = _distmap->rows();
    const int cols = _distmap->cols();

    const short coords_x[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
    const short coords_y[8] = {-1,  0,  1, -1, 1, -1, 0, 1};

    VoronoiQueue tmp;
    for(VertexMap::iterator it = _vMap.begin(); it != _vMap.end(); it++)
    {
        VoronoiVertex* first = it->second;
        if(!first->visited())
        {
            tmp.push(first);
        }
        while(!tmp.empty())
        {
            VoronoiVertex* current = tmp.top();
            tmp.pop();

            const int cx = current->position().x();
            const int cy = current->position().y();

            for(short int i = 0; i < 8; ++i)
            {
                const int nx = cx + coords_x[i];
                const int ny = cy + coords_y[i];

                if((nx >= 0) && (ny >= 0) && (nx < rows) && (ny < cols) && ((*_voro).at<uchar>(nx, ny) != 0) && (!((*_distmap)(nx, ny).visited())))
                {
                    VoronoiVertex* neighbor = &(*_distmap)(nx, ny);
                    neighbor->setVisited();
                    if(current->distance() == neighbor->distance())
                    {
                        _vMap.erase(neighbor->position());
                    }
                    tmp.push(neighbor);
                }
            }
        }
    }
}


void VoronoiDiagram::distmap()
{
    const int rows = _distmap->rows();
    const int cols = _distmap->cols();

//    const short int coords_x[4] = {-1, 1,  0, 0};
//    const short int coords_y[4] = { 0, 0, -1, 1};

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
            if((nx >= 0) && (ny >= 0) && (nx < rows) && (ny < cols) && ((*_distmap)(nx, ny).distance() != 0))
            {
                VoronoiVertex* neighbor = &(*_distmap)(nx, ny);
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


void VoronoiDiagram::distmap2voronoi()
{
    _voro = new cv::Mat(_distmap->rows(), _distmap->cols(), CV_8UC1, cv::Scalar(0));

    const int rows = _distmap->rows();
    const int cols = _distmap->cols();

//    const short int coords_x[4] = {-1, 1,  0, 0};
//    const short int coords_y[4] = { 0, 0, -1, 1};

    const short int coords_x[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
    const short int coords_y[8] = {-1,  0,  1, -1, 1, -1, 0, 1};

    for(int y = 0; y < cols; ++y)
    {
        for(int x = 0; x < rows; ++x)
        {
            if((*_distmap)(x, y).distance() != 0)
            {
                VoronoiVertex* current = &(*_distmap)(x, y);
                for(short int i = 0; i < 8; ++i)
                {
                    const int neighbor_x = x + coords_x[i];
                    const int neighbor_y = y + coords_y[i];

                    if((neighbor_x >= 0) && (neighbor_y >= 0) && (neighbor_x < rows) && (neighbor_y < cols) && ((*_distmap)(neighbor_x, neighbor_y).distance() != 0))
                    {
                        VoronoiVertex* neighbor = &(*_distmap)(neighbor_x, neighbor_y);

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
                            _voro->at<uchar>(x, y) = 255;
                            _vMap.insert(VoronoiPair(current->position(), current));
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


void VoronoiDiagram::queueFiller()
{
    _vQueue = new VoronoiQueue;

    const int rows = _distmap->rows();
    const int cols = _distmap->cols();

//    const short int coords_x[4] = {1, 0, -1,  0};
//    const short int coords_y[4] = {0, 1,  0, -1};

    const short int coords_x[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
    const short int coords_y[8] = {-1,  0,  1, -1, 1, -1, 0, 1};

    for(int c = 0; c < cols; ++c)
    {
        for(int r = 0; r < rows; ++r)
        {
            VoronoiVertex* obstacle = &((*_distmap)(r, c));

            const int obstacle_x = obstacle->position().x();
            const int obstacle_y = obstacle->position().y();

            bool counter = false;
            for(short int i = 0; (i < 8) && (counter == false); ++i)
            {
                int neighbor_x = obstacle_x + coords_x[i];
                int neighbor_y = obstacle_y + coords_y[i];

                // If at least one of the neighbors is a free cell, put the current obstacle into the queue
                if((neighbor_x >= 0) && (neighbor_y >= 0) && (neighbor_x < rows) && (neighbor_y < cols) && ((*_distmap)(neighbor_x, neighbor_y).distance() != 0))
                {
                    counter = true;
                    _vQueue->push(obstacle);
                }
            }
        }
    }
}


void VoronoiDiagram::distmap2image()
{
    _drawableDistmap = MatrixXf::Zero(_distmap->rows(), _distmap->cols());
    const unsigned int rows = _distmap->rows();
    const unsigned int cols = _distmap->cols();

    for(unsigned int y = 0; y < cols; ++y)
    {
        for(unsigned int x = 0; x < rows; ++x)
        {
            _drawableDistmap(x, y) = (*_distmap)(x, y).distance();
        }
    }
}


void VoronoiDiagram::loadPGM()
{
    string tag;
    _file >> tag;
    if(tag != "P5")
    {
        cerr << "Awaiting 'P5' in pgm header, found " << tag << endl;
        exit(-1);
    }

    int size_x, size_y;
    while(_file.peek() == ' ' || _file.peek() == '\n')
    {
        _file.ignore();
    }
    while(_file.peek() == '#')
    {
        _file.ignore(255, '\n');
    }
    _file >> size_x;
    while(_file.peek() == '#')
    {
        _file.ignore(255, '\n');
    }
    _file >> size_y;
    while(_file.peek() == '#')
    {
        _file.ignore(255, '\n');
    }
    _file >> tag;
    if(tag != "255")
    {
        cerr << "Awaiting '255' in pgm header, found " << tag << endl;
        exit(-1);
    }

    double a = get_time();
    _distmap = new DistanceMap(size_x, size_y);
    double b = get_time();
    cout << "Creation of _distmap: " << b-a << endl;
    for(int y = 0; y < size_y; ++y)
    {
        for(int x = 0; x < size_x; ++x)
        {
            float c = _file.get();
            (*_distmap)(x, y).setPosition(x, y);
            (*_distmap)(x, y).setValue(c);
            (*_distmap)(x, y).setNearest(x, y);
            if(c == 0)
            {
                (*_distmap)(x, y).setDistance(0);
                (*_distmap)(x, y).setParent(x, y);
            }
            else
            {
                (*_distmap)(x, y).setDistance(INF);
                (*_distmap)(x, y).setParent(INF, INF);
            }
        }
    }
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



/** UNUSED FUNCTIONS. THEY WILL BE DELETED*/
//void VoronoiDiagram::fillLookUpTable(int dTh, float dRho, int maxDist)
//{
//    int cols_half = dTh;
//    float angular_step = M_PI/dTh;
//    int rows = maxDist/dRho+1;
//    cout << "MAX DIST IS: " << maxDist << endl;
//    _lut = new LookUpTable(rows, cols_half*2+1);

//    for(float j = -cols_half; j <= cols_half; ++j)
//    {
//        float th = angular_step*j;
//        for(float i = 0; i < rows; ++i)
//        {
//            float rho = dRho*i;
//            (*_lut)(i, j+cols_half) = Vector2f(cos(th)*rho, sin(th)*rho);
//        }
//    }
//}


//void VoronoiDiagram::testLookUpTable(int dTh, float dRho, int maxDist)
//{
//    int cols_half = dTh;
//    float angular_step = M_PI/dTh;
//    int rows = maxDist/dRho+1;
//    _lut = new LookUpTable(rows, cols_half*2+1);

//    for(float j = -cols_half; j <= cols_half; ++j)
//    {
//        float th = angular_step*j;
//        for(float i = 0; i < rows; ++i)
//        {
//            float rho = dRho*i;
//            (*_lut)(i, j+cols_half) = Vector2f(rho, th);
//        }
//    }

//    cout << "dumping s.l.u.t." << endl;
//    for(int i = 0; i < rows; ++i)
//    {
//        for(int j = 0; j < cols_half*2+1; ++j)
//        {
//            cout << "(" << (*_lut)(i, j).x() << ", " << (*_lut)(i, j).y() << ")";
//        }
//        cout << endl;
//    }
//}


//void VoronoiDiagram::titSeeker(const Vector2i &mouse_coords)
//{
//    int tit_rows = _lut->cols();
//    int tit_cols = _lut->rows();

//    _tit = new MatrixXf(MatrixXf::Zero(tit_rows, tit_cols));

//    cout << "# of rows: " << tit_rows << endl;
//    cout << "# of cols: " << tit_cols << endl;

//    for(int j = 0; j < tit_rows; j++)
//    {
//        for(int i = 0; i < tit_cols; i++)
//        {
//            Vector2i ilut((*_lut)(i, j).x(), (*_lut)(i, j).y());
//            Vector2i tit_coords = ilut + mouse_coords;
//            //            (*_tit)(j, (tit_cols - 1) - i) = (*_voro).at<uchar>(tit_coords.x(), tit_coords.y());
//            (*_tit)(j, (tit_cols - 1) - i) = _testVMap(tit_coords.x(), tit_coords.y());
//        }
//    }
//}


//void VoronoiDiagram::proxySetter()
//{
//    int tit_rows = _lut->cols();
//    int tit_cols = _lut->rows();

//    int counter = 0;
//    cout << "SIZE into proxy setter: " << _vMap.size() << endl;
//    for(VertexMap::iterator it = _vMap.begin(); it != _vMap.end(); ++it)
//    {
//        MatrixXf tmp = MatrixXf::Zero(tit_rows, tit_cols);
//        for(int j = 0; j < tit_rows; j++)
//        {
//            for(int i = 0; i < tit_cols; i++)
//            {
//                Vector2i ilut((*_lut)(i, j).x(), (*_lut)(i, j).y());
////                Vector2i tit_coords = ilut + (it->second).position();
//                Vector2i tit_coords = ilut + (it->second)->position();
//                //                tmp(j, (tit_cols - 1) - i) = (*_voro).at<uchar>(tit_coords.x(), tit_coords.y());
//                tmp(j, (tit_cols - 1) - i) = _testVMap(tit_coords.x(), tit_coords.y());
//            }
//        }
////        (it->second).setProxy(&tmp);
//        (it->second)->setProxy(&tmp);
//        counter++;
//        //        std::stringstream sstm;
//        //        string prefix = "images/";
//        //        string name = ".pgm";
//        //        sstm << prefix << counter << name;
//        //        string result = sstm.str();
//        //        savePGM(result.c_str(), tmp);
//    }
//}
