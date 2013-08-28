#include "voronoi_diagram.h"

#define INF 1000000

using namespace std;
using namespace Eigen;



VoronoiDiagram::VoronoiDiagram(istream& file_, int squaredResolution_, int thetaRes_, float rhoRes_) : _file(file_), _squaredResolution(squaredResolution_), _thetaRes(thetaRes_), _rhoRes(rhoRes_)
{
    _distmap = 0;
    _distqueue = 0;
    _lut = 0;
    _tit = 0;
    _voro = 0;



    _posqueue = 0;
}


VoronoiDiagram::~VoronoiDiagram(){}


void VoronoiDiagram::newQueueFiller()
{
    _posqueue = new PositionQueue;

    const int rows = _input.rows();
    const int cols = _input.cols();

    const short int coords_x[4] = {1, 0, -1,  0};
    const short int coords_y[4] = {0, 1,  0, -1};

    for(int y = 0; y < cols; ++y)
    {
        for(int x = 0; x < rows; ++x)
        {
            Vector2i* v;
            bool counter = false;
            for(short int i = 0; (i < 4) && (counter == false); ++i)
            {
                int r = coords_x[i];
                int c = coords_y[i];

                int neighbor_x = x + r;
                int neighbor_y = y + c;

                //Out of boundaries
                if((neighbor_x >= 0) && (neighbor_y >= 0) && (neighbor_x < rows) && (neighbor_y < cols) && (_input(neighbor_x, neighbor_y) != 0))
                {
                    counter = true;
                    _posqueue->push(v);
                }
            }
        }
    }
}


//void VoronoiDiagram::queueFiller()
//{
//    _distqueue = new DistanceQueue;

//    const int rows = _distmap->rows();
//    const int cols = _distmap->cols();

//    const short int coords_x[4] = {1, 0, -1,  0};
//    const short int coords_y[4] = {0, 1,  0, -1};

//    for(int y = 0; y < cols; ++y)
//    {
//        for(int x = 0; x < rows; ++x)
//        {
//            VoronoiVertex* v = &((*_distmap)(x, y));

//            const int v_x = v->_position.x();
//            const int v_y = v->_position.y();

//            bool counter = false;
//            for(short int i = 0; (i < 4) && (counter == false); ++i)
//            {
//                int r = coords_x[i];
//                int c = coords_y[i];

//                int neighbor_x = v_x + r;
//                int neighbor_y = v_y + c;

//                //Out of boundaries
//                if((neighbor_x >= 0) && (neighbor_y >= 0) && (neighbor_x < rows) && (neighbor_y < cols) && ((*_distmap)(neighbor_x, neighbor_y)._distance != 0))
//                {
//                    counter = true;
//                    _distqueue->push(v);
//                }
//            }
//        }
//    }
//}


void VoronoiDiagram::newDistmap()
{
//    const int rows = _input.rows();
//    const int cols = _input.cols();

//    const short int coords_x[4] = {-1, 1,  0, 0};
//    const short int coords_y[4] = { 0, 0, -1, 1};

//    while(!_posqueue->empty())
//    {
//        Vector2i current = _posqueue->top();
//        _posqueue->pop();

//        const int current_x = current.x();
//        const int current_y = current.y();

//        for(short int i = 0; i < 4; ++i)
//        {
//            const short int r = coords_x[i];
//            const short int c = coords_y[i];

//            int nc_x = current_x + r;
//            int nc_y = current_y + c;

//            //Check if into boundaries or not an obstacle
//            if((nc_x >= 0) && (nc_y >= 0) && (nc_x < rows) && (nc_y < cols) && (_input(nc_x, nc_y) != 0))
//            {
//                VoronoiVertex* neighbor = &(*_distmap)(nc_x, nc_y);
//                Vector2i nc(nc_x, nc_y);

//                double neighbor_to_current_parent_dist = (nc - current->_parent).squaredNorm();
//                if(neighbor_to_current_parent_dist < neighbor->_distance)
//                {
//                    neighbor->_distance = neighbor_to_current_parent_dist;
//                    neighbor->_parent = current->_parent;
//                    _distqueue->push(neighbor);
//                }
//            }
//        }
//    }
}


//void VoronoiDiagram::distmap()
//{
//    const int rows = _distmap->rows();
//    const int cols = _distmap->cols();

//    const short int coords_x[4] = {-1, 1,  0, 0};
//    const short int coords_y[4] = { 0, 0, -1, 1};

//    while(!_distqueue->empty())
//    {
//        VoronoiVertex* current = _distqueue->top();
//        _distqueue->pop();

//        const int current_x = current->_position.x();
//        const int current_y = current->_position.y();

//        for(short int i = 0; i < 4; ++i)
//        {
//            const short int r = coords_x[i];
//            const short int c = coords_y[i];

//            int nc_x = current_x + r;
//            int nc_y = current_y + c;

//            //Check if into boundaries or not an obstacle
//            if((nc_x >= 0) && (nc_y >= 0) && (nc_x < rows) && (nc_y < cols) && ((*_distmap)(nc_x, nc_y)._distance != 0))
//            {
//                VoronoiVertex* neighbor = &(*_distmap)(nc_x, nc_y);
//                Vector2i nc(nc_x, nc_y);

//                double neighbor_to_current_parent_dist = (nc - current->_parent).squaredNorm();
//                if(neighbor_to_current_parent_dist < neighbor->_distance)
//                {
//                    neighbor->_distance = neighbor_to_current_parent_dist;
//                    neighbor->_parent = current->_parent;
//                    _distqueue->push(neighbor);
//                }
//            }
//        }
//    }
//}


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
//            (*_tit)(j, (tit_cols - 1) - i) = (*_voro).at<uchar>(tit_coords.x(), tit_coords.y());
//        }
//    }
//}


//void VoronoiDiagram::proxySetter()
//{
//    int tit_rows = _lut->cols();
//    int tit_cols = _lut->rows();

//    int counter = 0;
//    for(VertexMap::iterator it = _vMap.begin(); it != _vMap.end(); ++it)
//    {
//        MatrixXf tmp = MatrixXf::Zero(tit_rows, tit_cols);
//        for(int j = 0; j < tit_rows; j++)
//        {
//            for(int i = 0; i < tit_cols; i++)
//            {
//                Vector2i ilut((*_lut)(i, j).x(), (*_lut)(i, j).y());
//                Vector2i tit_coords = ilut + (it->second).position();
//                tmp(j, (tit_cols - 1) - i) = (*_voro).at<uchar>(tit_coords.x(), tit_coords.y());
//            }
//        }
//        (it->second).setProxy(&tmp);
//        counter++;
//    }
//}


//void VoronoiDiagram::spikeFinder()
//{
//    cout << "rows: " << _tit->rows() << ", cols: " << _tit->cols() << endl;
//    int range = _tit->rows();

//    int first = (*_tit)(0, 0);
//    int counter = 0;
//    for(int i = 1; i < range; i++)
//    {
//        cout << " " << (*_tit)(i, 0) << endl;
//        if(first != (*_tit)(i, 0))
//        {
//            counter++;
//            first = (*_tit)(i, 0);
//        }
//    }
//    cout << "Spikes: " << ceil(counter/2) << endl;
//}


//void VoronoiDiagram::distmap2voronoi()
//{
//    _voro = new cv::Mat(_distmap->rows(), _distmap->cols(), CV_8UC1, cv::Scalar(0));

//    const int rows = _distmap->rows();
//    const int cols = _distmap->cols();

//    const short int coords_x[] = { 0, 0, -1, 1, -1, -1,  1, 1, 0};
//    const short int coords_y[] = {-1, 1,  0, 0, -1,  1, -1, 1, 0};

////    voromap.fill(255);            VALID IF VOROMAP IS A MATRIXXF
//    for(int y = 0; y < cols; ++y)
//    {
//        for(int x = 0; x < rows; ++x)
//        {
//            VoronoiVertex* current = &(*_distmap)(x, y);
//            for(short int i = 0; i < 4; ++i)
//            {
//                const short int r = coords_x[i];
//                const short int c = coords_y[i];

//                const int neighbor_x = x + r;
//                const int neighbor_y = y + c;

//                if((neighbor_x >= 0) && (neighbor_y >= 0) && (neighbor_x < rows) && (neighbor_y < cols))
//                {
//                    VoronoiVertex* neighbor = &(*_distmap)(neighbor_x, neighbor_y);

//                    /*GOOD VORONOI TO WORK ON, BAD RECONSTRUCTED MAP*/
//                    float pdist = (current->_parent-neighbor->_parent).squaredNorm();
//                    if(pdist > _squaredResolution)
//                    {
////                        cv::line(*_voro, cv::Point(y, x), cv::Point(current->_parent.y(), current->_parent.x()), cv::Scalar(127));
//                        for(int p = 0; p < 9; ++p)
//                        {
//                            int drawable_x = current->_parent.x()+coords_x[p];
//                            int drawable_y = current->_parent.y()+coords_y[p];

//                            // Draw obstacles
////                            if((drawable_x >= 0) && (drawable_y >= 0) && (drawable_x < rows) && (drawable_y < cols))
////                            {
////                                _voro->at<uchar>(drawable_x, drawable_y) = 50;
////                            }
//                        }
//                        _voro->at<uchar>(x, y) = 255;
//                        _vMap.insert(VoronoiPair(Vector2i(x, y), (*_distmap)(x, y)));
//                    }

//                    /*BAD VORONOI TO WORK ON, GOOD RECONSTRUCTED MAP*/
////                    float ndist = (current->_position-neighbor->_position).squaredNorm();
////                    if(ndist < pdist)
////                    {
////                        voromap(current->_position.x(), current->_position.y()) = 0;
////                        for(int p = 0; p < 9; ++p)
////                        {
////                            int drawable_x = current->_parent.x()+coords_x[p];
////                            int drawable_y = current->_parent.y()+coords_y[p];

////                            if((drawable_x >= 0) && (drawable_y >= 0) && (drawable_x < rows) && (drawable_y < cols))
////                                voromap(drawable_x, drawable_y) = 80;
////                        }
////                    }
//                }
//            }
//        }
//    }
//    cv::dilate(*_voro, *_voro, cv::Mat(), cv::Point(-1, -1), 3);
////    cv::erode(*_voro, *_voro, cv::Mat(), cv::Point(-1, -1));
////    cv::blur(*_voro, *_voro, cv::Size(3, 3));
////    cv::erode(*_voro, *_voro, cv::Mat(), cv::Point(-1, -1), 2);
//}


//void VoronoiDiagram::cvmat2eigenmat()
//{
//    _drawableVoromap = MatrixXf::Zero(_distmap->rows(), _distmap->cols());
//    for(int i = 0; i < _voro->rows; ++i)
//    {
//        for(int j = 0; j < _voro->cols; ++j)
//        {
//            _drawableVoromap(i, j) = _voro->at<uchar>(i, j);
//        }
//    }
//}


//void VoronoiDiagram::eroded2eigen()
//{
//    _drawableEroded = MatrixXf::Zero(_distmap->rows(), _distmap->cols());
//    for(int i = 0; i < _voro->rows; ++i)
//    {
//        for(int j = 0; j < _voro->cols; ++j)
//        {
//            _drawableEroded(i, j) = _voro->at<uchar>(i, j);
//        }
//    }
//}

////void VoronoiDiagram::eigenmat2cvmat(cv::Mat &out, const MatrixXf &in)
////{
////    for(int i = 0; i < in.rows(); ++i)
////    {
////        for(int j = 0; j < in.cols(); ++j)
////        {
////            out.at<uchar>(i, j) = in(i, j);
////        }
////    }
////}


//void VoronoiDiagram::distmap2image()
//{
//    _drawableDistmap = MatrixXf::Zero(_distmap->rows(), _distmap->cols());
//    const int rows = _distmap->rows();
//    const int cols = _distmap->cols();

//    for(size_t y = 0; y < cols; ++y)
//    {
//        for(size_t x = 0; x < rows; ++x)
//        {
//            _drawableDistmap(x, y) = (*_distmap)(x, y)._distance;
//        }
//    }
//}


//void VoronoiDiagram::vmap2eigenmat()
//{
//    _testVMap = MatrixXf::Zero(_distmap->rows(), _distmap->cols());
//    _testVMap.fill(255);
//    for(VertexMap::iterator it = _vMap.begin(); it != _vMap.end(); ++it)
//    {
//        VoronoiVertex v = it->second;
//        _testVMap(v.position().x(), v.position().y()) = 0;
//    }
//}


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
            (*_distmap)(x, y)._position = Vector2i(x, y);
            (*_distmap)(x, y)._value = c;

            if(c == 0)
            {
                (*_distmap)(x, y)._distance = 0;
                (*_distmap)(x, y)._parent = Vector2i(x, y);
            }
            else
            {
                (*_distmap)(x, y)._distance = INF;
                (*_distmap)(x, y)._parent = Vector2i(INF, INF);
            }
        }
    }
}


void VoronoiDiagram::newLoad()
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

    _input = MatrixXi::Zero(size_x, size_y);
    for(int y = 0; y < size_y; ++y)
    {
        for(int x = 0; x < size_x; ++x)
        {
            float c = _file.get();
            if(c != 0)
            {
                _input(x, y) = INF;
            }
        }
    }
}


//void VoronoiDiagram::savePGM(const char *filename, const MatrixXf& image_)
//{
//    FILE* F = fopen(filename, "w");
//    if(!F)
//    {
//        cerr << "could not open 'result.pgm' for writing!" << endl;
//        return;
//    }

//    const int size_x = image_.rows();
//    const int size_y = image_.cols();

//    fprintf(F, "P5\n");
//    fprintf(F, "#CREATOR: Voronoi Extractor\n");
//    fprintf(F, "%d %d\n", size_x, size_y);
//    fprintf(F, "255\n");

//    float maxval = 0;
//    for(int y = 0; y < size_y; ++y)
//    {
//        for(int x = 0; x < size_x; ++x)
//        {
//            maxval = maxval > image_(x,y) ? maxval : image_(x,y);
//        }
//    }

//    float scale = 255/sqrt(maxval);

//    for(int y = 0; y < size_y; ++y)
//    {
//        for(int x = 0; x < size_x; ++x)
//        {
//            unsigned char c = scale*sqrt(image_(x,y));
//            fputc(c, F);
//        }
//    }
////    cerr << "MAXVAL:" << maxval << endl;
//    fclose(F);
//}
