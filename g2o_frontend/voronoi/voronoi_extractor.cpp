#include <Eigen/Core>

#include <fstream>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <sys/time.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>


#define INF 1000000


using namespace std;
using namespace Eigen;



inline double get_time()
{
    struct timeval ts;
    gettimeofday(&ts,0);
    return ts.tv_sec + ts.tv_usec*1e-6;
}


struct VoronoiVertex
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VoronoiVertex()
    {
        _parent = Vector2i(INF, INF);
        _position = Vector2i(INF, INF);
        _value = 0;
        _distance = 0.0;
    }

    VoronoiVertex(const Vector2i& par_, const Vector2i& pos_, const float& dis_, const int& val_)
    {
        _parent = par_;
        _position = pos_;
        _distance = dis_;
        _value = val_;
    }

    Vector2i _parent;
    Vector2i _position;
    float _distance;
    int _value;
};


struct VertexComparator
{
    inline bool operator() (const VoronoiVertex* lhs, const VoronoiVertex* rhs) const
    {
        if(lhs->_distance <= rhs->_distance)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
};


typedef Matrix<VoronoiVertex, Dynamic, Dynamic> DistanceMap;
typedef Matrix<Vector2f, Dynamic, Dynamic> LookUpTable;
typedef priority_queue<VoronoiVertex, vector<VoronoiVertex*>, VertexComparator> DistanceQueue;



DistanceMap* distanceMap = 0;
DistanceQueue* distQueue = 0;
LookUpTable* lut = 0;
MatrixXf* tit = 0;
//MatrixXf* voro = 0;
cv::Mat* voro = 0;



void fillLookUpTable(int dTh, float dRho, int maxDist, LookUpTable*& map)
{
    int cols_half = dTh;
    float angular_step = M_PI/dTh;
    int rows = maxDist/dRho+1;
    map = new LookUpTable(rows, cols_half*2+1);

    for(float j = -cols_half; j <= cols_half; ++j)
    {
        float th = angular_step*j;
        for(float i = 0; i < rows; ++i)
        {
            float rho = dRho*i;
            (*map)(i, j+cols_half) = Vector2f(cos(th)*rho, sin(th)*rho);
        }
    }
}


void testLookUpTable(int dTh, float dRho, int maxDist, LookUpTable*& lut)
{
    int cols_half = dTh;
    float angular_step = M_PI/dTh;
    int rows = maxDist/dRho+1;
    lut = new LookUpTable(rows, cols_half*2+1);

    for(float j = -cols_half; j <= cols_half; ++j)
    {
        float th = angular_step*j;
        for(float i = 0; i < rows; ++i)
        {
            float rho = dRho*i;
            (*lut)(i, j+cols_half) = Vector2f(rho, th);
        }
    }

    cout << "dumping s.l.u.t." << endl;
    for(int i = 0; i < rows; ++i)
    {
        for(int j = 0; j < cols_half*2+1; ++j)
        {
            cout << "(" << (*lut)(i, j).x() << ", " << (*lut)(i, j).y() << ")";
        }
        cout << endl;
    }
}


void titSeeker(MatrixXf*& tit, const LookUpTable& lut, const MatrixXf& voro, const Vector2i& mouse_coords)
{
    int tit_rows = lut.cols();
    int tit_cols = lut.rows();

    tit = new MatrixXf(MatrixXf::Zero(tit_rows, tit_cols));

    for(int j = 0; j < tit_rows; j++)
    {
        for(int i = 0; i < tit_cols; i++)
        {
            Vector2i ilut(lut(i, j).x(), lut(i, j).y());
            Vector2i tit_coords = ilut + mouse_coords;
            (*tit)(j, (tit_cols - 1) - i) = voro(tit_coords.x(), tit_coords.y());
        }
    }
}


void loadPGM(istream& is, DistanceMap*& distmap)
{
    string tag;
    is >> tag;
    if(tag != "P5")
    {
        cerr << "Awaiting 'P5' in pgm header, found " << tag << endl;
        exit(-1);
    }

    int size_x, size_y;
    while(is.peek() == ' ' || is.peek() == '\n')
    {
        is.ignore();
    }
    while(is.peek() == '#')
    {
        is.ignore(255, '\n');
    }
    is >> size_x;
    while(is.peek() == '#')
    {
        is.ignore(255, '\n');
    }
    is >> size_y;
    while(is.peek() == '#')
    {
        is.ignore(255, '\n');
    }
    is >> tag;
    if(tag != "255")
    {
        cerr << "Awaiting '255' in pgm header, found " << tag << endl;
        exit(-1);
    }

    distmap = new DistanceMap(size_x, size_y);

    for(int y = 0; y < size_y; ++y)
    {
        for(int x = 0; x < size_x; ++x)
        {
            float c = is.get();
            (*distmap)(x, y)._position = Vector2i(x, y);
            (*distmap)(x, y)._value = c;

            if(c == 0)
            {
                (*distmap)(x, y)._distance = 0;
                (*distmap)(x, y)._parent = Vector2i(x, y);
            }
            else
            {
                (*distmap)(x, y)._distance = INF;
                (*distmap)(x, y)._parent = Vector2i(INF, INF);
            }
        }
    }
}


void savePGM(const char *filename, MatrixXf& map)
{
    FILE* F = fopen(filename, "w");
    if(!F)
    {
        cerr << "could not open 'result.pgm' for writing!" << endl;
        return;
    }

    const int size_x = map.rows();
    const int size_y = map.cols();

    fprintf(F, "P5\n");
    fprintf(F, "#CREATOR: Voronoi Extractor\n");
    fprintf(F, "%d %d\n", size_x, size_y);
    fprintf(F, "255\n");

    float maxval = 0;
    for(int y = 0; y < size_y; ++y)
    {
        for(int x = 0; x < size_x; ++x)
        {
            maxval = maxval > map(x,y) ? maxval : map(x, y);
        }
    }

    float scale = 255/sqrt(maxval);

    for(int y = 0; y < size_y; ++y)
    {
        for(int x = 0; x < size_x; ++x)
        {
            unsigned char c = scale*sqrt(map(x,y));
            fputc(c, F);
        }
    }
    cerr << "MAXVAL:" << maxval << endl;
    fclose(F);
}


void queueFiller(DistanceMap& distmap, DistanceQueue* disqueue)
{
    const int rows = distmap.rows();
    const int cols = distmap.cols();

    const short int coords_x[4] = {1, 0, -1,  0};
    const short int coords_y[4] = {0, 1,  0, -1};

    for(int y = 0; y < cols; ++y)
    {
        for(int x = 0; x < rows; ++x)
        {
            VoronoiVertex* v = &distmap(x, y);

            const int v_x = v->_position.x();
            const int v_y = v->_position.y();

            bool counter = false;
            for(short int i = 0; (i < 4) && (counter == false); ++i)
            {
                int r = coords_x[i];
                int c = coords_y[i];

                int neighbor_x = v_x + r;
                int neighbor_y = v_y + c;

                //Out of boundaries
                if((neighbor_x >= 0) && (neighbor_y >= 0) && (neighbor_x < rows) && (neighbor_y < cols) && (distmap(neighbor_x, neighbor_y)._distance != 0))
                {
                    counter = true;
                    disqueue->push(v);
                }
            }
        }
    }
}


void distmap2image(DistanceMap& distmap, MatrixXf& image)
{
    const int rows = distmap.rows();
    const int cols = distmap.cols();

    for(size_t y = 0; y < cols; ++y)
    {
        for(size_t x = 0; x < rows; ++x)
        {
            image(x, y) = distmap(x, y)._distance;
        }
    }
}


void map2distmap(DistanceMap& distmap, DistanceQueue& disque)
{
    const int rows = distmap.rows();
    const int cols = distmap.cols();

    const short int coords_x[4] = {-1, 1,  0, 0};
    const short int coords_y[4] = { 0, 0, -1, 1};

    while(!disque.empty())
    {
        VoronoiVertex* current = disque.top();
        disque.pop();

        const int current_x = current->_position.x();
        const int current_y = current->_position.y();

        for(short int i = 0; i < 4; ++i)
        {
            const short int r = coords_x[i];
            const short int c = coords_y[i];

            int nc_x = current_x + r;
            int nc_y = current_y + c;

            //Check if into boundaries or not an obstacle
            if((nc_x >= 0) && (nc_y >= 0) && (nc_x < rows) && (nc_y < cols) && (distmap(nc_x, nc_y)._distance != 0))
            {
                VoronoiVertex* neighbor = &distmap(nc_x, nc_y);
                Vector2i nc(nc_x, nc_y);

                double neighbor_to_current_parent_dist = (nc - current->_parent).squaredNorm();
                if(neighbor_to_current_parent_dist < neighbor->_distance)
                {
                    neighbor->_distance = neighbor_to_current_parent_dist;
                    neighbor->_parent = current->_parent;
                    disque.push(neighbor);
                }
            }
        }
    }
}


void distmap2voronoi(DistanceMap& distmap, cv::Mat& voromap)
{
    const int rows = distmap.rows();
    const int cols = distmap.cols();

    const short int coords_x[] = { 0, 0, -1, 1, -1, -1,  1, 1, 0};
    const short int coords_y[] = {-1, 1,  0, 0, -1,  1, -1, 1, 0};

//    voromap.fill(255);            VALID IF VOROMAP IS A MATRIXXF
    for(int y = 0; y < cols; ++y)
    {
        for(int x = 0; x < rows; ++x)
        {
            VoronoiVertex* current = &distmap(x, y);

            for(short int i = 0; i < 4; ++i)
            {
                const short int r = coords_x[i];
                const short int c = coords_y[i];

                const int neighbor_x = x + r;
                const int neighbor_y = y + c;

                if((neighbor_x >= 0) && (neighbor_y >= 0) && (neighbor_x < rows) && (neighbor_y < cols))
                {
                    VoronoiVertex* neighbor = &distmap(neighbor_x, neighbor_y);

//                    float pdist = (current->_parent-neighbor->_parent).squaredNorm();
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

                    /*PREVIOUS WORKING VERSION*/
                    float pdist = (current->_parent-neighbor->_parent).squaredNorm();
                    if(pdist > 100)
                    {
                        voromap.at<uchar>(x, y) = 255;
                        cv::line(voromap, cv::Point(y, x), cv::Point(current->_parent.y(), current->_parent.x()), cv::Scalar(127));
                        for(int p = 0; p < 9; ++p)
                        {
                            int drawable_x = current->_parent.x()+coords_x[p];
                            int drawable_y = current->_parent.y()+coords_y[p];

                            if((drawable_x >= 0) && (drawable_y >= 0) && (drawable_x < rows) && (drawable_y < cols))
                            {
                                voromap.at<uchar>(drawable_x, drawable_y) = 50;
                            }
                        }
                    }
                }
            }
        }
    }
}


void cvmat2eigenmat(MatrixXf& out, const cv::Mat& in)
{
    for(int i = 0; i < in.rows; ++i)
    {
        for(int j = 0; j < in.cols; ++j)
        {
            out(i, j) = in.at<uchar>(i, j);
        }
    }
}


void eigenmat2cvmat(cv::Mat& out, const MatrixXf& in)
{
    for(int i = 0; i < in.rows(); ++i)
    {
        for(int j = 0; j < in.cols(); ++j)
        {
            out.at<uchar>(i, j) = in(i, j);
        }
    }
}



void mouseEvent(int evt, int x, int y, int flags, void* param)
{
    if(evt == CV_EVENT_LBUTTONDOWN)
    {
//        titSeeker(tit, *lut, *voro, Vector2i(x, y));
//        savePGM("tit.pgm", *tit);
//        cv::Mat im2 = cv::imread("tit.pgm", CV_LOAD_IMAGE_UNCHANGED);
//        cv::imshow("tit", im2);
//        im2.release();
    }
}


int main(int argc, char** argv)
{
    if(argc < 2 || argc > 3)
    {
        cerr << "usage: " << argv[0] <<" <pgm map> " << endl;
        exit(-1);
    }

    ifstream is(argv[1]);
    if(!is)
    {
        cerr << "Could not open map file for reading." << endl;
        exit(-1);
    }

    distanceMap = new DistanceMap;
    distQueue = new DistanceQueue;

    double pre = get_time();
    loadPGM(is, distanceMap);
    double im = get_time();
    queueFiller(*distanceMap, distQueue);
    double post = get_time();
    cerr << "load time: " << (im - pre)*1000 << endl;
    cerr << "queue filling time: " << (post - im)*1000 << endl;

    double ini = get_time();
    map2distmap(*distanceMap, *distQueue);
    double end = get_time();
    cerr << "total time: " << (end - ini)*1000 << endl;

    MatrixXf seconda = MatrixXf::Zero(distanceMap->rows(), distanceMap->cols());
    distmap2image(*distanceMap, seconda);
    savePGM("distmap.pgm", seconda);

//    voro = new MatrixXf(MatrixXf::Zero(distanceMap->rows(), distanceMap->cols()));
    voro = new cv::Mat(distanceMap->rows(), distanceMap->cols(), CV_8UC1, cv::Scalar(0));
    distmap2voronoi(*distanceMap, *voro);

    MatrixXf voro2 = MatrixXf(MatrixXf::Zero(distanceMap->rows(), distanceMap->cols()));
    cvmat2eigenmat(voro2, *voro);
    savePGM("voro.pgm", voro2);

    lut = new LookUpTable;
    int dt = 256;
    float dr = 0.125;
    int max = 10;
    fillLookUpTable(dt, dr, max, lut);

    cv::namedWindow("MyWindow", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
    cv::setMouseCallback("MyWindow", mouseEvent, 0);
    cv::namedWindow("tit", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
    //load and display an image
    cv::Mat img = cv::imread("voro.pgm", CV_LOAD_IMAGE_UNCHANGED);
    cv::imshow("MyWindow", img);

    //wait for key press
    cv::waitKey(0);

    //cleaning up
    cv::destroyWindow("MyWindow");
    cv::destroyWindow("tit");
    img.release();

    exit(0);
}
