#include "manifold_voronoi_extractor.h"


using namespace std;
using namespace Eigen;


#define INF 1000000


namespace manifold_voronoi
{

ManifoldVoronoiData::ManifoldVoronoiData(int id, IdContext* context): ImageData(id, context)
{
    resolution = 0.2;
    node = 0;
}


void ManifoldVoronoiData::serialize(ObjectData& data, IdContext& context)
{
    ImageData::serialize(data, context);
    data.setFloat("resolution", resolution);
    data.setPointer("node", node);
}


void ManifoldVoronoiData::deserialize(ObjectData& data, IdContext& context)
{
    ImageData::deserialize(data, context);
    resolution = data.getFloat("resolution");
    data.getReference("node").bind(node);
}


ManifoldVoronoiExtractor::ManifoldVoronoiExtractor(int id, boss::IdContext* context): StreamProcessor(id,context)
{
    _manager = 0;
    _cache = 0;
    _resolution = 0.2;
    _xSize = 100;
    _ySize = 100;
    _normalThreshold = 0.64;
    _dequeSize = 30;
}


/** This function can be optimized*/
void ManifoldVoronoiExtractor::process(Serializable* s)
{
    put(s);
    NewKeyNodeMessage* km = dynamic_cast<NewKeyNodeMessage*>(s);
    int cx = _xSize * 0.5;
    int cy = _ySize * 0.5;
    float ires = 1./_resolution;
    if(km)
    {
        PwnCloudCache::HandleType h = _cache->get((SyncSensorDataNode*) km->keyNode);
        cacheHandles.push_back(h);
        while(cacheHandles.size() > _dequeSize)
        {
            cacheHandles.pop_front();
        }
        MapNode* n = km->keyNode;
        Isometry3d inT = n->transform().inverse();

        ManifoldVoronoiData* vdata = new ManifoldVoronoiData();
        vdata->resolution = _resolution;
        boss_map::ImageBLOB* imageBlob = new boss_map::ImageBLOB();
        imageBlob->cvImage().create(_xSize,_ySize, CV_16UC1);
        imageBlob->cvImage().setTo(30000);
        imageBlob->adjustFormat();
        vdata->setTimestamp(0);
        uint16_t obstacle = 65535;
        for(list<PwnCloudCache::HandleType>::iterator it = cacheHandles.begin(); it != cacheHandles.end(); it++)
        {
            PwnCloudCache::HandleType& h = *it;
            CloudWithImageSize* cloud_ = h.get();
            MapNode* cn = h.key();
            Isometry3d currentTransform=inT * cn->transform();
            pwn::Cloud cloud = *cloud_;
            Isometry3f  currentTransformf;
            convertScalar(currentTransformf, currentTransform);
            cloud.transformInPlace( currentTransformf);

            for(size_t i = 0; i < cloud.points().size(); i++)
            {
                pwn::Normal& n = cloud.normals()[i];
                pwn::Point& p = cloud.points()[i];
                int x = cx + p.x()*ires;
                int y = cy + p.y()*ires;
                if ( (x<0) ||
                     (x>=_xSize) ||
                     (y < 0) ||
                     (y>=_ySize) ){
                    continue;
                }
                uint16_t& imZ = imageBlob->cvImage().at<uint16_t>(x,y);
                int pz =  10000 - 1000 * p.z();
                if(imZ == obstacle)
                {
                    continue;
                }
                if(imZ < pz)
                {
                    continue;
                }
                if(n.squaredNorm() < 0.1)
                {
                    continue;
                }
                if(n.z() < _normalThreshold)
                {
                    imZ = obstacle;
                }
                else
                {
                    imZ = pz;
                }
            }
        }
        vdata->imageBlob().set(imageBlob);
        vdata->node = n;
        put(vdata);
    }
}


void ManifoldVoronoiExtractor::serialize(ObjectData& data, IdContext& context)
{
    StreamProcessor::serialize(data,context);
    data.setPointer("manager", _manager);
    data.setPointer("cache", _cache);
    data.setFloat("resolution", _resolution);
    data.setInt("xSize", _xSize);
    data.setInt("ySize", _ySize);
    data.setFloat("normalThreshold", _normalThreshold);
    data.setInt("dequeSize", _dequeSize);
}


void ManifoldVoronoiExtractor::deserialize(ObjectData& data, IdContext& context)
{
    StreamProcessor::deserialize(data,context);
    data.getReference("manager").bind(_manager);
    data.getReference("cache").bind(_cache);
    _resolution = data.getFloat("resolution");
    _xSize = data.getInt("xSize");
    _ySize = data.getInt("ySize");
    _normalThreshold = data.getFloat("normalThreshold");
    _dequeSize = data.getInt("dequeSize");
}


ManifoldVoronoi::ManifoldVoronoi(const cv::Mat& img_, int squaredResolution_)
{
    _squaredResolution = squaredResolution_;
    _distmap = 0;
    _graph = 0;
    _vQueue = 0;
    _voro = 0;

    this->init(img_);
}


ManifoldVoronoi::~ManifoldVoronoi()
{
    delete _graph;
    delete _voro;
    delete _distmap;
    delete _vQueue;
}


void ManifoldVoronoi::diagram2graph()
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

                if((nx >= 0) && (ny >= 0) && (nx < rows) && (ny < cols) && ((*_voro).at<uint16_t>(nx, ny) != 0) && (!((*_distmap)(nx, ny).visited())))
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


void ManifoldVoronoi::graph()
{
    _graph = new cv::Mat(_distmap->rows(), _distmap->cols(), CV_16UC1, cv::Scalar(0));
    for(VertexMap::iterator it = _candidates.begin(); it != _candidates.end(); ++it)
    {
        VoronoiVertex* current = it->second;
        int cx = current->position().x();
        int cy = current->position().y();

        _graph->at<uint16_t>(cv::Point(cy, cx)) = 65535;
//        _graph->at<cv::Vec3b>(cv::Point(cy, cx)) = cv::Vec3b(255, 255, 255);
    }
}


void ManifoldVoronoi::distmap()
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


void ManifoldVoronoi::distmap2voronoi()
{
    _voro = new cv::Mat(_distmap->rows(), _distmap->cols(), CV_16UC1, cv::Scalar(0));

    const int rows = _distmap->rows();
    const int cols = _distmap->cols();

//    const short int coords_x[4] = {-1, 1,  0, 0};
//    const short int coords_y[4] = { 0, 0, -1, 1};

    const short int coords_x[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
    const short int coords_y[8] = {-1,  0,  1, -1, 1, -1, 0, 1};

    for(int c = 0; c < cols; ++c)
    {
        for(int r = 0; r < rows; ++r)
        {
            if((*_distmap)(r, c).distance() != 0)
            {
                VoronoiVertex* current = &(*_distmap)(r, c);
                for(short int i = 0; i < 8; ++i)
                {
                    const int neighbor_x = r + coords_x[i];
                    const int neighbor_y = c + coords_y[i];

                    if((neighbor_x >= 0) && (neighbor_y >= 0) && (neighbor_x < rows) && (neighbor_y < cols) && ((*_distmap)(neighbor_x, neighbor_y).distance() != 0))
                    {
                        VoronoiVertex* neighbor = &(*_distmap)(neighbor_x, neighbor_y);
                        float pdist = (current->parent() - neighbor->parent()).squaredNorm();
                        if(pdist > _squaredResolution)
                        {
                            _voro->at<uint16_t>(r, c) = 65535;
                            _vMap.insert(VoronoiPair(current->position(), current));
                        }
                    }
                }
            }
        }
    }
}


void ManifoldVoronoi::fillQueue()
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
            if(obstacle->value() != 65535)
            {
                continue;
            }
            const int obstacle_x = obstacle->position().x();
            const int obstacle_y = obstacle->position().y();

            bool counter = false;
            int neighbors = 0;
            for(short int i = 0; (i < 8) /*&& (counter == false)*/; ++i)
            {
                int neighbor_x = obstacle_x + coords_x[i];
                int neighbor_y = obstacle_y + coords_y[i];

                // If at least one of the neighbors is a free cell, put the current obstacle into the queue
                if((neighbor_x >= 0) && (neighbor_y >= 0) && (neighbor_x < rows) && (neighbor_y < cols) && ((*_distmap)(neighbor_x, neighbor_y).distance() != 0))
                {
                    counter = true;
                    neighbors++;
                }
            }
            if(counter && (neighbors < 6) && (neighbors > 0))
            {
                _vQueue->push(obstacle);
                obstacle->setPushed();
//                cout << "MAX neighbors: " << neighbors << endl;
            }
        }
    }
}


void ManifoldVoronoi::distmap2image()
{
    _drawableDistmap = MatrixXf::Zero(_distmap->rows(), _distmap->cols());
    const unsigned int rows = _distmap->rows();
    const unsigned int cols = _distmap->cols();

    for(unsigned int y = 0; y < cols; ++y)
    {
        for(unsigned int x = 0; x < rows; ++x)
        {
            _drawableDistmap(y, x) = (*_distmap)(x, y).distance();
        }
    }
}


void ManifoldVoronoi::init(const cv::Mat& img_)
{
    int xsize = img_.rows;
    int ysize = img_.cols;

    _distmap = new DistanceMap(xsize, ysize);
    for(int c = 0; c < ysize; c++)
    {
        for(int r = 0; r < xsize; r++)
        {
            uint16_t pixel = img_.at<uint16_t>(r, c);
            if((pixel == 30000) || (pixel == 65535))
            {
                (*_distmap)(r, c).setDistance(0);
                (*_distmap)(r, c).setParent(r, c);
            }
            else
            {
                (*_distmap)(r, c).setDistance(INF);
                (*_distmap)(r, c).setParent(INF, INF);
            }
            (*_distmap)(r, c).setPosition(r, c);
            (*_distmap)(r, c).setValue(pixel);
            (*_distmap)(r, c).setNearest(r, c);
        }
    }
}


void ManifoldVoronoi::savePGM(const char *filename, const MatrixXf& image_)
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


BOSS_REGISTER_CLASS(ManifoldVoronoiData)
BOSS_REGISTER_CLASS(ManifoldVoronoiExtractor)
}
