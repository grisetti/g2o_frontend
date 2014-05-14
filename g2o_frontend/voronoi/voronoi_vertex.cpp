#include "voronoi_vertex.h"

#define INF 1000000

using namespace std;
using namespace Eigen;


VoronoiVertex::VoronoiVertex()
{
    _merged = false;
    _pushed = false;
    _visited = false;

    _nearest = Eigen::Vector2i(INF, INF);
    _parent = Eigen::Vector2i(INF, INF);
    _position = Eigen::Vector2i(INF, INF);
    _graphPose = Eigen::Vector3d(INF, INF, 0);

    _id = -1;
    _distance = 0.0;
    _value = 0;
}


VoronoiVertex::VoronoiVertex(const VoronoiVertex* v)
{
    this->_merged = false;
    this->_pushed = false;
    this->_visited = false;

    this->_nearest = v->nearest();
    this->_parent = v->parent();
    this->_position = v->position();
    this->_graphPose = v->graphPose();

    this->_id = v->id();
    this->_distance = v->distance();
    this->_value = v->value();
}


VoronoiVertex::~VoronoiVertex() {;}


bool VoronoiVertex::write(ostream& os)
{
    os << _id << " " << _graphPose.x() << " " << _graphPose.y() << " " << _graphPose.z();
    return os.good();
}


VoronoiLaser::VoronoiLaser() : VoronoiData()
{
    _tag = "ROBOTLASER1";

    _type = 0;
//    _firstBeamAngle = -M_PI;
//    _fov = 2*M_PI;
    _firstBeamAngle = -M_PI * 0.5;
    _fov = M_PI;
    _angularStep = 0.03066406; // It was 0.00872222;
    _maxRange = 200.;
    _accuracy = 0.1;
    _remissionMode = 0;
}


bool VoronoiLaser::write(ostream& os)
{
    os << _type << " " << _firstBeamAngle << " " << _fov << " " << _angularStep
       << " " << _maxRange << " " << _accuracy << " " << _remissionMode << " ";
    os << ranges().size();
    for(size_t i = 0; i < ranges().size(); ++i)
    {
        os << " " << ranges()[i];
    }
    os << " " << 0; // remissions

    // odometry pose
    Rotation2Dd r(0);
    r.fromRotationMatrix(_sensorPose.linear());
    Vector2d t = _sensorPose.translation();
    Vector3d p = Vector3d(t.x(), t.y(), r.angle());

    os << " " << p.x() << " " << p.y() << " " << p.z();
    os << " " << p.x() << " " << p.y() << " " << p.z();

    // Useless values
    time_t ts = time(NULL);
    os << " " <<  0. << " " <<  0. << " " << 0. << " " << 0. << " " << 0.;
    os << " " << ts << " hostname " << ts;

    return os.good();
}
