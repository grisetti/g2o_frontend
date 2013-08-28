#include "voronoi_aligner.h"


VoronoiAligner::VoronoiAligner()
{
    _ref = 0;
    _curr = 0;
}


VoronoiAligner::VoronoiAligner(VoronoiDiagram* ref_, VoronoiDiagram* curr_)
{
    _ref = ref_;
    _curr = curr_;
}


VoronoiAligner::~VoronoiAligner() {;}


void VoronoiAligner::align(const Eigen::Vector2i& refPoint, const Eigen::Vector2i& currPoint)
{

}
