#ifndef VORONOI_ALIGNER_H
#define VORONOI_ALIGNER_H

#include "voronoi_diagram.h"

class VoronoiAligner
{
    VoronoiAligner();
    VoronoiAligner(VoronoiDiagram* ref_, VoronoiDiagram* curr_);
    ~VoronoiAligner();

    inline void setReferenceVoronoi(VoronoiDiagram* ref_) { _ref = ref_; }
    inline void setCurrentVoronoi(VoronoiDiagram* curr_) { _curr = curr_; }

    void align(const Eigen::Vector2i& refPoint, const Eigen::Vector2i& currPoint);

    VoronoiDiagram* _ref;
    VoronoiDiagram* _curr;
};


#endif // VORONOI_ALIGNER_H
