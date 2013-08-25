#ifndef _PWN_VOXEL_CALCULATOR_H_
#define _PWN_VOXEL_CALCULATOR_H_
#include "g2o_frontend/boss_logger/eigen_boss_plugin.h" 
#include "g2o_frontend/boss/object_data.h"
#include "g2o_frontend/boss/identifiable.h"
#include "frame.h"

#include <map>

using namespace std;
using namespace Eigen;

namespace pwn {

  class VoxelCalculator : public boss::Identifiable {

    struct VoxelAccumulator {
      Point accumulator;
      int numPoints;
      // HAKKE
      int index;
      
	
      VoxelAccumulator() {
	accumulator[0] = accumulator[1] = accumulator[2] = 0.0f;
	numPoints = 0;
	// HAKKE
	index = -1;
      }
	
      void add(const Point &/*v*/) {
	// HAKKE
	/* numPoints++; */
	/* accumulator += v; */
      }
	
      Point average() const {
	float f = 1.0f / numPoints;
	return accumulator * f; 
      }
    };
      
    struct IndexComparator {
      int indeces[3];
      bool operator < (const IndexComparator &s) const {
	if(indeces[0] < s.indeces[0]) return true;
	if(indeces[0] == s.indeces[0] && indeces[1] < s.indeces[1]) return true;
	if(indeces[1] == s.indeces[1] && indeces[2] < s.indeces[2]) return true;
	return false;
      }
    };
      
    typedef map<IndexComparator, VoxelAccumulator> AccumulatorMap;

    float _resolution;
  public: 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
    // Constructor and destructor
    VoxelCalculator(int id=-1, boss::IdContext* context=0);
    virtual ~VoxelCalculator();
    inline float resolution() const {return _resolution;}
    inline void setResolution(float resolution_) {_resolution = resolution_;}
    
    void compute(Frame &frame, float resolution);
    void compute(Frame &frame);

    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);

  };

}

#endif
