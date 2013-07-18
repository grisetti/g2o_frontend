#include "voxelcalculator.h"

using namespace std;
using namespace Eigen;

namespace pwn {

  void VoxelCalculator::compute(Frame &frame, float resolution) {
    std::cerr << "COMPUTING VOXEL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl; 
      
    AccumulatorMap accumulatorMap;
    float inverseResolution = 1.0f / resolution;

    std::cerr << "Size: " << frame.points().size() << std::endl;
    for(size_t i = 0; i < frame.points().size(); i++) {
      const Point &point = frame.points()[i];

      IndexComparator s;
      //std::cerr << "Inverse resolution: " << inverseResolution << std::endl;
      //std::cerr << "Point: " << point[0] << " --- " << point[1] << " --- " << point[2] << std::endl;
      s.indeces[0] = (int) (point[0] * inverseResolution);
      s.indeces[1] = (int) (point[1] * inverseResolution);
      s.indeces[2] = (int) (point[2] * inverseResolution);

      //std::cerr << "Indeces: " << s.indeces[0] << " --- " << s.indeces[0] << " --- " << s.indeces[0] << std::endl;

      AccumulatorMap::iterator it = accumulatorMap.find(s);
      if(it == accumulatorMap.end()) {
	VoxelAccumulator voxelAccumulator;
	voxelAccumulator.accumulator[0] = point[0];
	voxelAccumulator.accumulator[1] = point[1];
	voxelAccumulator.accumulator[2] = point[2];
	voxelAccumulator.numPoints = 1;
	accumulatorMap.insert(make_pair(s, voxelAccumulator));
      }
      else {
	VoxelAccumulator &voxelAccumulator = it->second;
	voxelAccumulator.add(point);
      }
    }

    cerr << "Voxelization resized the cloud from " << frame.points().size() << " to ";
    frame.clear();
    for(AccumulatorMap::iterator it = accumulatorMap.begin(); it != accumulatorMap.end(); it++) {
      VoxelAccumulator &voxelAccumulator = it->second;
      Point average = voxelAccumulator.average();
      frame.points().push_back(average);
    }
    cerr << frame.points().size() << " points" << endl;
  }

}
