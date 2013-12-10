#include "voxelcalculator.h"

using namespace std;
using namespace Eigen;

namespace pwn {

  void VoxelCalculator::compute(Frame &frame, float res) {
    float oldRes = resolution();
    setResolution(res);
    compute(frame);
    setResolution(oldRes);
  }

  void VoxelCalculator::compute(Frame &frame) {
    AccumulatorMap accumulatorMap;
    float inverseResolution = 1.0f / _resolution;

    for(size_t i = 0; i < frame.points().size(); i++) {
      const Point &point = frame.points()[i];

      IndexComparator s;
      s.indeces[0] = (int) (point[0] * inverseResolution);
      s.indeces[1] = (int) (point[1] * inverseResolution);
      s.indeces[2] = (int) (point[2] * inverseResolution);

      AccumulatorMap::iterator it = accumulatorMap.find(s);
      if(it == accumulatorMap.end()) {
	VoxelAccumulator voxelAccumulator;
	voxelAccumulator.accumulator = point;
	voxelAccumulator.numPoints = 1;
	voxelAccumulator.index = i;

	accumulatorMap.insert(make_pair(s, voxelAccumulator));
      }
      else {
	VoxelAccumulator &voxelAccumulator = it->second;
	voxelAccumulator.add(point);
      }
    }

    std::cout << "Voxelization resized the cloud from " << frame.points().size() << " to ";
    // HAKKE
    // frame.clear();
    Frame tmpFrame;
    tmpFrame.clear();
    for(AccumulatorMap::iterator it = accumulatorMap.begin(); it != accumulatorMap.end(); it++) {
      VoxelAccumulator &voxelAccumulator = it->second;
      // HAKKE
      // Point average = voxelAccumulator.average();
      // frame.points().push_back(average);
      tmpFrame.points().push_back(frame.points()[voxelAccumulator.index]);
      tmpFrame.normals().push_back(frame.normals()[voxelAccumulator.index]);
      tmpFrame.stats().push_back(frame.stats()[voxelAccumulator.index]);
      if(frame.pointInformationMatrix().size() == frame.points().size() &&
	 frame.normalInformationMatrix().size() == frame.points().size()) {
	tmpFrame.pointInformationMatrix().push_back(frame.pointInformationMatrix()[voxelAccumulator.index]);
	tmpFrame.normalInformationMatrix().push_back(frame.normalInformationMatrix()[voxelAccumulator.index]);
      }
      if(frame.traversabilityVector().size() == frame.points().size()) {
	tmpFrame.traversabilityVector().push_back(frame.traversabilityVector()[voxelAccumulator.index]);
      }
      if(frame.gaussians().size() == frame.points().size()) {
	tmpFrame.gaussians().push_back(frame.gaussians()[voxelAccumulator.index]);
      }
    }

    // HAKKE
    frame.clear();
    frame = tmpFrame;

    std::cout << frame.points().size() << " points" << std::endl;
  }
}
