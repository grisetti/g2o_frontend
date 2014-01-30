#include "odometry_controller.h"

namespace pwn {
  bool OdometryController::fileInitialization(const char *groundtruthFilename_, const char *associationsFilename_,
					      const char *trajectoryFilename_, const char *benchmarkFilename_) {
    _groundtruthFilename = groundtruthFilename_;
    _associationsFilename = associationsFilename_;
    _trajectoryFilename = trajectoryFilename_; 
    _benchmarkFilename = benchmarkFilename_;

    // Create benchmark file
    cout << "Creating benchmark file \'" << benchmarkFilename_ <<"\'... ";
    _ofsBenchmark.open(benchmarkFilename_);
    if (!_ofsBenchmark) {
      cerr << "Impossible to create the file containing the values of the benchmark." << endl;
      return false;
    }
    cout << "done." << endl;
  
    // Create trajectory file
    cout << "Creating trajectory file \'" << trajectoryFilename_ <<"\'... ";
    _ofsTrajectory.open(trajectoryFilename_);
    if (!_ofsTrajectory) {
      cerr << "Impossible to create the file containing the trajectory computed." << endl;
      return false;
    }
    cout << "done." << endl;
  
    // Open associations file
    cout << "Opening associations file \'" << associationsFilename_ <<"\'... ";
    _ifsAssociations.open(associationsFilename_);
    if (!_ifsAssociations) {
      cerr << "Impossible to open the associations file." << endl;
      return false;
    }
    cout << "done." << endl;

    // Opening and parsing (if provided) ground truth file for the initial transformation
    cout << "Opening ground truth file \'" << groundtruthFilename_ <<"\'... ";
    _ifsGroundtruth.open(groundtruthFilename_);
    if (!_ifsGroundtruth) {
      cerr << "Impossible to open the ground truth file." << endl;
      return false;
    }
    cout << "done." << endl;

    return true;
  }

  void OdometryController::getGroundTruthPose(Eigen::Isometry3f &pose, const double poseTimestamp) {
    _ifsGroundtruth.clear();
    _ifsGroundtruth.seekg(0, ios::beg);
    char line[4096];
    double currentTimestamp;
    pose = Eigen::Isometry3f::Identity();
    double minDifference = std::numeric_limits<float>::max();
    Vector6f poseVector;
    float qw;
    while (_ifsGroundtruth && _ifsGroundtruth.getline(line, 4096)) {
      istringstream issGroundtruth(line);
      issGroundtruth >> currentTimestamp;
      if(fabs(currentTimestamp - poseTimestamp) < minDifference) {
	issGroundtruth >> poseVector[0] >> poseVector[1] >> poseVector[2]
		       >> poseVector[3] >> poseVector[4] >> poseVector[5] >> qw;
	Eigen::Quaternionf q = Eigen::Quaternionf(qw, poseVector[3], poseVector[4], poseVector[5]);
	pose.translation() = Eigen::Vector3f(poseVector[0], poseVector[1], poseVector[2]);
	pose.linear() = q.toRotationMatrix();
	minDifference = fabs(currentTimestamp - poseTimestamp);
      }
    }
    pose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  }
}
