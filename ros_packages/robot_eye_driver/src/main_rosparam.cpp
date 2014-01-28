#include "roboteye_node.h"

using namespace std;
using namespace roboteye;

//global parameters
std::string outfilename;

ros::Time getRosTime(roboteye_node& re) {
    re.setLastStamp(ros::Time::now());
    return re.lastStamp();
}

void publishScan(roboteye_node& re){

    // creating the ros_msg to be published
//    Mutex& m = re.laserCallBack().getMutex();
//    m->lock();

    robot_eye_driver::RobotEyeScan scan_msg;

    PolarMeasurements pm_current;
    bool pm_exist = re.laserCallBack().pop(pm_current);
    if (!pm_exist) {
        cerr << "!";
    }
//    m->unlock();

    unsigned int nr = pm_current.size();
    re.setNumReadings(nr);
    roboteye::RobotEyeConfig scan_config;
    scan_config.time_increment = nr/re.laserFreq();
    // scan_config is to be used for interpolating the scan in case of the robot is moving
    scan_config.scan_time = 1./re.laserFreq();

    re.setConfig(scan_config);

    roboteye::RobotEyeScan scan_current;
    for(unsigned int i = 0; i < nr; ++i){
        scan_current.intensities.push_back(pm_current[i].intensity);
        Eigen::Vector3d range = Eigen::Vector3d(pm_current[i].azimuth, pm_current[i].elevation, pm_current[i].range);
        scan_current.ranges.push_back(range);
    }
    re.setScan(scan_current);

    ros::Time scan_time = getRosTime(re);
    //add freqFilter

    // populate the RobotEyeScan message
    scan_msg.header.stamp = scan_time;
    scan_msg.header.frame_id = "roboteye_frame";
    scan_msg.azimuth_min = re.config().min_azimuth;
    scan_msg.azimuth_max = re.config().max_azimuth;
    scan_msg.azimuth_increment = re.config().azim_increment;
    scan_msg.elevation_min = re.config().min_elevation;
    scan_msg.elevation_max = re.config().max_elevation;
    scan_msg.elevation_increment = re.config().elev_increment;
    scan_msg.time_increment = re.config().time_increment;
    scan_msg.scan_time = re.config().scan_time;
    scan_msg.range_min = re.config().min_range;
    scan_msg.range_max = re.config().max_range;

    scan_msg.measurements.resize(nr);
    scan_msg.intensities.resize(nr);

    for(unsigned int i = 0; i < re.scan().ranges.size(); ++i){
        geometry_msgs::Vector3 v_ranges;
        v_ranges.x = re.scan().ranges[i].x();
        v_ranges.y = re.scan().ranges[i].y();
        v_ranges.z = re.scan().ranges[i].z();

        scan_msg.measurements[i] = v_ranges;
        scan_msg.intensities[i] = re.scan().intensities[i];
    }
    re.scanPub().publish(scan_msg);

    cout << "p ";
}

int main(int argc, char **argv)
{
    cout << "=== ROBOTEYE node ===" << endl << endl;

    ros::init(argc, argv, "roboteye_node");
    ros::Time::init();
    roboteye_node re;
    ros::Rate r(20);

    dynamic_reconfigure::Server<robot_eye_driver::RobotEyeParametersConfig> server;
    dynamic_reconfigure::Server<robot_eye_driver::RobotEyeParametersConfig>::CallbackType f;
    f = boost::bind(&roboteye_node::dynamic_reconf_callback, &re, _1, _2);
    server.setCallback(f);

    //TODO COMMAND SUBSCRIBED FROM EXTERNAL: STATE -> RUNNING?
    //the _state variableof re is modified by a subscriber that check if an external command switches from run to pause or stop
//    re.setState(RUN);
//    usleep(2e3);

    while(ros::ok()) {
        ros::spinOnce();
        bool run = re.isRunning();
        if (run)
            publishScan(re);
        else
            ROS_WARN_ONCE_NAMED("eval", "Waiting for RobotEye Laser-Data Acquisition");

        r.sleep();
    }

    cout << endl;
    cout << "=== END ===" << endl;
    return 0;
}
