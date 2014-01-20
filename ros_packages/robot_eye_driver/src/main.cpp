#include "roboteye_node.h"

using namespace std;
using namespace roboteye;

//global parameters
//    int seconds;
double az_rate;
double n_lines;
double laser_freq;
double averaging;
bool intensity;

std::string outfilename;

void init_parameters(int argc, char ** argv){

    // initialize parameters
    g2o::CommandArgs arg;
//    arg.param("t",seconds, 15, "choose for how many seconds you want to exec this routine");
    arg.param("az_rate", az_rate, 10, "set the number of rounds per second [max:15(5400 degrees/sec)]");
    arg.param("nlines", n_lines, 100, "set the number of horizontal lines (vertical resolution) [min:_az_rate]");
    arg.param("avg", averaging, 1, "set the 'averaging' value range:[1,5]");
    arg.param("lfreq", laser_freq, 10000, "set the measurement frequency [range:[1, 30000]]");
    arg.param("intensity", intensity, 0, "enable the streaming of intensity values [can be 'on' only if lfreq <= 10000] default off");
    arg.param("o", outfilename, "Eyebot_pcd.pcd", "output filename of a pcd format file");

    arg.parseArgs(argc, argv);

    // check parameters consistency
    if(averaging > 5){
        cout << "-avg must be an integer value from 1 to 5" << endl << endl;
        exit(1);
    }

    if(laser_freq > 30000 || laser_freq == 0){
        cout << "-lfreq must be an integer value in [1, 30000]" << endl << endl;
        exit(1);
    }

    if(az_rate > 15 || az_rate == 0){
        cout << "-az_rate must be a value in (0, 15]" << endl << endl;
        exit(1);
    }

    if(n_lines < az_rate){
        cout << "-nlines must be at least equal to az_rate" << endl << endl;
        exit(1);
    }

    if(intensity && laser_freq > 10000){
        cout << "-intensity can't be specified for lfreq values higher than 10000" << endl << endl;
        exit(1);
    }

//    if(seconds < 1){
//        cout << "-t must be an integer positive non zero value" << endl << endl;
//        exit(1);
//    }

    // print parameters
    cout << "LASER FREQUENCY:\t" << laser_freq << endl;
    cout << "AVERAGING:\t" << averaging << endl;
    cout << "GET INTENSITY:\t" << (intensity? "YES" : "NO") << endl;
    cout << "AZIMUTH RATE:\t" << az_rate << endl;
    cout << "N_LINES:\t" << n_lines << endl;
//    cout << "RUNNING FOR:\t" << seconds << " SECONDS" << endl;

}


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
    scan_config.time_increment = nr/re._laser_freq;
    scan_config.scan_time = 1/re._laser_freq;

    re.setConfig(scan_config);

    roboteye::RobotEyeScan scan_current;
    for(int i = 0; i < nr; i++){
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
    scan_msg.azimuth_min = scan_config.min_azimuth;
    scan_msg.azimuth_max = scan_config.max_azimuth;
    scan_msg.azimuth_increment = scan_config.azim_increment;
    scan_msg.elevation_min = scan_config.min_elevation;
    scan_msg.elevation_max = scan_config.max_elevation;
    scan_msg.elevation_increment = scan_config.elev_increment;
    scan_msg.time_increment = scan_config.time_increment;
    scan_msg.range_min = scan_config.min_range;
    scan_msg.range_max = scan_config.max_range;

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
    cout << "=== LASERONE node ===" << endl << endl;

    init_parameters(argc,argv);

    ros::init(argc, argv, "roboteye_node");
    ros::Time::init();
    roboteye_node re(az_rate, n_lines, laser_freq, averaging, intensity, outfilename);
    ros::Rate r(10);

    while(ros::ok()) {
        ros::spinOnce();
        cerr << "ok";
        publishScan(re);
        cerr << "ok";

        r.sleep();
    }

    cout << endl;
    cout << "=== END ===" << endl;
    return 0;
}
