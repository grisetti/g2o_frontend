#include "roboteye_node.h"

namespace roboteye {

    using namespace ocular;
    using namespace std;
    using namespace Eigen;


    roboteye_node::roboteye_node(double az_rate, double n_lines, double laser_freq, double averaging, bool intensity, std::string outfilename) : _sensor_IP("169.254.111.100")
    {
        _az_rate = az_rate;
        _N_lines = n_lines;
        _laser_freq = laser_freq;
        _averaging = averaging;
        _intensity = intensity;
        _outfilename = outfilename;

        _num_readings = 0;
        _lastStamp = ros::Time::now();

        // connecting to RobotEye
        try {
            std::cout << "creating RE05Driver" << std::endl;
            laserone = new ocular::RE05Driver(_sensor_IP.c_str());
            cout << "[....] IP address of the RobotEye Laser scanner: " << _sensor_IP << endl;

        }
        catch(std::exception  &e) {
            std::cout << "Something went wrong, caught exception: " << e.what() << std::endl;
            return;
        }

        _scan_config.min_azimuth = deg2rad(0);
        _scan_config.max_azimuth = deg2rad(360);
        _scan_config.azim_increment = deg2rad(0.01); //to be checked
        _scan_config.min_elevation = deg2rad(-35);
        _scan_config.max_elevation = deg2rad(35);
        _scan_config.elev_increment = deg2rad(0.004); //to be checked
        _scan_config.min_range = 0.5; //see the documentation
        _scan_config.max_range = 250.0; //see the documentation

        _scan_pub = _handle.advertise<robot_eye_driver::RobotEyeScan>("roboteye_scan", 1024);

        _thrd = boost::thread(&roboteye_node::roboteyeRunning, this);
    }

    roboteye_node::~roboteye_node() {

        this->stopAndPrint();
//        _laser_callback.getMutex().lock();
//        _laser_callback.getMutex().~Mutex();
//        _thrd.interrupt();
//        _thrd.~thread();
    }

    void roboteye_node::setConfig(roboteye::RobotEyeConfig& scan_config) {
        _scan_config.time_increment = scan_config.time_increment;
        _scan_config.scan_time = scan_config.scan_time;
    }

    void roboteye_node::setScan(roboteye::RobotEyeScan& scan) {
        _scan.intensities = scan.intensities;
        _scan.ranges = scan.ranges;
    }



    void roboteye_node::roboteyeRunning(){

        /// homing the RobotEye, no need to call a sleep(), because Home() is blocking
        ocular::ocular_error_t err0 = laserone->Home();

        if(err0 != 0) {
            cout << "home error err: " << laserone->GetErrorString(err0) << endl;
            exit(1);
        }
        double azimuth, elevation;
        err0 = laserone->GetApertureAngles(azimuth, elevation);
        cout << "the initial home position is:\tazimuth = " << azimuth << ",\televation = " << elevation << ", err: " << laserone->GetErrorString(err0) << endl;

        /// start a Full Field Scan
        err0 = laserone->StartFullFieldScan(_az_rate, _N_lines); // e.g. 3600 degrees/sec (10 revolution/sec) with NLines_min = 3600/360
        if(err0 != 0) {
            cout << "Start FullField scan err: " << laserone->GetErrorString(err0) << endl;
            exit(1);
        }
        /// start the Laser
        err0 = laserone->StartLaser(_laser_freq, _averaging, _intensity, &_laser_callback);
        cout << "...starting streaming laser measurements" << endl;
        if(err0 != 0) {
            cout << "Start laser err: " << laserone->GetErrorString(err0) << endl;
            exit(1);
        }
    //    ocular::ocular_error_t err2 = laserone->StreamApertureAngles(5, &_angle_callback);
    //    cout << "...streaming aperture position" << ", err: " << laserone->GetErrorString(err2) << std::endl;

    //    cout << "going to sleep for " << _seconds << " seconds." << endl;
    //    int left_s = _seconds;
    //    left_s = sleep(_seconds);

    }


    void roboteye_node::stopAndPrint() {

        /// stopping laser callback
        cout << endl << "stopping laser" << endl;
        int attempts = 0;
        ocular::ocular_error_t err_ = laserone->StopLaser();
        while(err_ != 0 && attempts < 10){
            err_ = laserone->StopLaser();
            attempts++;
        }
        if(err_ != 0){
            cout << "got an error(StopLaser): " << laserone->GetErrorString(err_) << endl;
            exit(1);
        }

        /// stopping laser movements
        cout << "stopping laser motors" << endl;
        attempts = 0;
        err_ = laserone->Stop();
        while(err_ != 0 && attempts < 10){
            err_ = laserone->Stop();
            attempts++;
        }
        if(err_ != 0){
            cout << "got an error(Stop): " << laserone->GetErrorString(err_) << endl;
            exit(1);
        }

        /// stopping aperture angles callback
        //  cout << "stopping aperture angles streaming" << endl;
        //  attempts = 0;
        //  err_ = laserone->StopStreamingApertureAngles();
        //  while(err_ != 0 && attempts < 10){
        //    err_ = laserone->StopStreamingApertureAngles();
        //    attempts++;
        //  }
        //  if(err_ != 0){
        //    cout << "got an error: " << laserone->GetErrorString(err_) << endl;
        //    exit(1);
        //  }

        /// setting a desired final position
        cout << "setting a desired final position" << endl;
        attempts = 0;
        //  err_ = laserone->SetApertureAngles(0.0f, -35.0f, 1, &_notif_callback); // look down
        err_ = laserone->Home(); //home position should be az=0, el=0;
        while(err_ != 0 && attempts < 10){
            //    err_ = laserone->SetApertureAngles(0.0f, -35.0f, 1, &_notif_callback);
            err_ = laserone->Home();
            attempts++;
        }
        if(err_ != 0){
            std::cout << "got an error(setting home): " << laserone->GetErrorString(err_) << std::endl;
            exit(1);
        }

        /// printing the laser data
        this->printAndWriteLaserData(_outfilename);
    }

    void roboteye_node::printAndWriteLaserData(string outfilename) {

        /// debug
//        PolarList& meas_list = _laser_callback.measList();
//        for (PolarList::iterator it = meas_list.begin(); it != meas_list.end(); ++it) {
//            PolarMeasurements meas_all_callback = *it;
//            for(unsigned int j = 0; j < meas_all_callback.size(); j++) {
//                ocular::ocular_rbe_obs_t meas = meas_all_callback[j];
//                cerr << "azim: " << meas.azimuth << ",\telev: " << meas.elevation << ",\trange: " << meas.range << ",\tintensity: " << meas.intensity << endl;
//            }
//        }
        cout << "writing Data for octave plotting and for PCD_viewer" << endl;
        ofstream plotting("plottami.txt");
        pcl::PointCloud<pcl::PointXYZI> cloud;

        // !!! all the observations have the same size
        EuclideanList& xyz_list = _laser_callback.xyzMeasList();
        unsigned int obs_dim = xyz_list.front().size();

        /// total number of points: number of observations * size of one observation
        cloud.width = xyz_list.size() * obs_dim;
        cloud.height = 1;
    //    cout  << ">>>>>>>>>>>>>> "<< cloud.width << " " << _laser_callback._xyz_meas_all.size() << endl;

        for (EuclideanList::iterator it = xyz_list.begin(); it != xyz_list.end(); ++it) {
            EuclideanMeasurements _xyz_all_callback = *it;

            for(unsigned int j = 0; j < _xyz_all_callback.size(); j++) {
                Eigen::Vector4f xyz_p = _xyz_all_callback[j];

                /// create file for octave plotting
                plotting << xyz_p[0] << " " << xyz_p[1] << " " << xyz_p[2] << " " << xyz_p[3];
                plotting << endl;

                /// creat a PCD file format
                pcl::PointXYZI pcl_p;
                pcl_p.x = xyz_p[0];
                pcl_p.y = xyz_p[1];
                pcl_p.z = xyz_p[2];
                pcl_p.intensity = xyz_p[3];
                cloud.points.push_back(pcl_p);
            }

        }

        cout << "Saving " << cloud.points.size() << " data points to " << outfilename.c_str() << endl;
        pcl::io::savePCDFileASCII(outfilename.c_str(), cloud);

        plotting.flush();
        plotting.close();
    }

}
