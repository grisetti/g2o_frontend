#include "roboteye_node.h"

namespace roboteye {

    using namespace ocular;
    using namespace std;
    using namespace Eigen;

roboteye_node::roboteye_node() : _sensor_IP("169.254.111.100")
{
    _isrunning = false;
    _num_readings = 0;
    _lastStamp = ros::Time::now();

    ros::NodeHandle private_handle("~");
    private_handle.param("az_rate", _az_rate, 10.);
    ROS_INFO("RobotEyeNode: az_rate = %f", _az_rate);

    private_handle.param("n_lines", _N_lines, 100);
    ROS_INFO("RobotEyeNode: n_lines = %i", _N_lines);

    private_handle.param("averaging", _averaging, 1);
    ROS_INFO("RobotEyeNode: averaging = %i", _averaging);

    private_handle.param("laser_freq",_laser_freq, 10000);
    ROS_INFO("RobotEyeNode: laser_freq = %i", _laser_freq);

    private_handle.param("intensity", _intensity, false);
    ROS_INFO("RobotEyeNode: intensity = %i", _intensity);

    // MERDACCIA
    int nstate = -1;
    private_handle.param("node_state", nstate, 2);
    if(nstate==1)  _node_state = RUN;
    else if(nstate==0) _node_state = STOP;
    else if(nstate==2) _node_state = PAUSE;
    ROS_INFO("RobotEyeNode: state = %i", _node_state);

    //this is the only parameters not availabe for dynamic reconfiguration
    private_handle.param<std::string>("outfilename", _outfilename, "EyebotPcd.pcd");
    ROS_INFO("RobotEyeNode: outfilename = %s", _outfilename.c_str());

    // connecting to RobotEye
    try {
        std::cout << "creating RE05Driver" << std::endl;
        laserone = new ocular::RE05Driver(_sensor_IP.c_str());
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
//    _thrd = boost::thread(&roboteye_node::setIsRunning, this);
}

    roboteye_node::roboteye_node(double az_rate, int n_lines, int averaging, int laser_freq, bool intensity, std::string outfilename) : _sensor_IP("169.254.111.100")
    {
        _az_rate = az_rate;
        _N_lines = n_lines;
        _averaging = averaging;
        _laser_freq = laser_freq;
        _intensity = intensity;
        _outfilename = outfilename;

        _isrunning = false;

        _num_readings = 0;
        _lastStamp = ros::Time::now();

        // connecting to RobotEye
        try {
            std::cout << "creating RE05Driver" << std::endl;
            laserone = new ocular::RE05Driver(_sensor_IP.c_str());
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

//        _thrd = boost::thread(&roboteye_node::setIsRunning, this);
    }

    roboteye_node::~roboteye_node() {

        /// stopping laser and motors
        this->stop();
        /// printing the whole laser data acquired
        this->printAndWriteLaserData(_outfilename);
//        _laser_callback.getMutex().lock();
//        _laser_callback.getMutex().~Mutex();
//        _thrd.interrupt();
//        _thrd.~thread();
    }

    /// check parameters consistency
    bool roboteye_node::check_consistency() {

        if(_averaging < 1 || _averaging > 5){
            cout << "-avg must be an integer value from 1 to 5" << endl << endl;
            return false;
        }


        if(_laser_freq < 1 || (_laser_freq > 10000 && _laser_freq != 30000)){
            cout << "-lfreq must be an integer value in [1, 10000] or equal to 30000" << endl << endl;
            return false;
        }


        if(_az_rate > 15. || _az_rate == 0.){
            cout << "-az_rate must be a value in (0, 15]" << endl << endl;
            return false;
        }

        if(_N_lines < _az_rate){
            cout << "-nlines must be at least equal to az_rate" << endl << endl;
            return false;
        }

        if(_intensity && _laser_freq == 30000){
            cout << "-intensity can't be specified for lfreq values equal 30000" << endl << endl;
            return false;
        }
        return true;
    }

    void roboteye_node::dynamic_reconf_callback(robot_eye_driver::RobotEyeParametersConfig& config, uint32_t level) {
        ROS_INFO("Reconfigure Request: %d %f %d %d %d %s",
                 config.node_state,
                 config.az_rate,
                 config.n_lines,
                 config.averaging,
                 config.laser_freq,
                 config.intensity?"True":"False");

        _az_rate = config.az_rate;
        _N_lines = config.n_lines;
        _averaging = config.averaging;
        _laser_freq = config.laser_freq;
        _intensity = config.intensity;
        if(check_consistency())
        {
            if(config.node_state == 1)
                _node_state = RUN;
            else if(config.node_state == 0)
                _node_state = STOP;
            else if(config.node_state == 2)
                _node_state = PAUSE;
        }
        else
        {
            _node_state = PAUSE;
            cerr << "FUCK YOU!" << endl;
            ROS_WARN("Some inconsistency in parameters settings. Check the product manual.");
        }

        this->setIsRunning();
    }

    void roboteye_node::setConfig(roboteye::RobotEyeConfig& scan_config) {
        _scan_config.time_increment = scan_config.time_increment;
        _scan_config.scan_time = scan_config.scan_time;
    }

    void roboteye_node::setScan(roboteye::RobotEyeScan& scan) {
        _scan.intensities = scan.intensities;
        _scan.ranges = scan.ranges;
    }


    void roboteye_node::setIsRunning() {
        if(_node_state == RUN) {
            usleep(2e3);
            _isrunning = true;
            this->roboteyeRun();
        }
        else if(_node_state == STOP) {
            usleep(2e3);
            _isrunning = false;
            this->roboteyeStop();
        }
        else if(_node_state == PAUSE) {
            usleep(2e3);
            _isrunning = false;
            this->roboteyePause();
        }
    }

    void roboteye_node::roboteyePause() {
      // to be done:
      //for now simply put the laser in home position or in a specific desired position

        ROS_WARN_ONCE_NAMED("eval", "Pausing RobotEye Laser-Data Acquisition");
//        ocular::ocular_error_t err0 = laserone->Home();
//        if(err0 != 0) {
//            cout << "home error err: " << laserone->GetErrorString(err0) << endl;
//            exit(1);
//        }
        this->stop();
        this->setDesideredApertureAngles();
    }

    void roboteye_node::roboteyeStop() {

        ROS_WARN_ONCE_NAMED("eval", "Stopping RobotEye Laser-Data Acquisition");
        this->stop();
        this->setDesideredApertureAngles();
    }

    void roboteye_node::roboteyeRun(){

        ROS_WARN_ONCE_NAMED("eval", "IP address of the RobotEye Laser scanner: %s", _sensor_IP.c_str());

        /// homing the RobotEye, no need to call a sleep(), because Home() is blocking
        ocular::ocular_error_t err0 = laserone->Home();
        if(err0 != 0) {
            cout << "home error err: " << laserone->GetErrorString(err0) << endl;
            exit(1);
        }
        double azimuth, elevation;
        err0 = laserone->GetApertureAngles(azimuth, elevation);
//        cout << "the initial home position is:\tazimuth = " << azimuth << ",\televation = " << elevation << ", err: " << laserone->GetErrorString(err0) << endl;

        /// start a Full Field Scan
        err0 = laserone->StartFullFieldScan(_az_rate, _N_lines); // e.g. 3600 degrees/sec (10 revolution/sec) with NLines_min = 3600/360
        if(err0 != 0) {
            cout << "Start FullField scan err: " << laserone->GetErrorString(err0) << endl;
            exit(1);
        }
        /// start the Laser
        cout << "...starting streaming laser measurements" << endl;
        err0 = laserone->StartLaser(_laser_freq, _averaging, _intensity, &_laser_callback);

        ROS_WARN_ONCE_NAMED("eval", "First RobotEye Laser-Data Received");
        if(err0 != 0) {
            cout << "Start laser err: " << laserone->GetErrorString(err0) << endl;
            exit(1);
        }
    //    ocular::ocular_error_t err2 = laserone->StreamApertureAngles(5, &_angle_callback);
    //    cout << "...streaming aperture position" << ", err: " << laserone->GetErrorString(err2) << std::endl;

    //    cout << "going to sleep for " << _seconds << " seconds." << endl;
    //    sleep(_seconds);

    }

    void roboteye_node::stop() {

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

//      /// stopping aperture angles callback
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

    }

    void roboteye_node::setDesideredApertureAngles() {

//      /// setting a desired position
        cout << "setting a desired position" << endl;
        int attempts = 0;
        ocular::ocular_error_t err_ = laserone->SetApertureAngles(0.0f, -35.0f, 1, &_notif_callback); // look down
//        err_ = laserone->Home(); //home position is az=0, el=0;
        while(err_ != 0 && attempts < 10){
            err_ = laserone->SetApertureAngles(0.0f, -35.0f, 1, &_notif_callback);
//            err_ = laserone->Home();
            attempts++;
        }
        if(err_ != 0){
            std::cout << "got an error(setting position): " << laserone->GetErrorString(err_) << std::endl;
            exit(1);
        }
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

        /** Because of all the observations have the same size the
         *  total number of points are equal to the number of observations * size of one observation
         */
        EuclideanList& xyz_list = _laser_callback.xyzMeasList();
        unsigned int obs_dim = xyz_list.front().size();
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
        pcl::io::savePCDFileBinary(outfilename.c_str(), cloud);

        plotting.flush();
        plotting.close();
    }

}
