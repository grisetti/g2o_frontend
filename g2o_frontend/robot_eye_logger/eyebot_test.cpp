#include <exception>
#include <iostream>
#include <boost/thread/thread.hpp>

#include "myCallbacks.h"

using namespace ocular;
using namespace std;

int main(int argc, char** argv) {

    AngleCB angle_callback;
    LaserCB laser_callback;
    NotificationCB notif_callback;

//    mutex_meas = new Mutex();
    ocular::RE05Driver* laserone;

    try {
        std::cout << "creating RE05Driver" << std::endl;
        // connect to RobotEye
        laserone = new ocular::RE05Driver("169.254.111.100");
    }
    catch(std::exception  &e) {
        std::cout << "Caught exception: " << e.what() << std::endl;
        return 1;
    }

    /// homing the RobotEye, Home() is blocking
    laserone->Home();

    double azimuth, elevation;
    ocular::ocular_error_t err0 = laserone->GetApertureAngles(azimuth, elevation);
    cout << "the initial home position is:\tazimuth = " << azimuth << ",\televation = " << elevation << ", err: " << laserone->GetErrorString(err0) << endl;

    /// start a Full Field Scan
    laserone->StartFullFieldScan(10, 100);

    ocular::ocular_error_t err1 = laserone->StartLaser(5000, 5, 0, &laser_callback);
    cout << "...starting streaming laser measurements" << endl;
    if(err1 != 0) {
        cout << "err: " << laserone->GetErrorString(err1) << endl;
        exit(1);
    }
    ocular::ocular_error_t err2 = laserone->StreamApertureAngles(5, &angle_callback);
    cout << "...streaming aperture position" << ", err: " << laserone->GetErrorString(err2) << std::endl;

    cout << "going to sleep" << endl;
    sleep(15);

    cout << "setting a desired final position" << endl;
    laserone->SetApertureAngles(0.0f, 180.0f, 1, &notif_callback);
    /// home position should be az=0, el=0;
    // laserone->Home(&notif_callback);

    cout << "stopping laser" << endl;
    laserone->StopLaser();
    cout << "stopping laser motors" << endl;
    laserone->Stop();
    cout << "stopping aperture angles streaming" << endl;
    laserone->StopStreamingApertureAngles();

    /// showing the laser data
    //to prevent possible later callbacks
//    mutex_meas->lock();
    for(int i = 0; i <= measurements.size(); i++) {
        ocular::ocular_rbe_obs_t meas = measurements[i];
        cout << "azim: " << meas.azimuth << ",\telev: " << meas.elevation << ",\trange: " << meas.range << ",\tintensity: " << meas.intensity << endl;
    }
//    mutex_meas->unlock();

    cout << endl;
    cout << "=== END ===" << endl;

    return 0;
}

