#include "roboteye_node.h"

using namespace std;

// parameters
std::string outfilename;
std::string _sensor_IP;
int _seconds;
double _az_rate;
double _N_lines;
double _laser_freq;
double _averaging;
bool _intensity;

void init_parameters(int argc, char ** argv){

    // initialize parameters
    _sensor_IP = "169.254.111.100";
    g2o::CommandArgs arg;
    arg.param("t", _seconds, 15, "choose for how many seconds you want to exec this routine");
    arg.param("az_rate", _az_rate, 10, "set the number of rounds per second [max:15(5400 degrees/sec)]");
    arg.param("nlines", _N_lines, 100, "set the number of horizontal lines (vertical resolution) [min:_az_rate]");
    arg.param("avg", _averaging, 1, "set the 'averaging' value range:[1,5]");
    arg.param("lfreq", _laser_freq, 10000, "set the measurement frequency [range:[1, 30000]]");
    arg.param("intensity", _intensity, 0, "enable the streaming of intensity values [can be 'on' only if lfreq <= 10000] default off");
    arg.param("o", outfilename, "Eyebot_pcd.pcd", "output filename of a pcd format file");

    arg.parseArgs(argc, argv);

    // check parameters consistency
    if(_averaging > 5){
        cout << "-avg must be an integer value from 1 to 5" << endl << endl;
        exit(1);
    }

    if(_laser_freq > 30000 || _laser_freq == 0){
        cout << "-lfreq must be an integer value in [1, 30000]" << endl << endl;
        exit(1);
    }

    if(_az_rate > 15 || _az_rate == 0){
        cout << "-az_rate must be a value in (0, 15]" << endl << endl;
        exit(1);
    }

    if(_N_lines < _az_rate){
        cout << "-nlines must be at least equal to az_rate" << endl << endl;
        exit(1);
    }

    if(_intensity && _laser_freq > 10000){
        cout << "-intensity can't be specified for lfreq values higher than 10000" << endl << endl;
        exit(1);
    }

    if(_seconds < 1){
        cout << "-t must be an integer positive non zero value" << endl << endl;
        exit(1);
    }

    // print parameters
    cout << "SENSOR IP:\t" << _sensor_IP << endl;
    cout << "LASER FREQUENCY:\t" << _laser_freq << endl;
    cout << "AVERAGING:\t" << _averaging << endl;
    cout << "GET INTENSITY:\t" << (_intensity? "YES" : "NO") << endl;
    cout << "AZIMUTH RATE:\t" << _az_rate << endl;
    cout << "N_LINES:\t" << _N_lines << endl;
    cout << "RUNNING FOR:\t" << _seconds << " SECONDS" << endl;

}

int main(int argc, char **argv)
{
    cout << "=== LASERONE node ===" << endl << endl;

    init_parameters(argc,argv);

    ros::init(argc, argv, "roboteye_node");
    roboteye_node re;
    ros::Rate r(10);
    ros::spin();


    cout << endl;
    cout << "=== END ===" << endl;
    return 0;
}
