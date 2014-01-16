#include "roboteye_node.h"

using namespace std;

//global parameters
double az_rate;
double n_lines;
double laser_freq;
double averaging;
bool intensity;

void init_parameters(int argc, char ** argv){

    // initialize parameters
    std::string outfilename;
//    int seconds;
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

int main(int argc, char **argv)
{
    cout << "=== LASERONE node ===" << endl << endl;

    init_parameters(argc,argv);

    ros::init(argc, argv, "roboteye_node");
    ros::Time::init();
    roboteye_node re(az_rate, n_lines, laser_freq, averaging, intensity);
    ros::Rate r(10);
    ros::spin();


    cout << endl;
    cout << "=== END ===" << endl;
    return 0;
}
