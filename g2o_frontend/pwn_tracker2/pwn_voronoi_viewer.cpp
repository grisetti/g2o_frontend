#include <list>
#include <set>
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"
#include "g2o_frontend/boss_map/reference_frame.h"
#include "g2o_frontend/boss_map/reference_frame_relation.h"
#include "g2o_frontend/boss_map/image_sensor.h"
#include "g2o_frontend/boss_map/laser_sensor.h"
#include "g2o_frontend/boss_map/imu_sensor.h"
#include "g2o_frontend/boss_map/sensor_data_synchronizer.h"
#include "g2o_frontend/boss_map/robot_configuration.h"
#include "g2o_frontend/boss_map/map_manager.h"
#include "g2o_frontend/boss_map/sensor_data_node.h"
#include "g2o_frontend/boss_map_building/map_g2o_reflector.h"
#include "pwn_tracker.h"
#include "pwn_cloud_cache.h"
#include "pwn_closer.h"
#include "g2o_frontend/pwn_boss/pwn_io.h"
#include "pwn_tracker_viewer.h"
#include "manifold_voronoi_extractor.h"
#include "g2o_frontend/voronoi/voronoi_diagram.h"


#include <QApplication>

#define MARKUSED(X)  X=X

/*#include "g2o_frontend/pwn_boss/pwn_sensor_data.h"*/
using namespace manifold_voronoi;
using namespace pwn_tracker;
using namespace boss_map_building;
using namespace boss_map;
using namespace boss;
using namespace std;


int main(int argc, char** argv)
{
    string filein = argv[1];

    list<Serializable*> objects;
    Deserializer des;
    des.setFilePath(filein);
    StreamProcessor* sp=loadProcessor("mySLAMPipeline", des, objects);

    if (! sp){
        cerr << "object not found, aborting";
        return 0;
    }

    StreamProcessorGroup* group = dynamic_cast<StreamProcessorGroup*>(sp);
    if (! group) {
        cerr << "object is not a pipeline, aborting";
        return 0;
    }

    // retrieve the manager from the pipeline. You will have to copy it in the result
    size_t pos = 0;
    MapManager* manager = group->byType<MapManager>(pos);
    if (! manager) {
        cerr << "unable to find the manager" << endl;
        return 0;
    }

    PwnCloudCache* cache = group->byType<PwnCloudCache>(pos);
    if (! cache) {
        cerr << "unable to find the cache" << endl;
        return 0;
    }

    cerr << "algo config loaded" << endl;

    RobotConfiguration* conf = 0;
    Serializable* s;
    while ( (s=des.readObject()) ){
        cerr << s->className() << endl;
        objects.push_back(s);
        RobotConfiguration * conf_ = dynamic_cast<RobotConfiguration*>(s);
        if (conf_) {
            conf = conf_;
            break;
        }
    }
    if (! conf) {
        cerr << "unable to get robot configuration, aborting " << endl;
        cerr << "objects.size(): " << objects.size() << endl;
        return -1;
    }
    cerr << "robot config loaded" << endl;

    group->setRobotConfiguration(conf);

    VisState* visState = new VisState(manager);
    QApplication* app = new QApplication(argc,argv);
    MyTrackerViewer *viewer = new MyTrackerViewer(visState);
    viewer->show();
    int counter = 0;
    while((s=des.readObject()))
    {
        objects.push_back(s);
        NewKeyNodeMessage* km = dynamic_cast<NewKeyNodeMessage*>(s);
        if(km)
        {
            PwnCloudCache::HandleType h=cache->get((SyncSensorDataNode*) km->keyNode);
            VisCloud* visCloud = new VisCloud(h.get());
            visState->cloudMap.insert(make_pair(km->keyNode, visCloud));
            viewer->updateGL();
        }
        ManifoldVoronoiData* vdata = dynamic_cast<ManifoldVoronoiData*>(s);
        if(vdata)
        {
            ImageBLOB* blob = vdata->imageBlob().get();
            cv::Mat image = blob->cvImage();
            MapNode* mnode = vdata->node;
            Eigen::Isometry3d iso = mnode->transform();
            cerr << "Pose: " << iso.translation().transpose() << endl;
            ManifoldVoronoi* mv = new ManifoldVoronoi(image, 100);
            mv->fillQueue();
            mv->distmap();
            mv->distmap2image();
            std::ostringstream bob;
            bob << "voronois/distmap_" << counter << ".pgm";
            std::string s1 = bob.str();
            mv->savePGM(s1.c_str(), mv->_drawableDistmap);
            mv->distmap2voronoi();
            mv->diagram2graph();
            mv->graph();
            cv::Mat result = cv::Mat::zeros(image.rows, image.cols*3, CV_16UC1);
            for(int r = 0; r < image.rows; r++)
            {
                for(int c = 0; c < image.cols; c++)
                {
//                    if((*(mv->_distmap))(r,c).pushed() == true)
//                    {
//                        result.at<uint16_t>(r, c) = 65535;
//                    }
                    result.at<uint16_t>(r, c) = image.at<uint16_t>(r, c);
                    result.at<uint16_t>(r, c + image.cols*1) = mv->_voro->at<uint16_t>(r, c);
                    result.at<uint16_t>(r, c + image.cols*2) = mv->_graph->at<uint16_t>(r, c);
                }
            }
            cv::Mat all = cv::Mat::zeros(image.rows, image.cols, CV_16UC3);
            for(int r = 0; r < image.rows; r++)
            {
                for(int c = 0; c < image.cols; c++)
                {
                    if(image.at<uint16_t>(r, c) != 65535)
                    {
                        all.at<cv::Vec3s>(r, c) = cv::Vec3s(image.at<uint16_t>(r, c), image.at<uint16_t>(r, c), image.at<uint16_t>(r, c));
                    }
                    if(mv->_voro->at<uint16_t>(r, c) != 0)
                    {
                        all.at<cv::Vec3s>(r, c) = cv::Vec3s(0, 0, 65535);
                        Vector2i parent = (*(mv->_distmap))(r,c).parent();
                        all.at<cv::Vec3s>(parent.x(), parent.y()) = cv::Vec3s(65535, 65535, 65535);
                    }
                }
            }

            std::ostringstream os1;
            os1 << "voronois/composed_" << counter << ".pgm";
            std::string s = os1.str();
            cv::imwrite(s, result);
            std::ostringstream os2;
            os2 << "voronois/overlay_" << counter << ".pgm";
            s = os2.str();
            cv::imwrite(s, all);

            vdata->imageBlob().set(0);
            delete mv;
            counter++;
        }
        app->processEvents();
    }
    cout << "OUT" << endl;
    visState->final = true;

    viewer->updateGL();
    while(viewer->isVisible())
    {
        app->processEvents();
    }
}
