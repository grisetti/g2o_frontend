#include "drawable_laser.h"
#include "graph_qglviewer.h"


using namespace std;



DrawableLaser::DrawableLaser() : Drawable()
{
    _laser = 0;
}


DrawableLaser::DrawableLaser(LaserRobotData* laser_) : Drawable()
{
    _laser = laser_;
}


void DrawableLaser::draw()
{
    LaserRobotData::Vector2fVector scan = _laser->floatCartesian();
    for(size_t i = 0; i < scan.size(); ++i)
    {
        glBegin(GL_POINTS);
        glColor4f(1.f, .0f, .0f, 1.f);
        glVertex3f(scan[i].x(), scan[i].y(), 0.f);
        glEnd();
    }
}
