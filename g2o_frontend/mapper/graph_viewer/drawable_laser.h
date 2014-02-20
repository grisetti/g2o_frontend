#ifndef DRAWABLE_LASER_H
#define DRAWABLE_LASER_H

#include "drawable.h"
#include "g2o_frontend/sensor_data/laser_robot_data.h"
#include <vector>



class DrawableLaser : public Drawable
{

public:
    DrawableLaser();
    DrawableLaser(LaserRobotData* laser_);

    inline virtual void setLaser(LaserRobotData* laser_) { _laser = laser_; }
    inline virtual LaserRobotData* laser() { return _laser; }
    inline virtual const LaserRobotData* laser() const { return _laser; }

    virtual void draw();

protected:
    LaserRobotData* _laser;
};
#endif // DRAWABLE_LASER_H
