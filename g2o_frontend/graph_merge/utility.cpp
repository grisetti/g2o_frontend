#include "utility.h"



namespace utility
{
    Eigen::Vector3d t2v(const Eigen::Isometry2d& iso)
    {
        Eigen::Vector3d t;
        t.x() = iso.translation().x();
        t.y() = iso.translation().y();
        Eigen::Rotation2Dd r(0);
        r.fromRotationMatrix(iso.linear());
        t.z() = r.angle();
        return t;
    }


    Eigen::Isometry2d v2t(const Eigen::Vector3d& v)
    {
        Eigen::Isometry2d iso;
        iso.translation() = v.head<2>();
        iso.linear() = Eigen::Rotation2Dd(v.z()).toRotationMatrix();
        return iso;
    }
}
