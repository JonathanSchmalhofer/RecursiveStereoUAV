///
/// @file
///
/// @brief Type for OXTS Data from KITTI DataSets - copied from https://github.com/rpng/kitti_parser/blob/master/src/kitti_parser/types/gpsimu_t.h
///
#ifndef OXTS_TYPE_HPP_
#define OXTS_TYPE_HPP_

namespace kitti_utils
{
    typedef struct oxts_data
    {
        // @brief timestamp
        long timestamp;

        // @brief lat:   latitude of the oxts-unit (deg)
        double lat;
        // @brief lon:   longitude of the oxts-unit (deg)
        double lon;
        // @brief alt:   altitude of the oxts-unit (m)
        double alt;
        // @brief roll:  roll angle (rad),    0 = level, positive = left side up,      range: -pi   .. +pi
        double roll;
        // @brief pitch: pitch angle (rad),   0 = level, positive = front down,        range: -pi/2 .. +pi/2
        double pitch;
        // @brief yaw:   heading (rad),       0 = east,  positive = counter clockwise, range: -pi   .. +pi
        double yaw;


        // @brief vn:    velocity towards north (m/s)
        double vn;
        // @brief ve:    velocity towards east (m/s)
        double ve;
        // @brief vf:    forward velocity, i.e. parallel to earth-surface (m/s)
        double vf;
        // @brief vl:    leftward velocity, i.e. parallel to earth-surface (m/s)
        double vl;
        // @brief vu:    upward velocity, i.e. perpendicular to earth-surface (m/s)
        double vu;


        // @brief ax:    acceleration in x, i.e. in direction of vehicle front (m/s^2)
        double ax;
        // @brief ay:    acceleration in y, i.e. in direction of vehicle left (m/s^2)
        double ay;
        // @brief az:    acceleration in z, i.e. in direction of vehicle top (m/s^2)
        double az;
        // @brief af:    forward acceleration (m/s^2)
        double af;
        // @brief al:    leftward acceleration (m/s^2)
        double al;
        // @brief au:    upward acceleration (m/s^2)
        double au;
        // @brief wx:    angular rate around x (rad/s)
        double wx;
        // @brief wy:    angular rate around y (rad/s)
        double wy;
        // @brief wz:    angular rate around z (rad/s)
        double wz;
        // @brief wf:    angular rate around forward axis (rad/s)
        double wf;
        // @brief wl:    angular rate around leftward axis (rad/s)
        double wl;
        // @brief wu:    angular rate around upward axis (rad/s)
        double wu;


        // @brief pos_accuracy:  velocity accuracy (north/east in m)
        double pos_accuracy;
        // @brief vel_accuracy:  velocity accuracy (north/east in m/s)
        double vel_accuracy;
        // @brief navstat:       navigation status (see navstat_to_string)
        int navstat;
        // @brief numsats:       number of satellites tracked by primary GPS receiver
        int numsats;
        // @brief posmode:       position mode of primary GPS receiver (see gps_mode_to_string)
        int posmode;
        // @brief velmode:       velocity mode of primary GPS receiver (see gps_mode_to_string)
        int velmode;
        // @brief orimode:       orientation mode of primary GPS receiver (see gps_mode_to_string)
        int orimode;
    } oxts_data;

}  // namespace kitti_utils

#endif  // KITTI_UTILS_H_
