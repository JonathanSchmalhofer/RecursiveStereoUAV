///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief Definition of datatypes for shared memory queue adapted from boost example
///
#ifndef PROCESSSYNCHRONIZATIONQUEUE_HPP_
#define PROCESSSYNCHRONIZATIONQUEUE_HPP_

#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>

#include "StereoImageMetaDataDefinitions.hpp"

namespace airsim_to_ros
{

struct Vector3d
{
    Vector3d()
        : x_(0), y_(0), z_(0)
    {}
    
    // the x coordinate
    double x_;
    
    // the y coordinate
    double y_;
    
    // the z coordinate
    double z_;
};

struct Quaternion3d
{
    Quaternion3d()
        : x_(0), y_(0), z_(0), w_(0)
    {}
    
    // the x coordinate
    double x_;
    
    // the y coordinate
    double y_;
    
    // the z coordinate
    double z_;
    
    // the w coordinate
    double w_;
};

struct Pose3d
{
    Pose3d()
        : position_(), orientation_()
    {}
    
    // the reference point for position and rotation (orientation)
    Vector3d position_;
    
    // the relative orientation of the object w.r.t its parent frame
    Quaternion3d orientation_;
};

struct ProcessSynchronizationQueue
{
    ProcessSynchronizationQueue()
        : message_available_(false), initial_pose_()
    {}

    //Mutex to protect access to the queue
    boost::interprocess::interprocess_mutex      mutex_;

    //Condition to wait when the queue is empty
    boost::interprocess::interprocess_condition  condition_empty_;

    //Condition to wait when the queue is full
    boost::interprocess::interprocess_condition  condition_full_;

    //Condition to wait until new trajectory was set
    boost::interprocess::interprocess_condition  condition_trajectory_;

    //Number of remaining sub-messages to load
    std::uint8_t number_remaining_sub_messages_;

    //Is there any message
    bool message_available_;
   
    //Additional meta information on image
    ImageMetaData message_image_information_;
    
    //Initial pose
    Pose3d initial_pose_;
    
    //New trajectory pose
    Pose3d trajectory_pose_;
};

} // namespace airsim_to_ros

#endif // #ifndef PROCESSSYNCHRONIZATIONQUEUE_HPP_
