#ifndef PROCESSSYNCHRONIZATIONQUEUE_HPP_
#define PROCESSSYNCHRONIZATIONQUEUE_HPP_

#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>

#include "StereoImageMetaDataDefinitions.hpp"

struct ProcessSynchronizationQueue
{
    ProcessSynchronizationQueue()
        :  message_available_(false)
    {}

    //Mutex to protect access to the queue
    boost::interprocess::interprocess_mutex      mutex_;

    //Condition to wait when the queue is empty
    boost::interprocess::interprocess_condition  condition_empty_;

    //Condition to wait when the queue is full
    boost::interprocess::interprocess_condition  condition_full_;

    //Number of remaining sub-messages to load
    std::uint8_t number_remaining_sub_messages_;

    //Is there any message
    bool message_available_;
   
    //Additional meta information on image
    ImageMetaData message_image_information_;
};

#endif // #ifndef PROCESSSYNCHRONIZATIONQUEUE_HPP_
