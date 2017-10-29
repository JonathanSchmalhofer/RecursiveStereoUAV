#ifndef PROCESSSYNCHRONIZATIONQUEUE_HPP_
#define PROCESSSYNCHRONIZATIONQUEUE_HPP_

// modified example from http://www.boost.org/doc/libs/1_37_0/doc/html/interprocess/synchronization_mechanisms.html

#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>

struct ProcessSynchronizationQueue
{
    ProcessSynchronizationQueue()
        : message_available_(false)
    {}
    
    //Mutex to protect access to the queue
    boost::interprocess::interprocess_mutex      mutex_;
    
    //Condition to wait when the queue is empty
    boost::interprocess::interprocess_condition  condition_empty_;
    
    //Condition to wait when the queue is full
    boost::interprocess::interprocess_condition  condition_full_;
    
    //Is there any message
    bool message_available_;
};

#endif // #ifndef PROCESSSYNCHRONIZATIONQUEUE_HPP_
