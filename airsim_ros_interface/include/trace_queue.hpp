#ifndef TRACE_QUEUE_HPP
#define TRACE_QUEUE_HPP

#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>

struct image_information
{
    image_information()
      :  image_is_bigendian_(false), header_frame_id_(""), image_encoding_("")
    {}
    
    std::uint8_t header_stamp_sec_;
    std::uint8_t header_stamp_nsec_;
    std::uint8_t header_seq_;
    std::string  header_frame_id_;
    std::uint32_t  image_height_;
    std::uint32_t  image_width_;
    std::string  image_encoding_;
    bool  image_is_bigendian_;
    std::uint32_t  image_step_;
};

struct trace_queue
{
   trace_queue()
      :  message_available_(false)
   {}

   //Mutex to protect access to the queue
   boost::interprocess::interprocess_mutex      mutex_;

   //Condition to wait when the queue is empty
   boost::interprocess::interprocess_condition  condition_empty_;

   //Condition to wait when the queue is full
   boost::interprocess::interprocess_condition  condition_full_;

   //Number of sub-messages to load
   std::uint8_t number_sub_messages_;

   //Is there any message
   bool message_available_;
   
   //Additional information on image
   image_information message_image_information_;
};

#endif // #ifndef TRACE_QUEUE_HPP
