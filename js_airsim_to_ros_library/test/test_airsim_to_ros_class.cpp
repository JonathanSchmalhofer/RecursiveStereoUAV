#include <gtest/gtest.h>
#include <random>
#include "js_airsim_to_ros_library/airsim_to_ros_class.h"

std::string generateRandomPortNumber(int length)
{
    const std::string VALID_CHARS = "0123456789";
    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0,VALID_CHARS.size() - 1);
    std::string your_random_string;
    std::generate_n(std::back_inserter(your_random_string), length, [&]()
    {
        return VALID_CHARS[distribution(generator)];
    });
    
    return your_random_string;
}

std::uint8_t isBigEndian()
{
    // Source: https://stackoverflow.com/questions/280162/is-there-a-way-to-do-a-c-style-compile-time-assertion-to-determine-machines-e
    union {
        long int l;
        char c[sizeof (long int)];
    } u;

    u.l = 1;

    if (u.c[sizeof(long int)-1] == 1)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

// Check for size of empty message to be zero /////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(AirSimToRosClassTestSuite, NoMessageReceived)
{
    // ARRANGE
    auto airsim_to_ros = js_airsim_to_ros_library::AirSimToRosClass("tcp://localhost:5564");

    // ACT
    auto empty_size = airsim_to_ros.GetReceivedMessageSize();

    // ASSERT
    EXPECT_EQ(empty_size, 0);
}

// Check for received Message to be correct ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(AirSimToRosClassTestSuite, ReceivedMessageCorrect)
{
    // ARRANGE
    std::string protocol = "tcp://";
    std::string port = generateRandomPortNumber(5); //"5676";
    std::string publisher_address = protocol + "*:" + port;
    std::string subscriber_address = protocol + "localhost:" + port;
    auto airsim_to_ros = js_airsim_to_ros_library::AirSimToRosClass(subscriber_address);
    
    zmq::context_t context = zmq::context_t(1);
    zmq::socket_t publishSocket = zmq::socket_t(context, ZMQ_PUB); //  We send updates via this socket
    publishSocket.setsockopt(ZMQ_SNDTIMEO, 50);
    publishSocket.bind(publisher_address);
    
    flatbuffers::FlatBufferBuilder fbb;
    // Image.header.stamp
    airsim_to_ros::time message_time(42,24);
    // Image.header
    auto header = airsim_to_ros::CreateHeader(
        fbb, 
        21, 
        &message_time,
        fbb.CreateString("test-header-ID"));
    fbb.Finish(header);
    // Image
    std::vector<std::uint8_t> image_data(3 * 640 * 480);
    auto image = airsim_to_ros::CreateImage(
        fbb, 
        header, 
        480, 
        640, 
        fbb.CreateString("rgb8"), 
        isBigEndian(), 
        sizeof(std::uint8_t) * 3 * 640, 
        fbb.CreateVector<std::uint8_t>(image_data));
    fbb.Finish(image);

    // ACT
    int buffer_size = fbb.GetSize();
    zmq::message_t image_msg(buffer_size);
    memcpy((void *)image_msg.data(), fbb.GetBufferPointer(), buffer_size);
    int sent_bytes = publishSocket.send(image_msg);
    usleep(100000);

    // ASSERT
    EXPECT_GT(sent_bytes, 0); // Image was sent
    EXPECT_EQ(airsim_to_ros.GetReceivedMessageSize(), 0); // No image yet received, because it was not yet retreived
    EXPECT_TRUE(airsim_to_ros.ReceivedMessage()); // Image was received
    EXPECT_GT(airsim_to_ros.GetReceivedMessageSize(), 0); // Received image is not empty
    EXPECT_EQ(airsim_to_ros.GetReceivedMessageSize(), buffer_size); // Received Image has same size as sent image
    
    // ACT #2
    airsim_to_ros::ImageBuilder builder(fbb);
    auto image_rcvd = airsim_to_ros::GetImage(airsim_to_ros.GetReceivedMessageData());
    
    // ASSERT #2
    EXPECT_EQ(image_rcvd->header()->seq(), 21);
    EXPECT_EQ(image_rcvd->header()->stamp()->sec(), 42);
    EXPECT_EQ(image_rcvd->header()->stamp()->nsec(), 24);
    EXPECT_STREQ(image_rcvd->header()->frame_id()->c_str(), "test-header-ID");
    EXPECT_EQ(image_rcvd->height(), 480);
    EXPECT_EQ(image_rcvd->width(), 640);
    EXPECT_STREQ(image_rcvd->encoding()->c_str(), "rgb8");
    EXPECT_EQ(image_rcvd->is_bigendian(), isBigEndian());
    EXPECT_EQ(image_rcvd->step(), sizeof(std::uint8_t) * 3 * 640);
    EXPECT_EQ(image_rcvd->data()->size(), 3 * 640 * 480);
    
    
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
