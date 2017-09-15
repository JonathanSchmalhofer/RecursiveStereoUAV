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

// Check for default values of constructor /////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(AirSimToRosClassTestSuite, DefaultValues)
{
    // ARRANGE
    js_airsim_to_ros_library::AirSimToRosClass airsim_to_ros("tcp://localhost:5676");

    // ACT

    // ASSERT
    EXPECT_EQ(airsim_to_ros.GetImageHeaderSeq(), 0);
    EXPECT_EQ(airsim_to_ros.GetImageHeaderStampSec(), 0);
    EXPECT_EQ(airsim_to_ros.GetImageHeaderStampNsec(), 0);
    EXPECT_EQ(airsim_to_ros.GetImageHeaderFrameid(), "");
    EXPECT_EQ(airsim_to_ros.GetImageHeight(), 0);
    EXPECT_EQ(airsim_to_ros.GetImageWidth(), 0);
    EXPECT_EQ(airsim_to_ros.GetImageEncoding(), "");
    EXPECT_EQ(airsim_to_ros.GetImageIsBigendian(), 0);
    EXPECT_EQ(airsim_to_ros.GetImageStep(), 0);
    EXPECT_EQ(NULL, airsim_to_ros.GetImageData());
    EXPECT_EQ(airsim_to_ros.GetImageDataSize(), 0);
}


// Check for received message to be correct ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(AirSimToRosClassTestSuite, ReceivedMessageCorrect)
{
    // ARRANGE
    std::string protocol = "tcp://";
    std::string port = generateRandomPortNumber(5); //"5676";
    std::string publisher_address = protocol + "*:" + port;
    std::string subscriber_address = protocol + "localhost:" + port;
    js_airsim_to_ros_library::AirSimToRosClass airsim_to_ros(subscriber_address);
    
    zmq::context_t context = zmq::context_t(1);
    zmq::socket_t publishSocket = zmq::socket_t(context, ZMQ_PUB); //  We send updates via this socket
    publishSocket.setsockopt(ZMQ_SNDTIMEO, 500);
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
    int return_value = publishSocket.send(image_msg);
    usleep(100000);

    // ASSERT
    EXPECT_NE(return_value, 0); // Image was sent without error
    EXPECT_EQ(airsim_to_ros.GetImageDataSize(), 0); // No image yet received, because it was not yet retreived
    if (airsim_to_ros.ReceivedMessage())
    {
      EXPECT_GT(airsim_to_ros.GetImageDataSize(), 0); // Received image is not empty

      EXPECT_EQ(airsim_to_ros.GetImageHeaderSeq(), 21);
      EXPECT_EQ(airsim_to_ros.GetImageHeaderStampSec(), 42);
      EXPECT_EQ(airsim_to_ros.GetImageHeaderStampNsec(), 24);
      EXPECT_EQ(airsim_to_ros.GetImageHeaderFrameid(), "test-header-ID");
      EXPECT_EQ(airsim_to_ros.GetImageHeight(), 480);
      EXPECT_EQ(airsim_to_ros.GetImageWidth(), 640);
      EXPECT_EQ(airsim_to_ros.GetImageEncoding(), "rgb8");
      EXPECT_EQ(airsim_to_ros.GetImageIsBigendian(), isBigEndian());
      EXPECT_EQ(airsim_to_ros.GetImageStep(), sizeof(std::uint8_t) * 3 * 640);
      EXPECT_EQ(airsim_to_ros.GetImageDataSize(), 3 * 640 * 480);
    }
    
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
