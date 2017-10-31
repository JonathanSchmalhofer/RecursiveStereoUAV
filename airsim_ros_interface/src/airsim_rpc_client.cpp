#include "StereoImageRpcClient.hpp"

/*
 * not needed if mutex is used for process synchronization
#include <chrono>  // for sleep_for
#include <thread>  // for sleep_for - see: https://stackoverflow.com/questions/4184468/sleep-for-milliseconds
*/

int main(int argc, const char *argv[])
{
    std::cout << "Press ENTER to start StereoImageRpcClient" << std::endl; std::cin.get();
    
    StereoImageRpcClient *client = new StereoImageRpcClient();    
    client->connect();
    msr::airlib::Pose initial_pose = client->getPositionAndOrientation();
    
    std::cout << "Initial Pose:" << std::endl;
    std::cout << " x = " << initial_pose.position.x() << std::endl;
    std::cout << " y = " << initial_pose.position.y() << std::endl;
    std::cout << " z = " << initial_pose.position.z() << std::endl; ;
    std::cout << " w = " << initial_pose.orientation.w() << std::endl; ;
    std::cout << " x = " << initial_pose.orientation.x() << std::endl; ;
    std::cout << " y = " << initial_pose.orientation.y() << std::endl; ;
    std::cout << " z = " << initial_pose.orientation.z() << std::endl;
    
    std::cout << "Start Main Loop?" << std::endl; std::cin.get();
    
    msr::airlib::Pose delta_pose, new_pose = initial_pose;
    
    while( true )
    {
        delta_pose = msr::airlib::Pose(msr::airlib::Vector3r(-0.01f, 0.0f, 0.01f), msr::airlib::Quaternionr(1.0f, 0.0f, 0.0f, 0.0f));
        new_pose = new_pose - delta_pose;
        
        client->setPositionAndOrientation(new_pose);
        
        std::cout << "Collided? Answer: " << client->hasCollided() << std::endl;
        
        std::vector<msr::airlib::VehicleCameraBase::ImageResponse> received_image_response_vector = client->requestStereoImagesAndDepthImage();
        
        /*
         * not needed if mutex is used for process synchronization
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        */
    }
    
    delete client;
    
    return 0;
}
