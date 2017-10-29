#include "StereoImageRpcClient.hpp"

struct StereoImageRpcClient::client_accessor {
    client_accessor()
    {
    }
    msr::airlib::MultirotorRpcLibClient client_;
};

StereoImageRpcClient::StereoImageRpcClient()
{
    pclient_accessor_.reset(new client_accessor());
}

StereoImageRpcClient::~StereoImageRpcClient()
{
    pclient_accessor_.reset();
    pclient_accessor_ = nullptr; // see: https://stackoverflow.com/questions/16151550/c11-when-clearing-shared-ptr-should-i-use-reset-or-set-to-nullptr
}

void StereoImageRpcClient::connect()
{
    pclient_accessor_->client_.confirmConnection();
}

std::vector<msr::airlib::VehicleCameraBase::ImageResponse> StereoImageRpcClient::requestStereoImagesAndDepthImage()
{
    const bool pixels_as_float_true = true;
    const bool pixels_as_float_false = false;
    const bool compress_true = true;
    const bool compress_false = false;
    
    //get right, left and depth images. First two as png, second as float16.
    std::vector<msr::airlib::VehicleCameraBase::ImageRequest> request =
    { 
        //png format
        msr::airlib::VehicleCameraBase::ImageRequest(0, msr::airlib::VehicleCameraBase::ImageType::Scene, pixels_as_float_false, compress_false),
        //uncompressed RGBA array bytes
        msr::airlib::VehicleCameraBase::ImageRequest(1, msr::airlib::VehicleCameraBase::ImageType::Scene, pixels_as_float_false, compress_false),       
        //floating point uncompressed image  
        msr::airlib::VehicleCameraBase::ImageRequest(1, msr::airlib::VehicleCameraBase::ImageType::DepthPlanner, pixels_as_float_true, compress_false) 
    };

    const std::vector<msr::airlib::VehicleCameraBase::ImageResponse>& response = pclient_accessor_->client_.simGetImages(request);
    
    return response;
}

bool StereoImageRpcClient::hasCollided()
{
    const auto& collision_info = pclient_accessor_->client_.getCollisionInfo();
    return collision_info.has_collided;
}
/*
void StereoImageRpcClient::setPositionAndOrientation(msr::airlib::real_T position_x, msr::airlib::real_T position_y, msr::airlib::real_T position_z, msr::airlib::real_T orientation_w, msr::airlib::real_T orientation_x, msr::airlib::real_T orientation_y, msr::airlib::real_T orientation_z )
{
    const bool ignore_collision_true = true;
    pclient_accessor_->client_.simSetPose(msr::airlib::Pose(msr::airlib::Vector3r(position_x, position_y, position_z), msr::airlib::Quaternionr(orientation_w, orientation_x, orientation_y, orientation_z)), ignore_collision_true);
}
*/

void StereoImageRpcClient::setPositionAndOrientation(const msr::airlib::Pose& Pose)
{
    const bool ignore_collision_true = true;
    pclient_accessor_->client_.simSetPose(Pose, ignore_collision_true);
}

msr::airlib::Pose StereoImageRpcClient::getPositionAndOrientation()
{
    msr::airlib::Pose current_position(pclient_accessor_->client_.getPosition(), pclient_accessor_->client_.getOrientation());
    return current_position;
}
