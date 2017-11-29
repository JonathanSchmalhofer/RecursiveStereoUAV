///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief Implementation of class StereoImageRpcClient based on original AirSim code
///
#include "StereoImageRpcClient.hpp"

struct StereoImageRpcClient::client_accessor
{
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

void StereoImageRpcClient::Connect()
{
    pclient_accessor_->client_.confirmConnection();
}

std::vector<std::pair<StereoImageType,msr::airlib::VehicleCameraBase::ImageResponse>> StereoImageRpcClient::RequestStereoImagesAndDepthImage()
{
    const bool pixels_as_float_true = true;
    const bool pixels_as_float_false = false;
    const bool compress_true = true;
    const bool compress_false = false;
    
    std::vector<msr::airlib::VehicleCameraBase::ImageRequest> request_left_image = { msr::airlib::VehicleCameraBase::ImageRequest(0, msr::airlib::VehicleCameraBase::ImageType::Scene, pixels_as_float_false, compress_false) };
    std::vector<msr::airlib::VehicleCameraBase::ImageRequest> request_right_image = { msr::airlib::VehicleCameraBase::ImageRequest(1, msr::airlib::VehicleCameraBase::ImageType::Scene, pixels_as_float_false, compress_false) };
    std::vector<msr::airlib::VehicleCameraBase::ImageRequest> request_depthplanner_image = { msr::airlib::VehicleCameraBase::ImageRequest(1, msr::airlib::VehicleCameraBase::ImageType::DepthPlanner, pixels_as_float_false, compress_false) };
    
    const std::vector<msr::airlib::VehicleCameraBase::ImageResponse> response_left_image = pclient_accessor_->client_.simGetImages(request_left_image);
    const std::vector<msr::airlib::VehicleCameraBase::ImageResponse> response_right_image = pclient_accessor_->client_.simGetImages(request_right_image);
    const std::vector<msr::airlib::VehicleCameraBase::ImageResponse> response_depthplanner_image = pclient_accessor_->client_.simGetImages(request_depthplanner_image);
    
    std::vector<std::pair<StereoImageType,msr::airlib::VehicleCameraBase::ImageResponse>> response_cumulated;
    
    response_cumulated.emplace_back(StereoImageType::DepthPlannerImage, response_depthplanner_image.at(0));
    response_cumulated.emplace_back(StereoImageType::RightStereoImage, response_right_image.at(0));
    response_cumulated.emplace_back(StereoImageType::LeftStereoImage, response_left_image.at(0));
    
    return response_cumulated;
}

bool StereoImageRpcClient::HasCollided()
{
    const auto& collision_info = pclient_accessor_->client_.getCollisionInfo();
    return collision_info.has_collided;
}

void StereoImageRpcClient::SetPositionAndOrientation(const msr::airlib::Pose& Pose)
{
    const bool ignore_collision_true = true;
    pclient_accessor_->client_.simSetPose(Pose, ignore_collision_true);
}

msr::airlib::Pose StereoImageRpcClient::GetPositionAndOrientation()
{
    msr::airlib::Pose current_position(pclient_accessor_->client_.getPosition(), pclient_accessor_->client_.getOrientation());
    return current_position;
}
