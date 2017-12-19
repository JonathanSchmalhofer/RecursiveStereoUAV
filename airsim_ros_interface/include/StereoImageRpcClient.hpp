///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief Definition of class StereoImageRpcClient based on original AirSim code
///
#ifndef STEREOIMAGERPCCLIENT_HPP_
#define STEREOIMAGERPCCLIENT_HPP_

#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
#include <common/Common.hpp>
#include <utility> // for std::pair

#include "StereoImageMetaDataDefinitions.hpp"

class StereoImageRpcClient
{
    public:
        StereoImageRpcClient();
        ~StereoImageRpcClient();
        void Connect();
        std::vector<std::pair<StereoImageType,msr::airlib::VehicleCameraBase::ImageResponse>> RequestStereoImagesAndDepthImage();
        bool HasCollided();
        void SetPositionAndOrientation(const msr::airlib::Pose& Pose);
        msr::airlib::Pose GetPositionAndOrientation();
        msr::airlib::GeoPoint GetGnssGeoPoint();
    private:
        struct client_accessor;
        std::unique_ptr<client_accessor> pclient_accessor_;
};

#endif // #ifndef STEREOIMAGERPCCLIENT_HPP_
