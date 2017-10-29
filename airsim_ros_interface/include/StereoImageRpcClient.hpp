#ifndef STEREOIMAGERPCCLIENT_HPP_
#define STEREOIMAGERPCCLIENT_HPP_

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/Common.hpp"

class StereoImageRpcClient
{
    public:
        StereoImageRpcClient();
        ~StereoImageRpcClient();
        void connect();
        std::vector<msr::airlib::VehicleCameraBase::ImageResponse> requestStereoImagesAndDepthImage();
        bool hasCollided();
        //void setPositionAndOrientation(msr::airlib::real_T position_x, msr::airlib::real_T position_y, msr::airlib::real_T position_z, msr::airlib::real_T orientation_w, msr::airlib::real_T orientation_x, msr::airlib::real_T orientation_y, msr::airlib::real_T orientation_z);
        void setPositionAndOrientation(const msr::airlib::Pose& Pose);
        msr::airlib::Pose getPositionAndOrientation();
    private:
        struct client_accessor;
        std::unique_ptr<client_accessor> pclient_accessor_;
};

#endif // #ifndef STEREOIMAGERPCCLIENT_HPP_
