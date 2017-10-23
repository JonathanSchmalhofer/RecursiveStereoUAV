// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/Common.hpp"

class StereoImageRpcClient
{
	public:
		StereoImageRpcClient();
		~StereoImageRpcClient();
		void connect();
		void StereoImageRpcClient::requestStereoImagesAndDepthImage();
		bool hasCollided();
		void setPosition(msr::airlib::real_T position_x, msr::airlib::real_T position_y, msr::airlib::real_T position_z, msr::airlib::real_T orientation_w, msr::airlib::real_T orientation_x, msr::airlib::real_T orientation_y, msr::airlib::real_T orientation_z);
	private:
		struct client_accessor;
		std::unique_ptr<client_accessor> pclient_accessor_;
};

