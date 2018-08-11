///
/// @file
/// @copyright Copyright (C) 2017-2018, Jonathan Bryan Schmalhofer
///
/// @brief Definition of additional datatypes to specify images in shared memory queue
///
#ifndef STEREOIMAGEMETADATADEFINITIONS_HPP_
#define STEREOIMAGEMETADATADEFINITIONS_HPP_

enum class StereoImageType
{
    Unknown = 0,
    LeftStereoImage = 1,
    RightStereoImage = 2,
    DepthPlannerImage = 3,
    DepthPerspectiveImage = 4,
    DepthVisImage = 5,
    DisparityNormalizedImage = 6,
    SegmentationImage = 7,
    SurfaceNormalsImage = 8
};

struct ImageMetaData
{
    ImageMetaData()
        :  image_is_bigendian_(false), header_frame_id_(""), image_encoding_(""), type_(StereoImageType::Unknown)
    {}
    
    std::uint8_t header_stamp_sec_;
    std::uint8_t header_stamp_nsec_;
    std::uint8_t header_seq_;
    std::string header_frame_id_;
    std::uint32_t image_height_;
    std::uint32_t image_width_;
    std::string image_encoding_;
    bool image_is_bigendian_;
    std::uint32_t image_step_;
    StereoImageType type_;
};

#endif // #ifndef STEREOIMAGEMETADATADEFINITIONS_HPP_
