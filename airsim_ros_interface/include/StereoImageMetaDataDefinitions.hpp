#ifndef STEREOIMAGEMETADATADEFINITIONS_HPP_
#define STEREOIMAGEMETADATADEFINITIONS_HPP_

enum class StereoImageType
{
    Unknown = 0,
    LeftStereoImage,
    RightStereoImage,
    DepthPlannerImage,
    DepthPerspectiveImage,
    DepthVisImage,
    DisparityNormalizedImage,
    SegmentationImage,
    SurfaceNormalsImage
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
