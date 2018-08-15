///
/// @file
/// @copyright Copyright (C) 2017-2018, Jonathan Bryan Schmalhofer
///
/// @brief Class with Utils for handling KITTI DataSet easier
///
#ifndef KITTI_UTILS_HPP_
#define KITTI_UTILS_HPP_

#include <cstdint>
#include <string>
#include <vector>
#include <fstream>      // std::ifstream
#include <iostream>
#include <cstddef>      // std::size_t - see: https://stackoverflow.com/questions/36594569/which-header-should-i-include-for-size-t
#include <cmath>        // cos

#include <boost/date_time.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/math/constants/constants.hpp> // pi - see: https://stackoverflow.com/questions/21867617/best-platform-independent-pi-constant

#include "oxts_type.hpp"

namespace kitti_utils
{

// Source for GetFileList(): https://gist.github.com/vivithemage/9517678
std::vector<std::string> GetFileList(const std::string& path, const std::string extension);

class KittiUtils
{
public:
    KittiUtils();
    
    ~KittiUtils();
    
    void LoadTimestamps(std::string path_timestamp_file, std::vector<long>& time, int& counter);
    void LoadOxtsLitePaths(std::string path_gpsimu_file, std::vector<long>& time, std::vector<std::string>& filelist);
    oxts_data GetOxtsData(std::vector<std::string> filelist, std::vector<long> timestamps, std::size_t idx, double &scale);
    boost::numeric::ublas::matrix<double> ConvertOxtsToPose(oxts_data datapoint, double scale);

private:    
    
    
};
}  // namespace kitti_utils

#endif  // KITTI_UTILS_HPP_
