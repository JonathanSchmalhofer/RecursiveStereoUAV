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
#include <boost/math/constants/constants.hpp>   // pi - see: https://stackoverflow.com/questions/21867617/best-platform-independent-pi-constant
#include <boost/numeric/ublas/matrix_proxy.hpp>

#include <Eigen/Dense>    // for Eigen::MatrixXd
#include <Eigen/Geometry> // for eulerAngles()

#include "invert_matrix.hpp"

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
    double GetScale(std::vector<std::string> filelist);
    boost::numeric::ublas::matrix<double> GetTr0(std::vector<std::string> filelist, double scale);
    boost::numeric::ublas::matrix<double> GetTransformation(oxts_data datapoint, double scale);
    oxts_data GetOxtsData(std::vector<std::string> filelist, std::vector<long> timestamps, std::size_t idx);
    boost::numeric::ublas::matrix<double> ConvertOxtsToPose(oxts_data datapoint, double scale, boost::numeric::ublas::matrix<double> Tr_0_inv);
    void GetEulerAngles(boost::numeric::ublas::matrix<double> pose, double &roll, double &pitch, double &yaw);
    void GetPosition(boost::numeric::ublas::matrix<double> pose, double &x, double &y, double &z);
    
private:    
    
    
};
}  // namespace kitti_utils

#endif  // KITTI_UTILS_HPP_
