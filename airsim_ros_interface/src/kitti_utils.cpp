///
/// @file
/// @copyright Copyright (C) 2017-2018, Jonathan Bryan Schmalhofer
///
/// @brief KITTI DataSet Utils
///

#include "kitti_utils.hpp"

namespace kitti_utils
{

// Source for GetFileList(): https://gist.github.com/vivithemage/9517678
std::vector<std::string> GetFileList(const std::string& path, const std::string extension)
{
    std::vector<std::string> file_list;
    if (!path.empty())
    {
        boost::filesystem::path apk_path(path);
        boost::filesystem::recursive_directory_iterator end;

        for (boost::filesystem::recursive_directory_iterator i(apk_path); i != end; ++i)
        {
            const boost::filesystem::path cp = (*i);
            if (extension == boost::filesystem::extension(cp.string()))
            {
                file_list.push_back(cp.string());
            }
        }
    }
    std::sort(file_list.begin(), file_list.end());
    return file_list;
}

/**
 * Read the oxts-Textfile according to the frame number/index idx.
 *
 * Copied from: https://github.com/rpng/kitti_parser/blob/master/src/kitti_parser/util/Loader.cpp
 */
oxts_data KittiUtils::GetOxtsData(std::vector<std::string> filelist, std::vector<long> timestamps, std::size_t idx, double &scale)
{
    // Make new measurement
    oxts_data datapoint;
    datapoint.timestamp = timestamps.at(idx);

    // Read in file of doubles
    std::vector<double> values;
    std::ifstream file_at_idx(filelist.at(idx), std::ios::in);

    // Keep storing values from the text file so long as data exists:
    double num = 0.0;
    while (file_at_idx >> num)
    {
        values.push_back(num);
    }

    // Set values, see the file 'dataformat.txt' for details on each one
    datapoint.lat = values.at(0);
    datapoint.lon = values.at(1);
    datapoint.alt = values.at(2);
    datapoint.roll = values.at(3);
    datapoint.pitch = values.at(4);
    datapoint.yaw = values.at(5);

    datapoint.vn = values.at(6);
    datapoint.ve = values.at(7);
    datapoint.vf = values.at(8);
    datapoint.vl = values.at(9);
    datapoint.vu = values.at(10);

    datapoint.ax = values.at(11);
    datapoint.ay = values.at(12);
    datapoint.az = values.at(13);
    datapoint.af = values.at(14);
    datapoint.al = values.at(15);
    datapoint.au = values.at(16);
    datapoint.wx = values.at(17);
    datapoint.wy = values.at(18);
    datapoint.wz = values.at(19);
    datapoint.wf = values.at(20);
    datapoint.wl = values.at(21);
    datapoint.wu = values.at(22);

    datapoint.pos_accuracy = values.at(23);
    datapoint.vel_accuracy = values.at(24);


    datapoint.navstat = (int)values.at(25);
    datapoint.numsats = (int)values.at(26);
    datapoint.posmode = (int)values.at(27);
    datapoint.velmode = (int)values.at(28);
    datapoint.orimode = (int)values.at(29);
    
    
    // also read first file
    std::ifstream first_file(filelist.at(0), std::ios::in);
    values.clear();

    // Keep storing values from the text file so long as data exists
    num = 0.0;
    while (first_file >> num)
    {
        values.push_back(num);
    }
    
    // compute mercator scale from latitude - always use first (!) latitude value for this, not idx
    // see also convertOxtsToPose.m in MATLAB dev kit from KITTI dataset
    scale = std::cos(values.at(0) * boost::math::constants::pi<double>() / 180.0f);

    // Return it
    return datapoint;
}

boost::numeric::ublas::matrix<double> KittiUtils::ConvertOxtsToPose(oxts_data datapoint, double scale)
{
    // Compare convertOxtsToPose.m from MATLAB DevKit of KITTI Dataset
    boost::numeric::ublas::matrix<double> Tr_0(4, 4);
    boost::numeric::ublas::matrix<double> pose(4, 4);
    boost::numeric::ublas::matrix<double> Rx(3, 3);
    boost::numeric::ublas::matrix<double> Ry(3, 3);
    boost::numeric::ublas::matrix<double> Rz(3, 3);
    boost::numeric::ublas::matrix<double> R(3, 3);
    
    // converts lat/lon coordinates to mercator coordinates using mercator scale
    // see latlonToMercator.m from MATLAB DevKit of KITTI Dataset
    double er = 6378137;
    double mx = scale * oxts_data.lon * boost::math::constants::pi<double>() * er / 180.0f;
    double my = scale * er * std::log( std::tan((90.0f+oxts_data.lat) * boost::math::constants::pi<double>() / 360.0f) );
    double mz = oxts_data.alt;
    
    // rotation matrix (OXTS RT3000 user manual, page 71/92)
    //
    // base => nav  (level oxts => rotated oxts)
    Rx(0,0) = 1;                            Rx(0,1) = 0;                        Rx(0,2) = 0;
    Rx(1,0) = 0;                            Rx(1,1) = std::cos(oxts_data.roll); Rx(1,2) = -std::sin(oxts_data.roll);
    Rx(2,0) = 0;                            Rx(2,1) = std::sin(oxts_data.roll); Rx(2,2) =  std::cos(oxts_data.roll);
    // base => nav  (level oxts => rotated oxts)
    Ry(0,0) =  std::cos(oxts_data.pitch);   Ry(0,1) = 0;                        Ry(0,2) = std::sin(oxts_data.pitch);
    Ry(1,0) =  0;                           Rx(1,1) = 1;                        Ry(1,2) = 0;
    Ry(2,0) = -std::sin(oxts_data.pitch);   Ry(2,1) = 0;                        Ry(2,2) =  std::cos(oxts_data.pitch);
    // base => nav  (level oxts => rotated oxts)
    Rz(0,0) = std::cos(oxts_data.yaw);      Rz(0,1) = -std::sin(oxts_data.yaw); Rz(0,2) = 0;
    Rz(1,0) = std::sin(oxts_data.yaw);      Rz(1,1) =  std::cos(oxts_data.yaw); Rz(1,2) = 0;
    Rz(2,0) = 0;                            Rz(2,1) =  0;                       Rz(2,2) =  1;
    // R = Rz*Ry*Rx;
    R = boost::numeric::ublas::prod(Ry, Rx);
    R = boost::numeric::ublas::prod(Rz, R);
    
    // in Matlab notation: Tr_0 = [R t;0 0 0 1];
    project(Tr_0, (0,2), (0,2)) = R;
    Tr_0(0,3)                   = mx;
    Tr_0(1,3)                   = my;
    Tr_0(2,3)                   = mz;
    Tr_0(3,0) = 0; Tr_0(3,1) = 0; Tr_0(3,2) = 0;  Tr_0(3,3) = 1;
    
    // in Matlab notation: pose = inv(Tr_0) * [R t;0 0 0 1]

    /*
    for (unsigned i = 0; i < m.size1 (); ++ i)
        for (unsigned j = 0; j < m.size2 (); ++ j)
            m (i, j) = 3 * i + j;
    */
    return pose;
}

KittiUtils::KittiUtils()
{
}

KittiUtils::~KittiUtils()
{
}

/**
 * Loads a timestamp file based on the path given
 *
 * Copied from: https://github.com/rpng/kitti_parser/blob/master/src/kitti_parser/util/Loader.cpp
 */
void KittiUtils::LoadTimestamps(std::string path_timestamp_file, std::vector<long>& time, int& counter)
{
    // Open the timestamp file
    std::string line;
    std::ifstream file_time(path_timestamp_file);
    // Load the timestamps
    while(getline(file_time, line))
    {
        // Skip empty lines
        if(line.empty())
            continue;
        // Parse data
        // http://www.boost.org/doc/libs/1_55_0/doc/html/date_time/posix_time.html#posix_ex
        boost::posix_time::ptime posix_timestamp(boost::posix_time::time_from_string(line));
        // Convert to long, subtract
        long timestamp = (posix_timestamp - boost::posix_time::ptime{{1970,1,1}, {}}).total_milliseconds();
        // Append
        time.push_back(timestamp);
        // Incrememt
        counter++;
        // Debug
        //cout << line << " => " << timestamp << endl;
    }
    // Close file
    file_time.close();
}

/**
 * This will read in the GPS/IMU timestamp file
 * From this it will also create the paths needed
 */
void KittiUtils::LoadOxtsLitePaths(std::string path_oxts_folder, std::vector<long>& time, std::vector<std::string>& filelist)
{
    // Load the timestamps for this type
    int count = 0;
    LoadTimestamps(path_oxts_folder+"timestamps.txt", time, count);
    
    filelist = GetFileList(path_oxts_folder+"data/", ".txt");
}

}  // namespace kitti_utils

