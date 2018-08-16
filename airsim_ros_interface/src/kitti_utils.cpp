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
oxts_data KittiUtils::GetOxtsData(std::vector<std::string> filelist, std::vector<long> timestamps, std::size_t idx)
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

    // Return it
    return datapoint;
}


double KittiUtils::GetScale(std::vector<std::string> filelist)
{
    // also read first file
    std::vector<double> values;
    std::ifstream first_file(filelist.at(0), std::ios::in);

    // Keep storing values from the text file so long as data exists
    double num = 0.0;
    while (first_file >> num)
    {
        values.push_back(num);
    }
    
    // compute mercator scale from latitude - always use first (!) latitude value for this, not idx
    // see also convertOxtsToPose.m in MATLAB dev kit from KITTI dataset
    double scale = std::cos(values.at(0) * boost::math::constants::pi<double>() / 180.0f);

    // Return it
    return scale;
}

boost::numeric::ublas::matrix<double> KittiUtils::GetTr0(std::vector<std::string> filelist, double scale)
{
    boost::numeric::ublas::matrix<double> Tr_0(4, 4);
    std::vector<long> empty_timestamps;
    empty_timestamps.push_back(0);
    
    // Get the first point of the oxts data...
    oxts_data first_data_point = GetOxtsData(filelist, empty_timestamps, 0);   
    
    // ...and convert the oxts-data into the Transformation of the first point
    Tr_0 = GetTransformation(first_data_point, scale);
    
    return Tr_0;
}

boost::numeric::ublas::matrix<double> KittiUtils::GetTransformation(oxts_data datapoint, double scale)
{
    // Compare convertOxtsToPose.m from MATLAB DevKit of KITTI Dataset
    boost::numeric::ublas::matrix<double> Tr(4, 4);
    boost::numeric::ublas::matrix<double> Rx(3, 3);
    boost::numeric::ublas::matrix<double> Ry(3, 3);
    boost::numeric::ublas::matrix<double> Rz(3, 3);
    boost::numeric::ublas::matrix<double> R(3, 3);
    
    // converts lat/lon coordinates to mercator coordinates using mercator scale
    // see latlonToMercator.m from MATLAB DevKit of KITTI Dataset
    double er = 6378137;
    double mx = scale * datapoint.lon * boost::math::constants::pi<double>() * er / 180.0f;
    double my = scale * er * std::log( std::tan((90.0f+datapoint.lat) * boost::math::constants::pi<double>() / 360.0f) );
    double mz = datapoint.alt;
    
    // rotation matrix (OXTS RT3000 user manual, page 71/92)
    //
    // base => nav  (level oxts => rotated oxts)
    Rx(0,0) = 1;                            Rx(0,1) = 0;                        Rx(0,2) = 0;
    Rx(1,0) = 0;                            Rx(1,1) = std::cos(datapoint.roll); Rx(1,2) = -std::sin(datapoint.roll);
    Rx(2,0) = 0;                            Rx(2,1) = std::sin(datapoint.roll); Rx(2,2) =  std::cos(datapoint.roll);
    // base => nav  (level oxts => rotated oxts)
    Ry(0,0) =  std::cos(datapoint.pitch);   Ry(0,1) = 0;                        Ry(0,2) = std::sin(datapoint.pitch);
    Ry(1,0) =  0;                           Ry(1,1) = 1;                        Ry(1,2) = 0;
    Ry(2,0) = -std::sin(datapoint.pitch);   Ry(2,1) = 0;                        Ry(2,2) =  std::cos(datapoint.pitch);
    // base => nav  (level oxts => rotated oxts)
    Rz(0,0) = std::cos(datapoint.yaw);      Rz(0,1) = -std::sin(datapoint.yaw); Rz(0,2) = 0;
    Rz(1,0) = std::sin(datapoint.yaw);      Rz(1,1) =  std::cos(datapoint.yaw); Rz(1,2) = 0;
    Rz(2,0) = 0;                            Rz(2,1) =  0;                       Rz(2,2) =  1;
    // R = Rz*Ry*Rx;
    std::cout << "Rx = " << Rx << std::endl;
    std::cout << "Ry = " << Ry << std::endl;
    std::cout << "Rz = " << Rz << std::endl;
    std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    R = boost::numeric::ublas::prod(Ry, Rx); std::cout << "Ry*Rx    = " << R << std::endl;
    R = boost::numeric::ublas::prod(Rz, R);  std::cout << "Rz*Ry*Rx = " << R << std::endl;
    
    // in Matlab notation: Tr = [R t;0 0 0 1];
    // DOES NOT WORK: boost::numeric::ublas::project(Tr, boost::numeric::ublas::range(0,2), boost::numeric::ublas::range(0,2)) = R;
    Tr(0,0) = R(0,0); Tr(0,1) = R(0,1); Tr(0,2) = R(0,2); Tr(0,3) = mx;
    Tr(1,0) = R(1,0); Tr(1,1) = R(1,1); Tr(1,2) = R(1,2); Tr(1,3) = my;
    Tr(2,0) = R(2,0); Tr(2,1) = R(2,1); Tr(2,2) = R(2,2); Tr(2,3) = mz;
    Tr(3,0) = 0;      Tr(3,1) = 0;      Tr(3,2) = 0;      Tr(3,3) = 1;
    
    return Tr;
}



boost::numeric::ublas::matrix<double> KittiUtils::ConvertOxtsToPose(oxts_data datapoint, double scale, boost::numeric::ublas::matrix<double> Tr_0_inv)
{
    boost::numeric::ublas::matrix<double> pose(4, 4);
    boost::numeric::ublas::matrix<double> Tr(4, 4);
    
    // in Matlab notation: pose = inv(Tr_0) * [R t;0 0 0 1]
    // with Tr = [R t;0 0 0 1]
    Tr = GetTransformation(datapoint, scale);
    pose = boost::numeric::ublas::prod(Tr_0_inv, Tr);
 
    return pose;
}

void KittiUtils::GetEulerAngles(boost::numeric::ublas::matrix<double> pose, double &roll, double &pitch, double &yaw)
{
    // Normalize
    pose /= pose(3,3);
    Eigen::Matrix3d rot_matrix;
    rot_matrix(0,0) = pose(0,0); rot_matrix(0,1) = pose(0,1); rot_matrix(0,2) = pose(0,2);
    rot_matrix(1,0) = pose(1,0); rot_matrix(1,1) = pose(1,1); rot_matrix(1,2) = pose(1,2);
    rot_matrix(2,0) = pose(2,0); rot_matrix(2,1) = pose(2,1); rot_matrix(2,2) = pose(2,2);
    Eigen::Vector3d rot = rot_matrix.eulerAngles(0, 1, 2); // order: roll - pitch - yaw
    roll  = rot(0);
    pitch = rot(1);
    yaw   = rot(2);
}

void KittiUtils::GetPosition(boost::numeric::ublas::matrix<double> pose, double &x, double &y, double &z)
{
    // Normalize
    pose /= pose(3,3);
    x     = pose(0,3);
    y     = pose(1,3);
    z     = pose(2,3);
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

