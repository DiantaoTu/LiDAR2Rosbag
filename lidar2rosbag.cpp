#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cmath>

#include<algorithm>
#include<chrono>
#include<sstream>
#include<string>
 
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosbag/bag.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace std;


/**
 * @description: 遍历指定目录下的某一类
 * @param pathName {string} 目录名称
 * @param &fileNames {vector<string>} 文件名，这里包含的是绝对路径
 * @param fileType {string} 遍历的文件的后缀
 * @return {*}
 */
void IterateFiles(string pathName, vector<string> &fileNames, string fileType);

// int 数字转字符串
string int2str(int num);

// 字符串分割，这个分割是把 xyz 变成 x y z ， 把 -x-zy 变成 -x -z y , 不是常规意义的分割字符串
vector<string> SplitString(const string& str);

// 得到新的坐标系下 XYZ三个轴对应于老的坐标系下三个轴的顺序，举个例子，用 2代表新的坐标系，1代表老的坐标系，那么某种变换是
// X2 = -Y1      Y2 = -Z1       Z2 = X1
// 这种情况下输出的顺序就是 {-2, -3, 1}, 这是因为输出的是新的XYZ对应的老XYZ的顺序，新的X对应老的-Y，而-Y就是用-2表示，
// 因此第一个数字是-2，新的Y对应老的-Z, 而-Z用-3表示，因此第二个数字是-3
// XYZ和数字之间的对应关系写在了 str_to_num 里
vector<int> NewXYZfromOldXYZ(const vector<string>& old_order, const vector<string>& new_order);

// 根据坐标系的顺序得到变换的顺序矩阵， input_order 是输入的点云的坐标顺序，output order是输出的点云的坐标顺序
// 这里的坐标顺序是 右-前-上 的顺序，比如输入点云是 x向右 y向前 z向上，那么input order就是 "xyz"，
// 输出的点云是 x向前 y向左 z向上，那么output order就是 "-yxz", 这个变换矩阵就是把点云从 "xyz" 变成 "-yxz"
Eigen::Matrix4d SetTransformation(string input_order, string output_order);


ros::Publisher pubLaserCloud;

// xyz三个轴的顺序和数字之间的对应关系
map<string, int> str_to_num = {{"x", 1}, {"-x", -1}, {"y", 2}, {"-y", -2}, {"z", 3}, {"-z", -3}};


int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar2rosbag");
    ros::NodeHandle nh;

    if(argc < 3)
    {
        printf("ERROR: Please follow the example: rosrun pkg node input num_output:\n  rosrun lidar2rosbag lidar2rosbag /data/KITTI/dataset/sequences/04/ 04 \n");
        return -2;
    }

    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_pub", 2);

    std::string input_path = argv[1];
    std::string output_file = argv[2];
    // 点云坐标顺序
    std::string input_order, output_order;

    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();

    if(argc == 5)
    {
        input_order = argv[3];
        output_order = argv[4];
        transformation = SetTransformation(input_order, output_order);
        cout << "Transformation is : " << endl << transformation << endl;
    }


    vector<string> pcd_names;
    IterateFiles(input_path, pcd_names, ".pcd");
    if(pcd_names.size() == 0)
        cout << "no pcd in " << input_path << endl;

    rosbag::Bag bag;
    bag.open(output_file + ".bag", rosbag::bagmode::Write);

    ros::Time base_time = ros::Time::now();

    for(int i = 0; i < pcd_names.size(); i ++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr lidarPoints(new pcl::PointCloud<pcl::PointXYZI>);
        if(pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_names[i], *lidarPoints) == -1)
        {
            std::cout << "Fail to load point at " << pcd_names[i] << std::endl;
            return -1;
        }
        pcl::transformPointCloud(*lidarPoints, *lidarPoints, transformation);

        ros::Time timestamp_ros = base_time + ros::Duration(0.1 * i);      // 每秒10帧，所以每次增加0.1秒
        double time = timestamp_ros.toSec();
        lidarPoints->header.stamp = time ;
        lidarPoints->header.frame_id = "velodyne";

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*lidarPoints, output);
        output.header.stamp = timestamp_ros  ;
        output.header.frame_id = "velodyne";
        
        pubLaserCloud.publish(output);

        bag.write("/velodyne_points", timestamp_ros, output);
        // std::cout<<"ros time : "<< output.header.stamp.toSec() <<"  with  "<<timestamp_ros.toSec() << std::endl;
        cout << i  << " / " << pcd_names.size() << endl;

    }
    bag.close();

    std::cout << "lidar to rosbag done" << std::endl;
    
    return 0;
}


void IterateFiles(string pathName, vector<string> &fileNames, string fileType)
{
    fileNames.clear();
    boost::filesystem::directory_iterator endIter;
    for (boost::filesystem::directory_iterator iter(pathName); iter != endIter; ++iter)
    {
        if (boost::filesystem::is_regular_file(iter->status()))
        {
            std::string type = iter->path().extension().string();
            transform(type.begin(), type.end(), type.begin(), (int (*)(int))tolower);
            if (type == fileType)
                fileNames.push_back(iter->path().string());
        }
        else if (boost::filesystem::is_directory(iter->path()))
        {
            cout << iter->path().string() << endl;
            vector<string> names;
            IterateFiles(iter->path().string(), names, fileType);
            fileNames.insert(fileNames.end(), names.begin(), names.end());
        }
    }
    sort(fileNames.begin(), fileNames.end(), std::less<std::string>());
}

// convert a int to a string
string int2str(int num) {
    ostringstream oss;
    if (oss << num) {
        string str(oss.str());
        return str;
    }
    else {
        cout << "[float2str] - Code error" << endl;
        exit(0);
    }
}

vector<string> SplitString(const string& str)
{
    vector<string> split;
    int start = 0, end = 0;
    for(int end = 0; end < str.size(); end++)
    {
        if(str[end] == 'x' || str[end] == 'y' || str[end] == 'z')
        {
            split.push_back(str.substr(start, end - start + 1));
            start = end + 1;
        }
    }
    return split;
}

vector<int> NewXYZfromOldXYZ(const vector<string>& old_order, const vector<string>& new_order)
{
    vector<int> new_xyz;
    size_t idx = 0;
    // 先找到x对应
    for(idx = 0; idx < new_order.size(); idx++)
    {
        if(new_order[idx] == "x")
        {
            new_xyz.push_back(str_to_num[old_order[idx]]);
            break;
        }
        else if(new_order[idx] == "-x")
        {
            new_xyz.push_back(-str_to_num[old_order[idx]]);
        }      
    }
    // 再找到y的对应
    for(idx = 0; idx < new_order.size(); idx++)
    {
        if(new_order[idx] == "y")
        {
            new_xyz.push_back(str_to_num[old_order[idx]]);
            break;
        }
        else if(new_order[idx] == "-y")
        {
            new_xyz.push_back(-str_to_num[old_order[idx]]);
        }      
    }
    // 最后找到z对应
    for(idx = 0; idx < new_order.size(); idx++)
    {
        if(new_order[idx] == "z")
        {
            new_xyz.push_back(str_to_num[old_order[idx]]);
            break;
        }
        else if(new_order[idx] == "-z")
        {
            new_xyz.push_back(-str_to_num[old_order[idx]]);
        }      
    }
    return new_xyz;
}

Eigen::Matrix4d SetTransformation(string input_order, string output_order)
{
    vector<string> order_old = SplitString(input_order);
    vector<string> order_new = SplitString(output_order);
    vector<int> new_xyz = NewXYZfromOldXYZ(order_old, order_new);
    Eigen::Matrix3d rot = Eigen::Matrix3d::Zero();
    for(int row_idx = 0; row_idx < rot.rows(); row_idx++)
    {
        rot(row_idx, abs(new_xyz[row_idx]) - 1) = (new_xyz[row_idx] > 0 ? 1 : -1);
    }
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = rot;
    return T;
}
