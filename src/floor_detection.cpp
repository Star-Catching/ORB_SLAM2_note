#include "floor_detection.h"

//debug
#include "debug.h"
//glog
#include "glog/logging.h"  


void Floor::FloorDetection::initialize_params(std::string path)
{
    sensor_height = 1.5;          // approximate sensor height [m]
    height_clip_range_low  = 0.5;
    height_clip_range_high = 0.5; // points with heights in [sensor_height - height_clip_range_low, sensor_height + height_clip_range_high] will be used for floor detection
    floor_pts_thresh = 1024;       // minimum number of support points of RANSAC to accept a detected floor plane
    floor_normal_thresh = 10.0;   // verticality check thresold for the detected floor plane [deg]
    use_normal_filtering = true;  // if true, points with "non-"vertical normals will be filtered before RANSAC
    normal_filter_thresh = 20.0;  // "non-"verticality check threshold [deg]

    cv::FileStorage fSettings(path, cv::FileStorage::READ);
    fx = fSettings["Camera.fx"];
    fy = fSettings["Camera.fy"];
    cx = fSettings["Camera.cx"];
    cy = fSettings["Camera.cy"];
    depthFactor = fSettings["DepthMapFactor"];
    LOG(INFO) << "floor detection param loading finish";
    LOG(INFO) << "\nfx = " << fx << "\nfy = " << fy << "\ncx = " << cx << "\ncy = " << cy << "\ndepthFactor = " << depthFactor;
}

Floor::FloorCoeffs Floor::FloorDetection::floor_detection_cbk(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    FloorCoeffs coeffs;
    coeffs.coeffs = {0};
    
    if(cloud->empty()) {
      return coeffs;
    }
    // floor detection
    //hwh 提取位于地面区间的点云数据，并按照点云法向量进行剔除，在拟合出平面向量
    boost::optional<Eigen::Vector4f> detect_floor = detect(cloud);
    
    // publish the detected floor coefficients
    //// 0TODO 
    coeffs.timestamp = cloud->header.stamp;
    //coeffs.timestamp = 0;
    if(detect_floor) {
        coeffs.coeffs.resize(4);
        for(int i = 0; i < 4; i++) {
            coeffs.coeffs[i] = (*detect_floor)[i];
        }
        std::lock_guard<std::mutex> lock(floor_coeffs_queue_mutex);
        floor_coeffs_queue.push_back(coeffs);
        LOG(INFO) << "find floor " << coeffs.coeffs[0] << " " << coeffs.coeffs[1] << " " << coeffs.coeffs[2] << " " << coeffs.coeffs[3] << " timestamp = " << coeffs.timestamp;
    }
    return coeffs;
}

void Floor::FloorDetection::runFloor(cv::Mat rgb_image, cv::Mat depth_image)
{
    if(enable != true){
        LOG_FIRST_N(INFO, 1) << "Floor function is disable";
        return;
    }
    // 将深度图像转换为PCL格式
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
    // 从RGB和深度图像中提取2D平面点并保存为PCL 3D点
    for (int y = 0; y < depth_image.rows; y++) {
        for (int x = 0; x < depth_image.cols; x++) {
            pcl::PointXYZRGB point;

            point.z = depth_image.at<uint16_t>(y, x) / depthFactor;    
            point.x = (x - cx) * point.z / fx;
            point.y = (y - cy) * point.z / fy;
            point.r = rgb_image.at<cv::Vec3b>(y, x)[2];
            point.g = rgb_image.at<cv::Vec3b>(y, x)[1];
            point.b = rgb_image.at<cv::Vec3b>(y, x)[0];
            if(point.z != 0){
                cloud->points.push_back(point);
                // LOG(INFO) << "point: x = " << point.x << " y = " << point.y << " z = " << point.z ; 
                // LOG(INFO) << "depth = " << depth_image.at<uint16_t>(y, x); 
            }
            if(point.y > -2 && point.y != 0){
                // LOG(INFO) << "point: x = " << point.x << " y = " << point.y << " z = " << point.z ; 
                cloud_plane->points.push_back(point);
            }
        }
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_filter(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud_plane);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);//设置滤波时创建的体素体积为1 cm3的立方体
    sor.filter(*cloud_plane_filter);
    LOG(INFO) << "cloud_plane_filter size = " << cloud_plane_filter->points.size();

    //floor
    Debug::TicToc tic;
    Floor::FloorCoeffs coeffs = floor_detection_cbk(cloud_plane_filter);
    LOG(INFO) << "Floor detection cost " << tic.toc() << " ms";

    // 保存点云到PCL文件
    // pcl::io::savePCDFileBinary("output_cloud.pcd", *cloud);
    // if(cloud_plane_filter->points.size() == 0){
    //     LOG(WARNING) << "cloud_plane_filter has no points";
    //     return;
    //     // continue;
    // }
    // pcl::io::savePCDFileBinary("output_cloud_plane.pcd", *cloud_plane_filter);

    // 可视化点云
    // Debug::DebugPCL debug_pcl;
    // debug_pcl.visualization2(cloud,cloud_plane_filter,coeffs);
}
