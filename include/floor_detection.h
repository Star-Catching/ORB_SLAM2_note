#ifndef _FLOOR_DETECTION_
#define _FLOOR_DETECTION_

#include <mutex>
#include <memory>
#include <iostream>
#include <string>
#include <boost/optional.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include "glog/logging.h"     //头文件

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
using namespace std;

namespace Floor{

struct FloorCoeffs {
  uint64_t timestamp;
  std::vector<float> coeffs;
};

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;

class FloorDetection{
public:
    typedef pcl::PointXYZRGB PointT;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FloorDetection(std::string path, bool _enable): enable(_enable){
        initialize_params(path);
    }
    ~FloorDetection() {}

    void initialize_params(std::string path);
    FloorCoeffs floor_detection_cbk(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    void runFloor(cv::Mat rgb_image, cv::Mat depth_image);
    boost::optional<Eigen::Vector4f> detect(const pcl::PointCloud<PointT>::Ptr& cloud) const {
        // filtering before RANSAC (height and normal filtering)
        pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
        filtered = cloud;
        //hwh 提取在[sensor_height - height_clip_range_low + height_clip_range_high]区域内的点云
        //?hwh 下面height_clip_range_low 和 height_clip_range_high可能写反了
        // filtered = plane_clip(filtered, Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height + height_clip_range_high), false);
        // filtered = plane_clip(filtered, Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height - height_clip_range_low), true);

        //hwh 将“非”垂直法线的点将在RANSAC之前进行过滤
        if(use_normal_filtering) {
            filtered = normal_filtering(filtered);
        }

        // too few points for RANSAC
        if(filtered->size() < floor_pts_thresh) {
            //hwh boost::none是boost::optional<>库中的一个特殊值，表示一个空值
            LOG(INFO) << "too few points for RANSAC " << filtered->size();
            return boost::none;
        }else{
            LOG(INFO) << "filtered->size() =  " << filtered->size();
        }

        // RANSAC
        //hwh SampleConsensusModelPlane是用于拟合平面模型的采样一致性模型,可以用于从点云数据中提取平面模型
        pcl::SampleConsensusModelPlane<PointT>::Ptr model_p(new pcl::SampleConsensusModelPlane<PointT>(filtered));
        //hwh RandomSampleConsensus从点云数据中估计出一个模型的参数(平面模型，球体模型，圆柱体模型等)，并去除数据点集中的离群点的采样一致性算法类
        pcl::RandomSampleConsensus<PointT> ransac(model_p);
        ransac.setDistanceThreshold(0.1);
        ransac.computeModel();

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        ransac.getInliers(inliers->indices);

        // too few inliers
        if(inliers->indices.size() < floor_pts_thresh) {
            LOG_EVERY_T(INFO, 0.5) << "too few inliers " << inliers->indices.size();
            return boost::none;
        }

        // verticality check of the detected floor's normal
        // Eigen::Vector4f reference = Eigen::Vector4f::UnitY();

        Eigen::VectorXf coeffs;
        ransac.getModelCoefficients(coeffs);

        ////0TODO 是否有必要加这个约束 (值得注意的是，TUM数据集坐标系为右下前坐标系，即高度正方向为Y，向下)
        //hwh 没必要加了，因为运行时并不是相对地面水平运行
        ////0TODO 判断Y的绝对值， 大于0.8
        Eigen::Vector4f vec = coeffs.head<4>();
        if(std::abs(vec[1]) < 0.8 || std::abs(vec[3]) < 1){
            if(std::abs(vec[1]) > 0.8){
                LOG(WARNING) << "vec y is error y = " << vec[1];
            }else{
                LOG(WARNING) << "z is error z = " << vec[3];
            }
            return boost::none;
        }
        // //hwh coeffs.head<3>()表示获取向量的前3个值
        // double dot = coeffs.head<3>().dot(reference.head<3>());
        // //hwh 验证计算后的平面向量也得符合地面向量范围约束
        // if(std::abs(dot) < std::cos(floor_normal_thresh * M_PI / 180.0)) {
        //     // the normal is not vertical
        //     LOG(INFO) << "---------------the normal is not vertical error";
        //     return boost::none;
        // }

        // make the normal upward
        //hwh 因为地面向量也可能是竖直向下，所以这里作者将向量都统一到方向向上
        if(coeffs.head<3>().dot(Eigen::Vector3f::UnitY()) < 0.0f) {
            coeffs *= -1.0f;
        }

        return Eigen::Vector4f(coeffs);
    }

    pcl::PointCloud<PointT>::Ptr plane_clip(const pcl::PointCloud<PointT>::Ptr& src_cloud, const Eigen::Vector4f& plane, bool negative) const {
        //hwh PlaneClipper3D为PCL库中用于三维平面裁剪的库，输入参数为平面方程即 方向向量和距离值
        pcl::PlaneClipper3D<PointT> clipper(plane);
        pcl::PointIndices::Ptr indices(new pcl::PointIndices);
        //hwh 裁剪
        clipper.clipPointCloud3D(*src_cloud, indices->indices);

        pcl::PointCloud<PointT>::Ptr dst_cloud(new pcl::PointCloud<PointT>);

        //hwh ExtractIndices用于从点云数据中提取指定索引处的点云数据,pcl::Indices类用于存储需要提取的点的索引
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(src_cloud);
        extract.setIndices(indices);
        //hwh negative为true时，表示提取索引之外的点； negative为false时，表示提取索引内的点
        extract.setNegative(negative);
        extract.filter(*dst_cloud);

        return dst_cloud;
    }

    bool height_detection(pcl::PointXYZRGB point) const{
        if(point.y < 1.5 - 1.0 || point.y > 1.5 + 1.0){
            return false;
        }else{
            return true;
        }
    }
    pcl::PointCloud<PointT>::Ptr normal_filtering(const pcl::PointCloud<PointT>::Ptr& cloud) const {
        //hwh NormalEstimation 用于计算点云数据中每个点的法向量
        //hwh                  点计算曲率的方式：基于最近邻搜索的算法，可以通过估计每个点的曲面来计算法向量
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        ne.setInputCloud(cloud);

        //hwh 设置最近邻搜索方法为 kdtree
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        ne.setSearchMethod(tree);

        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        //hwh 设置法向量计算时使用的最近邻点的数量
        //hwh 默认使用setRadiusSearch方法设置的搜索半径来进行最近邻搜索。
        ne.setKSearch(10);
        //hwh 指定法向量计算时使用的视点位置 如果知道点云数据是从某个特定视点获取的，可以使用setViewPoint来指定该视点的位置，以获得更准确的法向量计算结果
        ne.setViewPoint(0.0f, sensor_height, 0.0f);
        //hwh 计算点云数据中每个点的法向量
        ne.compute(*normals);

        pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
        filtered->reserve(cloud->size());

        //TODO 判断Y的绝对值， 大于0.8
        for(size_t i = 0; i < cloud->size(); i++) {
            //hwh 计算点云数据中每个点的法向量与Z轴的夹角（余弦值） 两个单位向量点乘得到cos值
            // float dot = normals->at(i).getNormalVector3fMap().normalized().dot(Eigen::Vector3f::UnitY());
            Eigen::Vector3f vec = normals->at(i).getNormalVector3fMap().normalized();
            //hwh cos在0-90为减函数所以这里是 > ; 下面表示，将点云法向量在地面向量(0,0,1) 20°范围内的点云保存起来
            pcl::PointXYZRGB point = cloud->points[i];
            // if(std::abs(dot) > std::cos(normal_filter_thresh * M_PI / 180.0) || height_detection(point)) {
            if(std::abs(vec[1]) > 0.8 && height_detection(point)) {
                filtered->push_back(cloud->at(i));
            }
        }
        filtered->width = filtered->size();
        filtered->height = 1;
        filtered->is_dense = false;

        return filtered;
    }

    std::mutex floor_coeffs_queue_mutex;
    std::deque<FloorCoeffs> floor_coeffs_queue;
private:
    // floor detection parameters
    // see initialize_params() for the details
    double sensor_height;
    double height_clip_range_high;
    double height_clip_range_low;

    uint16_t floor_pts_thresh;
    double floor_normal_thresh;

    bool use_normal_filtering;
    double normal_filter_thresh;
    float fx;
    float fy;
    float cx;
    float cy;
    float depthFactor;
    bool enable;
};
    
}
#endif