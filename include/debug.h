#ifndef __DEBUG__H__
#define __DEBUG__H__

//glog
#include "glog/logging.h"  

//pcl
#include <thread>
//#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

//time
#include <ctime>
#include <cstdlib>
#include <chrono>
//floor
#include "floor_detection.h"

namespace Debug{

class DebugLog
{
private:
    /* data */
public:
    DebugLog(){
        log_init();
    }
    ~DebugLog(){
        google::ShutdownGoogleLogging();
    }
    void log_init()
    {
        google::InitGoogleLogging("DebugLog");

        FLAGS_alsologtostderr  = true;      // 设置日志消息除了日志文件之外是否去标准输出
        FLAGS_colorlogtostderr = true;      // 设置记录到标准输出的颜色消息（如果终端支持）
        FLAGS_log_prefix       = true;      // 设置日志前缀是否应该添加到每行输出
        FLAGS_logbufsecs       = 0;         // 设置可以缓冲日志的最大秒数，0指实时输出
        FLAGS_logtostderr      = true;      // 设置日志消息为标准输出
        LOG(INFO) << "LOG init finish";
        google::InstallFailureSignalHandler();      //启用段错误检测
    }
};

class DebugPCL
{
public:
    DebugPCL(){};
    ~DebugPCL(){};
    void visualization(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void visualization2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane, Floor::FloorCoeffs coeffs);
};

class TicToc
{
  public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

}

#endif