#include "debug.h"

void Debug::DebugPCL::visualization(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::visualization::PCLVisualizer viewer("cloud");
    viewer.setBackgroundColor(0, 0, 0);         //初始化黑色背景

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> fildColor(cloud);
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud, fildColor, "sample");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample"); // 设置点云大小
    viewer.addCoordinateSystem(0.5); // 添加坐标系，并设置比例
    viewer.initCameraParameters(); // 通过设置照相机参数使得从默认的角度和方向观察点云

    //法向计算
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//建立kdtree来进行近邻点集搜索
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	//为kdtree添加点云数据
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	//点云法向计算时，需要搜索的近邻点大小
	n.setKSearch(50);
	//开始进行法向计算
	n.compute(*normals);
	//* normals should not contain the point normals + surface curvatures
    //添加需要显示的点云法向。cloud为原始点云模型，normal为法向信息，1表示需要显示法向的点云间隔，即每1个点显示一次法向，0.01表示法向长度。
	viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals, 100, 0.1, "normals");

    while (!viewer.wasStopped()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5)); //休眠5ms
        viewer.spinOnce();
    }
}

bool height_detection(pcl::PointXYZRGB point){
        if(point.y < 1.5 - 1.0 || point.y > 1.5 + 1.0){
            return false;
        }else{
            return true;
        }
    }
void Debug::DebugPCL::visualization2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane, Floor::FloorCoeffs coeffs)
{
    pcl::visualization::PCLVisualizer viewer("cloud");

    int v1(0); // 创建新的视角
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1); // 4个参数分别是X轴的最小值，最大值，Y轴的最小值，最大值，取值0-1，v1是标识
    viewer.setBackgroundColor(0, 0, 0, v1); // 设置视口的背景颜色
    viewer.addText("cloud", 10, 10, "v1 text", v1); // 添加文本

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> raw_cloud(cloud);
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud, raw_cloud, "raw_cloud" ,v1);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "raw_cloud"); // 设置点云大小
    viewer.addCoordinateSystem(0.5); // 添加坐标系，并设置比例
    viewer.initCameraParameters(); // 通过设置照相机参数使得从默认的角度和方向观察点云
    // 设置视角的位置和方向
    viewer.setCameraPosition(0, 0, 0,      // 视点位置为原点
                            0, 0, 1,      // 视线方向为正Z轴方向
                            0, -1, 0);    // 上方向为负Y轴方向

    pcl::ModelCoefficients p_coeffs;
    p_coeffs.values.push_back(coeffs.coeffs[0]);
    p_coeffs.values.push_back(coeffs.coeffs[1]);
    p_coeffs.values.push_back(coeffs.coeffs[2]);
    p_coeffs.values.push_back(coeffs.coeffs[3]);
    viewer.addPlane(p_coeffs, -1, -10, 0,"plane");

    int v2(0); // 创建新的视角
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);; // 4个参数分别是X轴的最小值，最大值，Y轴的最小值，最大值，取值0-1，v1是标识
    viewer.setBackgroundColor(0, 0, 0, v2); // 设置视口的背景颜色
    viewer.addText("cloud_plane", 10, 10, "v2 text", v2); // 添加文本

    // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> plane_cloud(cloud_plane);
    // viewer.addPointCloud<pcl::PointXYZRGB>(cloud_plane, plane_cloud, "cloud_plane",v2);
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_plane"); // 设置点云大小

    // viewer.addCoordinateSystem(0.5); // 添加坐标系，并设置比例
    // viewer.initCameraParameters(); // 通过设置照相机参数使得从默认的角度和方向观察点云

    //法向计算
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//建立kdtree来进行近邻点集搜索
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	//为kdtree添加点云数据
	tree->setInputCloud(cloud_plane);
	n.setInputCloud(cloud_plane);
	n.setSearchMethod(tree);
	//点云法向计算时，需要搜索的近邻点大小
	n.setKSearch(50);
    n.setViewPoint(0.0f, 1.5f, 0.0f);
	//开始进行法向计算
	n.compute(*normals);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    // filtered->reserve(cloud_plane->size());

    pcl::PointCloud<pcl::Normal>::Ptr n_plane(new pcl::PointCloud<pcl::Normal>);
    for(size_t i = 0; i < cloud_plane->size(); i++) {
        //hwh 计算点云数据中每个点的法向量与Z轴的夹角（余弦值） 两个单位向量点乘得到cos值
        // float dot = normals->at(i).getNormalVector3fMap().normalized().dot(Eigen::Vector3f::UnitY());
        Eigen::Vector3f vec = normals->at(i).getNormalVector3fMap().normalized();
        //hwh cos在0-90为减函数所以这里是 > ; 下面表示，将点云法向量在地面向量(0,0,1) 20°范围内的点云保存起来
        pcl::PointXYZRGB point = cloud_plane->points[i];
        // if(std::abs(dot) > std::cos(20 * M_PI / 180.0) || height_detection(point)) {
        if(std::abs(vec[1]) > 0.8 && height_detection(point)) {
            // LOG(INFO) << "x = " << vec[0] << " y = " << vec[1] << " z = " << vec[2];
            filtered->push_back(cloud_plane->at(i));
            n_plane->push_back(normals->at(i));
        }
    }
    if(filtered->size() == 0){
        LOG(WARNING) << "no suitable plane point";
        viewer.spinOnce();
        viewer.close();
        return ;
    }
	//* normals should not contain the point normals + surface curvatures
    //添加需要显示的点云法向。cloud为原始点云模型，normal为法向信息，1表示需要显示法向的点云间隔，即每1个点显示一次法向，0.1表示法向长度。
	// viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud_plane, normals, 100, 0.1, "normals" , v2);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> plane_cloud(filtered);
    viewer.addPointCloud<pcl::PointXYZRGB>(filtered, plane_cloud, "cloud_plane",v2);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_plane"); // 设置点云大小

    viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(filtered, n_plane, 100, 0.1, "normals" , v2);
    viewer.addPlane(p_coeffs, -1, -10, 0,"plane2");
    while (!viewer.wasStopped()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5)); //休眠5ms
        viewer.spinOnce();
    }
}