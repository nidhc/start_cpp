#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <librealsense2/rs.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/common/centroid.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/ppf.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/common/time.h>
#include <pcl/console/time.h>
#include <pcl/console/parse.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/surface/mls.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/ppf_registration.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/registration/icp.h>

#include <opencv2/opencv.hpp>

#include "MechEyeApi.h"
#include "SampleUtil.h"

#include "ini.h"
#include "print_rci.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "robot.h"
#include "move.h"
#include "control_tools.h"

using namespace std;
using namespace pcl;
using namespace cv;

const double camera_cx = 325.2611;
const double camera_cy = 242.04899;
const double camera_fx = 572.4114;
const double camera_fy = 573.57043;
//using namespace xmate;
//using CartesianControl = std::function<CartesianPose(RCI::robot::RobotState robot_state)>;
//
//std::string ipaddr = "192.168.0.160";
//uint16_t port = 1337;
//xmate::Robot robot(ipaddr, port,XmateType::XMATE7_PRO);

/**
 * \brief 将mech点云转换为pcl点云
 * @param 转换对象pcl点云
 * @param 被转换对象mech点云
 */
void toPCL(pcl::PointCloud<pcl::PointXYZ> &pointCloud,
           const mmind::api::PointXYZMap &pointXYZMap) {
    uint32_t size = pointXYZMap.height() * pointXYZMap.width();
    pointCloud.resize(size);

    for (size_t i = 0; i < size; i++) {
        pointCloud[i].x = 0.001 * pointXYZMap[i].x; // mm to m
        pointCloud[i].y = 0.001 * pointXYZMap[i].y; // mm to m
        pointCloud[i].z = 0.001 * pointXYZMap[i].z; // mm to m
    }

    return;
}

int main(int argc, char **argv) {
//    388, 164, 34, 45
    Mat depth = imread("../000079.png", IMREAD_ANYDEPTH);
    Mat mask = imread("../000079_000000.png");
    Mat rgb = imread("../000079rgb.png");
    vector<Mat> mv;//可存放Mat类型的容器
    split(mask, mv);//将多通道 拆分成 单通道（通道分离
    Mat img, rgb_seg;
    depth.copyTo(img, mv[0]);
    rgb.copyTo(rgb_seg, mv[0]);
    imshow("rgb_seg", rgb_seg);
    waitKey();
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    for (int m = 0; m < img.rows; m++)
        for (int n = 0; n < img.cols; n++) {
            // 获取深度图中(m,n)处的值
            unsigned short d = img.at<unsigned short>(m, n);
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointXYZ p;

            // 计算这个点的空间坐标
            p.z = double(d) / 1000;
            p.x = (n - camera_cx) * p.z / camera_fx;
            p.y = (m - camera_cy) * p.z / camera_fy;
            // 把p加入到点云中
            cloud->points.push_back(p);
        }
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout << "point cloud size = " << cloud->points.size() << endl;
    cloud->is_dense = false;
    pcl::io::savePCDFile("../000079.pcd", *cloud);
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()) {
    }

}