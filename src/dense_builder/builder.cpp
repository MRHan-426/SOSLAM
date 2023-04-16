#include "builder.h"
//#include "include/utils/matrix_utils.h"

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include <memory>   // for make_shared

#include <iostream>

// //PCL
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>


using namespace Eigen;
using namespace std;

void Builder::processFrame(cv::Mat &rgb, cv::Mat &depth, gtsam::Pose3 &pose, double depth_thresh){
    if(!mbInitialized) {
        std::cerr << "Please initialize the builder first." << std::endl;
        return;
    }
    PointCloudPCL::Ptr pLocalPointCloud = image2PointCloud(rgb, depth, depth_thresh);

    PointCloudPCL::Ptr pGlobalPointCloud = transformToWolrd(pLocalPointCloud, pose);   // pose: Twc
    mpCurrentMap = pGlobalPointCloud;

    // add to map
    addPointCloudToMap(pGlobalPointCloud);

    return;
}

void Builder::setCameraIntrinsic(Matrix3d &calibMat, double scale){
    mmCalib = calibMat;
    mdScale = scale;

    mbInitialized = true;
}

Builder::Builder():mbInitialized(false), mpPclMap(new PointCloudPCL){

}

PointCloudPCL::Ptr Builder::image2PointCloud( cv::Mat& rgb, cv::Mat& depth, double depth_thresh)
{
    PointCloudPCL::Ptr temp_cloud ( new PointCloudPCL );
    
    Camera camera;
    getCameraParam(mmCalib, camera);
    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            ushort * ptd = depth.ptr<ushort>(m);
            ushort d = ptd[n];
            if (d == 0)
                continue;

            PointT p;
            p.z = double(d) / mdScale;

            if( p.z < 0.5 || p.z > depth_thresh )
                continue;   

            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;

            //  B G R
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            temp_cloud->points.push_back( p );
        }

    temp_cloud->height = 1;
    temp_cloud->width = temp_cloud->points.size();
    temp_cloud->is_dense = true;
    return temp_cloud;
}

void Builder::getCameraParam(Matrix3d &calib, Camera &cam){
    // kalib matrix:
    // fx s cx
    // 0 fy cy
    // 0 0 1
    cam.fx = calib(0,0);
    cam.fy = calib(1,1);
    cam.cx = calib(0,2);
    cam.cy = calib(1,2);
}

PointCloudPCL::Ptr Builder::getMap()
{
    return mpPclMap;
}

PointCloudPCL::Ptr Builder::getCurrentMap()
{
    return mpCurrentMap;
}

PointCloudPCL::Ptr Builder::transformToWolrd( PointCloudPCL::Ptr& pPointCloud, gtsam::Pose3 &pose){
    // pose: Tcw 
    // tx ty tz (3 floats) give the position of 
    // the optical center of the color camera 
    // with respect to the world origin as defined by the motion capture system.
    PointCloudPCL::Ptr transformed_cloud(new PointCloudPCL);
    Matrix4d transform = pose.matrix();
//    std::cout << "transform " << transform << std::endl;
    pcl::transformPointCloud (*pPointCloud, *transformed_cloud, transform);
    
    return transformed_cloud;
}

void Builder::addPointCloudToMap(PointCloudPCL::Ptr pPointCloud){
    *mpPclMap = *mpPclMap + *pPointCloud;
}

void Builder::saveMap(const string& path){
  if(mpPclMap->points.size() < 1) return;
  pcl::io::savePCDFileASCII(path, *mpPclMap);
  std::cerr << "Saved " << mpPclMap->points.size () << " data points to " << path << std::endl;
}

void Builder::voxelFilter(double grid_size){
    // Voxel grid downsample
    static pcl::VoxelGrid<PointT> voxel;

    double gridsize = grid_size;
    voxel.setLeafSize( gridsize, gridsize, gridsize );
    voxel.setInputCloud( mpPclMap );
    PointCloudPCL::Ptr tmp( new PointCloudPCL() );
    voxel.filter( *tmp );
//    pcl::PointCloud<PointT> &cloud_in = *tmp;
//    pcl::PointCloud<PointT> &cloud_out = *mpPclMap;
//    // Allocate enough space and copy the basics
//    cloud_out.header   = cloud_in.header;
//    cloud_out.width    = cloud_in.width;
//    cloud_out.height   = cloud_in.height;
//    cloud_out.is_dense = cloud_in.is_dense;
//    cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
//    cloud_out.sensor_origin_ = cloud_in.sensor_origin_;
//    cloud_out.points.resize (cloud_in.points.size ());
//
//    if (cloud_in.points.empty ())
//        return;
//
////    if (isSamePointType<PointT, PointT> ())
//        // Copy the whole memory block
//        memcpy (&cloud_out.points[0], &cloud_in.points[0], cloud_in.points.size () * sizeof (PointT));
////    else
////        // Iterate over each point
////        for (std::size_t i = 0; i < cloud_in.points.size (); ++i)
////            copyPoint (cloud_in.points[i], cloud_out.points[i]);
    pcl::copyPointCloud(*tmp, *mpPclMap);
}