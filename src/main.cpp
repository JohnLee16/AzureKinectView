// main.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include "DataAcquirement.h"
#include "PointCloudViewer.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ascii_io.h>

int main()
{    
    //pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("main demo", true));
    
    
    /*viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ> rgb(point_cloud_ptr);
    viewer->addPointCloud<pcl::PointXYZ>(point_cloud_ptr, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();*/
    
   
    
    
    /*while (viewer -> wasStopped())
    {
        std::this_thread::sleep_for(100ms);
    }*/
    //PointCloudViewer viewer;
    //viewer.ViewPointCloud();
    DataAcquirement dataAcquirement;
    dataAcquirement.GetK4aRecord();
    
    std::cout << "Hello World!\n";
}
