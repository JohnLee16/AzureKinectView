#pragma once
#include <iostream>
#include <k4a/k4a.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;


class PointCloudViewer
{
public:
    PointCloudViewer();
    ~PointCloudViewer();
    void ViewPointCloud();

    cv::Mat create_mat_from_buffer(uint16_t* data, int width, int height, int channels);


};

