#include "PointCloudViewer.h"
#include <iomanip>

PointCloudViewer::PointCloudViewer()
{
}

PointCloudViewer::~PointCloudViewer()
{
}

void PointCloudViewer::ViewPointCloud()
{
	k4a_device_t device = nullptr;
	//int count = k4a_device_get_installed_count();

	if (k4a_device_open(0, &device) != K4A_RESULT_SUCCEEDED)
	{
		std::cerr << "" << endl;
		exit(1);
	}
	k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.synchronized_images_only = true;
    deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_30;
    deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_720P;
    deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;

    k4a_capture_t capture = NULL;

    k4a_image_t depthImage; //= k4a_capture_get_depth_image(capture);
    

    k4a_calibration_t calibration;
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &calibration))
    {
        cout << "Failed to get calibration" << endl;
        return;
    }
    const int width = calibration.color_camera_calibration.resolution_width;
    const int height = calibration.color_camera_calibration.resolution_height;
    auto calib = calibration.depth_camera_calibration;

    cv::Mat camera_matrix;
    cv::Mat new_camera_matrix;
    float factor{ 1.0 }; // scaling factor
    cv::Mat cv_undistorted_color;
    cv::Mat cv_undistorted_depth;
    cv::Mat cv_depth_downscaled;
    cv::Mat cv_color_downscaled;
    cv::Mat map1;
    cv::Mat map2;

    //init undistortion map
    auto intrinsics = calibration.color_camera_calibration.intrinsics.parameters.param;
    cv_undistorted_color = cv::Mat::zeros(
        height / factor,
        width / factor,
        CV_8UC4);

    cv_undistorted_depth = cv::Mat::zeros(
        height / factor,
        width / factor,
        CV_16U);

    cv_depth_downscaled = cv::Mat::zeros(
        height / factor,
        width / factor,
        CV_16U);
    cv_color_downscaled = cv::Mat::zeros(
        height / factor,
        width / factor,
        CV_8UC4);

    std::vector<double> _camera_matrix = {
intrinsics.fx / factor,
0.f,
intrinsics.cx / factor,
0.f,
intrinsics.fy / factor,
intrinsics.cy / factor,
0.f,
0.f,
1.f };
    camera_matrix = cv::Mat(3, 3, CV_64F, &_camera_matrix[0]);

    std::vector<double> _dist_coeffs = { intrinsics.k1, intrinsics.k2, intrinsics.p1,
                                       intrinsics.p2, intrinsics.k3, intrinsics.k4,
                                       intrinsics.k5, intrinsics.k6 };



    cv::Mat dist_coeffs = cv::Mat(8, 1, CV_64F, &_dist_coeffs[0]);
    new_camera_matrix = cv::getOptimalNewCameraMatrix(
        camera_matrix,
        dist_coeffs,
        cv_depth_downscaled.size(),
        0,
        cv_depth_downscaled.size());


    cv::Mat_<double> I = cv::Mat_<double>::eye(3, 3);

    map1 = cv::Mat::zeros(cv_depth_downscaled.size(), CV_16SC2);
    map2 = cv::Mat::zeros(cv_depth_downscaled.size(), CV_16UC1);
    initUndistortRectifyMap(camera_matrix, dist_coeffs, I, new_camera_matrix, cv::Size(width / factor, height / factor),
        map1.type(), map1, map2);


    k4a_transformation_t transformation = NULL;
    transformation = k4a_transformation_create(&calibration);

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &deviceConfig))
    {
        cout << "Failed to start cameras" << endl;
        return;
    }

    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);
    int count = 0;

    for (; ; )
    {
        std::ostringstream cnt;
        //string path = "Frames/";
        cnt << std::setw(6) << std::setfill('0') << count;
        std::string file = "frame-" + cnt.str() + ".depth.png";
        std::string file1 = "frame-" + cnt.str() + ".color.png";
        // Get a capture
        switch (k4a_device_get_capture(device, &capture, 1000))
        {
        case K4A_WAIT_RESULT_SUCCEEDED:
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            cout << "Timed out waiting for a capture" << endl;
            return ;
        case K4A_WAIT_RESULT_FAILED:
            cout << "Failed to read a capture" << endl;
            return ;
        }

        // Get a depth image
        depthImage = k4a_capture_get_depth_image(capture);
        if (depthImage == 0)
        {
            cout << "Failed to get depth image from capture" << endl;

        }

        int depth_image_width_pixels = k4a_image_get_width_pixels(depthImage);
        int depth_image_height_pixels = k4a_image_get_height_pixels(depthImage);
        cv::Mat imD = cv::Mat(depth_image_height_pixels, depth_image_width_pixels, CV_16UC1, (void*)k4a_image_get_buffer(depthImage));
        //ushort d = imD.ptr<ushort>(320)[320];
        //cout << "d = " << d << endl;
   // imshow("depth Image", imD);
    //waitKey(0);
    // Get a color image

        k4a_image_t color_image = k4a_capture_get_color_image(capture);

        if (color_image == 0)
        {
            cout << "Failed to get color image from capture" << endl;

        }
        int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
        int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
        cv::Mat imBGRA = cv::Mat(color_image_height_pixels, color_image_width_pixels, CV_8UC4, (void*)k4a_image_get_buffer(color_image));
        cv::Mat imBGR;
        cvtColor(imBGRA, imBGR, CV_BGRA2BGR);


        k4a_image_t transformed_depth_image;
        if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
            color_image_width_pixels,
            color_image_height_pixels,
            color_image_width_pixels * (int)sizeof(uint16_t),
            &transformed_depth_image))
        {
            cout << "Failed to create transformed depth image" << endl;
            return;
        }
        if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_color_camera(transformation, depthImage, transformed_depth_image))
        {

            cout << "Failed to compute transformed depth image" << endl;
            return;
        }


        cv::Mat frame;
        uint8_t* buffer = k4a_image_get_buffer(transformed_depth_image);
        uint16_t* depth_buffer = reinterpret_cast<uint16_t*>(buffer);
        create_mat_from_buffer(depth_buffer, color_image_width_pixels, color_image_height_pixels, 1).copyTo(frame);




        if (frame.empty())
            cout << "im_transformed_color_image is empty" << endl;


        cv::resize(
            frame,
            cv_depth_downscaled,
            cv_depth_downscaled.size(),
            CV_INTER_AREA);


        cv::resize(imBGRA,
            cv_color_downscaled,
            cv_color_downscaled.size(),
            cv::INTER_LINEAR);//CV_INTER_AREA

        remap(cv_depth_downscaled, cv_undistorted_depth, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        remap(cv_color_downscaled, cv_undistorted_color, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        cv::cvtColor(cv_undistorted_color, cv_undistorted_color, CV_BGRA2RGB);


        k4a_image_release(depthImage);
        k4a_image_release(color_image);
        k4a_image_release(transformed_depth_image);

        k4a_capture_release(capture);
        ++count;
    }
}

cv::Mat PointCloudViewer::create_mat_from_buffer(uint16_t* data, int width, int height, int channels = 1)
{
    cv::Mat mat(height, width, CV_MAKETYPE(CV_16U, channels));
    memcpy(mat.data, data, width * height * channels * sizeof(uint16_t));
    return mat;
}
