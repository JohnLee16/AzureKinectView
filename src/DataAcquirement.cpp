#include "DataAcquirement.h"
#include <iostream>
#include <Windows.h>


using namespace std;
using namespace cv;

#define VERIFY(result)                                                                                                 \
    if (result != K4A_RESULT_SUCCEEDED)                                                                                \
    {                                                                                                                  \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", #result " failed", __FILE__, __FUNCTION__, __LINE__);   \
        exit(1);                                                                                                       \
    }

DataAcquirement::DataAcquirement()
{
}

DataAcquirement::~DataAcquirement()
{
}

void DataAcquirement::clean_up(k4a_device_t device)
{
	if (device != NULL)
	{
		k4a_device_close(device);
	}
}
/// <summary>
/// Open kinect and start device
/// </summary>
/// <returns></returns>
bool DataAcquirement::OpenDevice()
{	
	k4a_device_t device = nullptr;
	int count = k4a_device_get_installed_count();

	if (k4a_device_open(0, &device) != K4A_RESULT_SUCCEEDED)
	{
		cerr << "" << endl;
		exit(1);
	}
	k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;

    // config for colored image
    /*deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;    
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_2160P;    
    deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_30;*/

    // config for depth image
    /*deviceConfig.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_2160P;
    deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_15;*/

    // config for IR image
    /*deviceConfig.depth_mode = K4A_DEPTH_MODE_PASSIVE_IR;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_2160P;
    deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_30;*/

    // configs
    deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_15;
    deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_2160P;
    deviceConfig.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;


	if (k4a_device_start_cameras(device, &deviceConfig) != K4A_RESULT_SUCCEEDED)
	{
		cerr << "" << endl;
		exit(1);
	}
	
    k4a_capture_t capture = NULL;
    switch (k4a_device_get_capture(device, &capture, 1000))
    {
    case K4A_WAIT_RESULT_SUCCEEDED:
        break;
    case K4A_WAIT_RESULT_TIMEOUT:
        printf("Timed out waiting for a capture\n");
        break;
    case K4A_WAIT_RESULT_FAILED:
        printf("Failed to read a capture\n");
    }
    
    k4a_image_t depthImage = k4a_capture_get_depth_image(capture);
    k4a_image_t irImage = k4a_capture_get_ir_image(capture);
    
    //VideoCapture videoCapture = VideoCapture(0);
    //while (videoCapture.isOpened())
    //{
    //    Mat frame;
    //    videoCapture >> frame;
    //    char key = waitKey(20);
    //    if (key == 27)
    //    {
    //        break;
    //    }
    //    imshow("image", frame);
    //    //videoCapture.grab();
    //    //videoCapture.read(images);
    //}
    
    
    while (TRUE)
    {
        if (k4a_device_get_capture(device, &capture, 1000) == K4A_WAIT_RESULT_SUCCEEDED)
        {
            Mat frame;
            depthImage = k4a_capture_get_depth_image(capture);
            frame = Mat(k4a_image_get_height_pixels(depthImage), k4a_image_get_width_pixels(depthImage), CV_8UC4, k4a_image_get_buffer(depthImage));
            char key = waitKey(40);
            if (key == 27)
            {
                break;
            }
            imshow("image", frame);
            k4a_capture_release(capture);
        }
        waitKey(10);
        //videoCapture.grab();
        //videoCapture.read(images);
    }


    uint8_t* data = k4a_image_get_buffer(depthImage);
    k4a_image_format_t formate = k4a_image_get_format(depthImage);
    Mat cImg = Mat(k4a_image_get_height_pixels(depthImage), k4a_image_get_width_pixels(depthImage), CV_8UC4, k4a_image_get_buffer(depthImage));
    //namedWindow("display window", WINDOW_AUTOSIZE);
    imshow("kinectViewer", cImg);
    waitKey(0);


    uint8_t* irdata = k4a_image_get_buffer(irImage);
    Mat irImg = Mat(k4a_image_get_height_pixels(depthImage), k4a_image_get_width_pixels(depthImage), CV_8UC4, k4a_image_get_buffer(depthImage));

	k4a_calibration_t calibration;

	/*if (K4A_RESULT_SUCCEEDED !=
		k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &calibration))
	{
		printf("Failed to get calibration\n");
		clean_up(device);
		return 1;
	}*/

    clean_up(device);
	
	return false;
}

bool DataAcquirement::OpenNIGrabberForKinect()
{

    return false;
}

bool DataAcquirement::GetPointCloud()
{
    k4a_device_t device = nullptr;
    k4a_image_t xy_table = NULL;
    k4a_image_t point_cloud = NULL;
    k4a_image_t depth_image = NULL;
    k4a_capture_t capture = NULL;
    std::string file_name;
    int point_count = 0;
    int count = k4a_device_get_installed_count();

    if (k4a_device_open(0, &device) != K4A_RESULT_SUCCEEDED)
    {
        cerr << "" << endl;
        exit(1);
    }
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_30;
    k4a_calibration_t calibration;
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &calibration))
    {
        printf("Failed to get calibration\n");
        exit(1);
    }

    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
        calibration.depth_camera_calibration.resolution_width,
        calibration.depth_camera_calibration.resolution_height,
        calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
        &xy_table);

    create_xy_table(&calibration, xy_table);

    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
        calibration.depth_camera_calibration.resolution_width,
        calibration.depth_camera_calibration.resolution_height,
        calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float3_t),
        &point_cloud);

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &deviceConfig))
    {
        printf("Failed to start cameras\n");
        exit(1);
    }

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("main demo", true));
    // Create a viewport
    int v1(0);
    viewer->createViewPort(0.0, 0.0, 1.0, 1.0, v1);
    viewer->setBackgroundColor(0.0, 0.0, 0.0);

    std::cout << "Showing the pointCloud..." << endl;
    
    viewer->addCoordinateSystem(0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
        // Get a capture
        switch (k4a_device_get_capture(device, &capture, 1000))
        {
        case K4A_WAIT_RESULT_SUCCEEDED:
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            printf("Timed out waiting for a capture\n");
            goto Exit;
        case K4A_WAIT_RESULT_FAILED:
            printf("Failed to read a capture\n");
            goto Exit;
        }

        // Get a depth image        
        depth_image = k4a_capture_get_depth_image(capture);
        if (depth_image == 0)
        {
            printf("Failed to get depth image from capture\n");
            continue;
        }

        generate_point_cloud(depth_image, xy_table, point_cloud, &point_count);

        pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud = transferToPointCloud(point_cloud, point_count);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_org_color(outputCloud, 255.0, 255, 255);
        viewer->addPointCloud(outputCloud, cloud_org_color, "MyCloud", v1);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "MyCloud");
        
        
        k4a_capture_release(capture);
        

        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
        viewer->removeAllPointClouds();
    }

    k4a_image_release(xy_table);
    k4a_image_release(point_cloud);
    k4a_image_release(depth_image);
Exit:
    if (device != NULL)
    {
        k4a_device_close(device);
    }
    return false;
}

bool DataAcquirement::GetK4aRecord()
{
    k4a_device_t device = nullptr;
    if (k4a_device_open(0, &device) != K4A_RESULT_SUCCEEDED)
    {
        cerr << "" << endl;
        exit(1);
    }
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    device_config.camera_fps = K4A_FRAMES_PER_SECOND_30;

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &device_config))
    {
        cerr << "" << endl;
        return false;
    }
    k4a_record_t recording_handle;
    if (K4A_RESULT_SUCCEEDED != k4a_record_create("k4arecord_output.mkv", device, device_config, &recording_handle))
    {
        cerr << "Something wrong in recording the device" << endl;
        return false;
    }
    // Add a custom video track to store processed depth images.
    // Read the depth resolution from the camera configuration so we can create our custom track with the same size.
    k4a_calibration_t sensor_calibration;
    VERIFY(k4a_device_get_calibration(device, device_config.depth_mode, K4A_COLOR_RESOLUTION_OFF, &sensor_calibration));
    uint32_t depth_width = (uint32_t)sensor_calibration.depth_camera_calibration.resolution_width;
    uint32_t depth_height = (uint32_t)sensor_calibration.depth_camera_calibration.resolution_height;

    BITMAPINFOHEADER codec_header;
    fill_bitmap_header(depth_width, depth_height, &codec_header);

    k4a_record_video_settings_t video_settings;
    video_settings.width = depth_width;
    video_settings.height = depth_height;
    video_settings.frame_rate = 30; // Should be the same rate as device_config.camera_fps

    // Add the video track to the recording.
    VERIFY(k4a_record_add_custom_video_track(recording_handle,
        "PROCESSED_DEPTH",
        "V_MS/VFW/FOURCC",
        (uint8_t*)(&codec_header),
        sizeof(codec_header),
        &video_settings));

    // Write the recording header now that all the track metadata is set up.
    VERIFY(k4a_record_write_header(recording_handle));

    // Start reading 100 depth frames (~3 seconds at 30 fps) from the camera.
    for (int frame = 0; frame < 1000; frame++)
    {
        k4a_capture_t capture;
        k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &capture, K4A_WAIT_INFINITE);
        if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            // Write the capture to the built-in tracks
            VERIFY(k4a_record_write_capture(recording_handle, capture));

            // Get the depth image from the capture so we can write a processed copy to our custom track.
            k4a_image_t depth_image = k4a_capture_get_depth_image(capture);
            if (depth_image)
            {
                // The YUY2 image format is the same stride as the 16-bit depth image, so we can modify it in-place.
                uint8_t* depth_buffer = k4a_image_get_buffer(depth_image);
                size_t depth_buffer_size = k4a_image_get_size(depth_image);
                for (size_t i = 0; i < depth_buffer_size; i += 2)
                {
                    // Convert the depth value (16-bit, in millimeters) to the YUY2 color format.
                    // The YUY2 format should be playable in video players such as VLC.
                    uint16_t depth = (uint16_t)(depth_buffer[i + 1] << 8 | depth_buffer[i]);
                    // Clamp the depth range to ~1 meter and scale it to fit in the Y channel of the image (8-bits).
                    if (depth > 0x3FF)
                    {
                        depth_buffer[i] = 0xFF;
                    }
                    else
                    {
                        depth_buffer[i] = (uint8_t)(depth >> 2);
                    }
                    // Set the U/V channel to 128 (i.e. grayscale).
                    depth_buffer[i + 1] = 128;
                }

                VERIFY(k4a_record_write_custom_track_data(recording_handle,
                    "PROCESSED_DEPTH",
                    k4a_image_get_device_timestamp_usec(depth_image),
                    depth_buffer,
                    (uint32_t)depth_buffer_size));

                k4a_image_release(depth_image);
            }

            k4a_capture_release(capture);
        }
        else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT)
        {
            // TIMEOUT should never be returned when K4A_WAIT_INFINITE is set.
            printf("k4a_device_get_capture() timed out!\n");
            break;
        }
        else
        {
            printf("k4a_device_get_capture() returned error: %d\n", get_capture_result);
            break;
        }
    }

    k4a_device_stop_cameras(device);

    printf("Saving recording...\n");
    VERIFY(k4a_record_flush(recording_handle));
    k4a_record_close(recording_handle);
    return false;
}

void DataAcquirement::create_xy_table(const k4a_calibration_t* calibration, k4a_image_t xy_table)
{
    k4a_float2_t* table_data = (k4a_float2_t*)(void*)k4a_image_get_buffer(xy_table);

    int width = calibration->depth_camera_calibration.resolution_width;
    int height = calibration->depth_camera_calibration.resolution_height;

    k4a_float2_t p;
    k4a_float3_t ray;
    int valid;

    for (int y = 0, idx = 0; y < height; y++)
    {
        p.xy.y = (float)y;
        for (int x = 0; x < width; x++, idx++)
        {
            p.xy.x = (float)x;

            k4a_calibration_2d_to_3d(
                calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

            if (valid)
            {
                table_data[idx].xy.x = ray.xyz.x;
                table_data[idx].xy.y = ray.xyz.y;
            }
            else
            {
                table_data[idx].xy.x = nanf("");
                table_data[idx].xy.y = nanf("");
            }
        }
    }
}


void DataAcquirement::generate_point_cloud(const k4a_image_t depth_image,
    const k4a_image_t xy_table,
    k4a_image_t point_cloud,
    int* point_count)
{
    int width = k4a_image_get_width_pixels(depth_image);
    int height = k4a_image_get_height_pixels(depth_image);

    uint16_t* depth_data = (uint16_t*)(void*)k4a_image_get_buffer(depth_image);
    k4a_float2_t* xy_table_data = (k4a_float2_t*)(void*)k4a_image_get_buffer(xy_table);
    k4a_float3_t* point_cloud_data = (k4a_float3_t*)(void*)k4a_image_get_buffer(point_cloud);

    *point_count = 0;
    for (int i = 0; i < width * height; i++)
    {
        if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
        {
            point_cloud_data[i].xyz.x = xy_table_data[i].xy.x * (float)depth_data[i];
            point_cloud_data[i].xyz.y = xy_table_data[i].xy.y * (float)depth_data[i];
            point_cloud_data[i].xyz.z = (float)depth_data[i];
            (*point_count)++;
        }
        else
        {
            point_cloud_data[i].xyz.x = nanf("");
            point_cloud_data[i].xyz.y = nanf("");
            point_cloud_data[i].xyz.z = nanf("");
        }
    }
}

void DataAcquirement::write_point_cloud(const char* file_name, const k4a_image_t point_cloud, int point_count)
{
    int width = k4a_image_get_width_pixels(point_cloud);
    int height = k4a_image_get_height_pixels(point_cloud);

    k4a_float3_t* point_cloud_data = (k4a_float3_t*)(void*)k4a_image_get_buffer(point_cloud);

    // save to the ply file
    std::ofstream ofs(file_name); // text mode first
    ofs << "ply" << std::endl;
    ofs << "format ascii 1.0" << std::endl;
    ofs << "element vertex"
        << " " << point_count << std::endl;
    ofs << "property float x" << std::endl;
    ofs << "property float y" << std::endl;
    ofs << "property float z" << std::endl;
    ofs << "end_header" << std::endl;
    ofs.close();

    std::stringstream ss;
    for (int i = 0; i < width * height; i++)
    {
        if (isnan(point_cloud_data[i].xyz.x) || isnan(point_cloud_data[i].xyz.y) || isnan(point_cloud_data[i].xyz.z))
        {
            continue;
        }

        ss << (float)point_cloud_data[i].xyz.x << " " << (float)point_cloud_data[i].xyz.y << " "
            << (float)point_cloud_data[i].xyz.z << std::endl;
    }

    std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}


pcl::PointCloud<pcl::PointXYZ>::Ptr DataAcquirement::transferToPointCloud(const k4a_image_t point_cloud, int point_count)
{
    
    int width = k4a_image_get_width_pixels(point_cloud);
    int height = k4a_image_get_height_pixels(point_cloud);

    k4a_float3_t* point_cloud_data = (k4a_float3_t*)(void*)k4a_image_get_buffer(point_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);

    outputCloud->width = width;
    outputCloud->height = height;
    //outputCloud->is_dense = false;
    outputCloud->points.resize((uint64_t)outputCloud->width * outputCloud->height);

    for (int i = 0; i < width * height; i++)
    {
        if (isnan(point_cloud_data[i].xyz.x) || isnan(point_cloud_data[i].xyz.y) || isnan(point_cloud_data[i].xyz.z))
        {
            outputCloud->points[i].x = 0;
            outputCloud->points[i].y = 0;
            outputCloud->points[i].z = 0;
            continue;
        }

        outputCloud->points[i].x = point_cloud_data[i].xyz.x;
        outputCloud->points[i].y = point_cloud_data[i].xyz.y;
        outputCloud->points[i].z = point_cloud_data[i].xyz.z;
    }
    return outputCloud;    
}


void DataAcquirement::fill_bitmap_header(uint32_t width, uint32_t height, BITMAPINFOHEADER* out)
{
    out->biSize = sizeof(BITMAPINFOHEADER);
    out->biWidth = width;
    out->biHeight = height;
    out->biPlanes = 1;
    out->biBitCount = 16;
    out->biCompression = FOURCC("YUY2");
    out->biSizeImage = sizeof(uint16_t) * width * height;
    out->biXPelsPerMeter = 0;
    out->biYPelsPerMeter = 0;
    out->biClrUsed = 0;
    out->biClrImportant = 0;
}
