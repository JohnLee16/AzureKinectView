#pragma once
#include <k4a.h>
#include <k4arecord/record.h>
#include <k4arecord/record.hpp>
#include <k4arecord/k4arecord_export.h>
#include <opencv2/opencv.hpp>
#include <opencv2/video/video.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

class DataAcquirement
{
public:
	DataAcquirement();
	~DataAcquirement();
	bool OpenDevice();
	bool OpenNIGrabberForKinect();
	bool GetPointCloud();
	bool GetK4aRecord();
	//bool GetK4aRecord(k4a_device_t device, const k4a_device_configuration_t device_config, k4a_record_t* recording_handle);

	static void clean_up(k4a_device_t device);

private:
	void create_xy_table(const k4a_calibration_t* calibration, k4a_image_t xy_table);
	void generate_point_cloud(const k4a_image_t depth_image, const k4a_image_t xy_table, k4a_image_t point_cloud, int* point_count);
	void write_point_cloud(const char* file_name, const k4a_image_t point_cloud, int point_count);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transferToPointCloud(const k4a_image_t point_cloud, int point_count);
	void fill_bitmap_header(uint32_t width, uint32_t height, BITMAPINFOHEADER* out);
};


