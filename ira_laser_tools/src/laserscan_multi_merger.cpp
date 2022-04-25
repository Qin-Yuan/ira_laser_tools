#include <ros/ros.h>
#include <string.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h> 
#include "sensor_msgs/LaserScan.h"
#include "pcl_ros/point_cloud.h"
#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>
// aserscan_multi_merger是基于ROS的将多个激光雷达传感器扫描的topic整合成一个topic的工具
#include <ira_laser_tools/laserscan_multi_mergerConfig.h>

using namespace std;
using namespace pcl;				 	 // 点云数据
using namespace laserscan_multi_merger;  // 激光雷达融合

class LaserscanMerger
{
public:
    LaserscanMerger();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, std::string topic);		 // 回调函数
    void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud); // 点云转雷达数据
    void reconfigureCallback(laserscan_multi_mergerConfig &config, uint32_t level);			 // 参数回调函数，获取点云融合后的参数

private:
    ros::NodeHandle node_;
    laser_geometry::LaserProjection projector_;   	// 将雷达的scan转换为点云
    tf::TransformListener tfListener_;			  	// tf坐标接听器

    ros::Publisher point_cloud_publisher_;      	// 点云发布
    ros::Publisher laser_scan_publisher_;			// 激光雷达发布
    vector<ros::Subscriber> scan_subscribers;		// vector动态消息接受，动态接受激光雷达数据
    vector<bool> clouds_modified;					// 点云容器
    vector<pcl::PCLPointCloud2> clouds;				// 点云数据
    vector<string> input_topics;					// 输入话题

    void laserscan_topic_parser();

    double angle_min;
    double angle_max;
    double angle_increment;
    double time_increment;
    double scan_time;
    double range_min;
    double range_max;

    string destination_frame;			// 目标帧(输出)
    string cloud_destination_topic;		// 点云话题(输出)
    string scan_destination_topic;		// 激光雷达话题(输出)
    string laserscan_topics;			// 激光扫描话题列(输入)
};

// 雷达数据融合
void LaserscanMerger::reconfigureCallback(laserscan_multi_mergerConfig &config, uint32_t level)
{
	this->angle_min = config.angle_min;              // 融合后的最小角度
	this->angle_max = config.angle_max;				 // 融合后的最大角度
	this->angle_increment = config.angle_increment;	 // 融合后的角度增量
	this->time_increment = config.time_increment;	 // 融合后的时间增量
	this->scan_time = config.scan_time;				 // 融合后的扫描时间
	this->range_min = config.range_min;				 // 融合后的最小范围
	this->range_max = config.range_max;				 // 融合后的最大范围
}

void LaserscanMerger::laserscan_topic_parser()
{
	// 订阅LaserScan主题
	ros::master::V_TopicInfo topics;
	ros::master::getTopics(topics);

    istringstream iss(laserscan_topics);
	vector<string> tokens;
	copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter<vector<string> >(tokens));
	vector<string> tmp_input_topics;
	for(int i=0;i<tokens.size();++i)
	{
        for(int j=0;j<topics.size();++j)
		{
			if( (tokens[i].compare(topics[j].name) == 0) && (topics[j].datatype.compare("sensor_msgs/LaserScan") == 0) )
			{
				tmp_input_topics.push_back(topics[j].name);
			}
		}
	}

	sort(tmp_input_topics.begin(),tmp_input_topics.end());
	std::vector<string>::iterator last = std::unique(tmp_input_topics.begin(), tmp_input_topics.end());
	tmp_input_topics.erase(last, tmp_input_topics.end());


	// 如果主题相同，不要重新订阅
	if( (tmp_input_topics.size() != input_topics.size()) || !equal(tmp_input_topics.begin(),tmp_input_topics.end(),input_topics.begin()))
	{

		// 取消订阅以前的主题
		for(int i=0; i<scan_subscribers.size(); ++i)
			scan_subscribers[i].shutdown();

		input_topics = tmp_input_topics;
		if(input_topics.size() > 0)
		{
            scan_subscribers.resize(input_topics.size());
			clouds_modified.resize(input_topics.size());
			clouds.resize(input_topics.size());
            ROS_INFO("Subscribing to topics\t%ld", scan_subscribers.size());
			for(int i=0; i<input_topics.size(); ++i)
			{
                scan_subscribers[i] = node_.subscribe<sensor_msgs::LaserScan> (input_topics[i].c_str(), 1, boost::bind(&LaserscanMerger::scanCallback,this, _1, input_topics[i]));
				clouds_modified[i] = false;
				cout << input_topics[i] << " ";
			}
		}
		else
            ROS_INFO("Not subscribed to any topic.");
	}
}

// 激光累雷达数据融合
LaserscanMerger::LaserscanMerger()
{
	ros::NodeHandle nh("~");

    nh.param<std::string>("destination_frame", destination_frame, "cart_frame");
    nh.param<std::string>("cloud_destination_topic", cloud_destination_topic, "/merged_cloud");
    nh.param<std::string>("scan_destination_topic", scan_destination_topic, "/scan_multi");
    nh.param<std::string>("laserscan_topics", laserscan_topics, "");
    nh.param("angle_min", angle_min, -2.36);
    nh.param("angle_max", angle_max, 2.36);
    nh.param("angle_increment", angle_increment, 0.0058);
    nh.param("scan_time", scan_time, 0.0333333);
    nh.param("range_min", range_min, 0.05);
    nh.param("range_max", range_max, 12.0);

    this->laserscan_topic_parser();			// 激光雷达解释器

	point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> (cloud_destination_topic.c_str(), 1, false);  // 点云数据发布
	laser_scan_publisher_ = node_.advertise<sensor_msgs::LaserScan> (scan_destination_topic.c_str(), 1, false);		 // 激光雷达话题发布
}

// 激光雷达数据回调，输入激光雷达数据 scan 和 话题名称 topic
void LaserscanMerger::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, std::string topic)
{
	sensor_msgs::PointCloud tmpCloud1,tmpCloud2;	// 点云数据
	sensor_msgs::PointCloud2 tmpCloud3;             // 点云数据，将LaserScan转化为点云数据

    // 验证TF是否知道如何从接收的扫描转换为目标扫描帧
	tfListener_.waitForTransform(scan->header.frame_id.c_str(), destination_frame.c_str(), scan->header.stamp, ros::Duration(1));
	// 雷达数据转换为点云数据, 最后一个参数，使用距离通道
    projector_.transformLaserScanToPointCloud(scan->header.frame_id, *scan, tmpCloud1, tfListener_, laser_geometry::channel_option::Distance);
	try
	{
		// 其中第一个参数为目标坐标系，第二个参数为原始点云，第三个参数为目标点云，
		// 第四个参数为接收到的坐标，正变换、逆变换 都包含其中
		tfListener_.transformPointCloud(destination_frame.c_str(), tmpCloud1, tmpCloud2);
	}catch (tf::TransformException ex){ROS_ERROR("%s",ex.what());return;}

	for(int i=0; i<input_topics.size(); ++i)
	{
		if(topic.compare(input_topics[i]) == 0)
		{
			sensor_msgs::convertPointCloudToPointCloud2(tmpCloud2,tmpCloud3);
			pcl_conversions::toPCL(tmpCloud3, clouds[i]);
			clouds_modified[i] = true;
		}
	}	

    // 记录有多少次扫描
	int totalClouds = 0;
	for(int i=0; i<clouds_modified.size(); ++i)
		if(clouds_modified[i])
			++totalClouds;

    // 只在所有订阅的扫描已经到达的情况下继续
	if(totalClouds == clouds_modified.size())
	{
		pcl::PCLPointCloud2 merged_cloud = clouds[0];
		clouds_modified[0] = false;

		for(int i=1; i<clouds_modified.size(); ++i)
		{
			pcl::concatenatePointCloud(merged_cloud, clouds[i], merged_cloud);
			clouds_modified[i] = false;
		}
	
		point_cloud_publisher_.publish(merged_cloud);

		Eigen::MatrixXf points;
		getPointCloudAsEigen(merged_cloud,points);
		pointcloud_to_laserscan(points, &merged_cloud);   // 点云数据转为雷达数据并发出
	}
}

// 点云数据转为雷达数据并发出							矩阵						点云
void LaserscanMerger::pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud)
{
	sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());			// 新建一个雷达数据
	output->header = pcl_conversions::fromPCL(merged_cloud->header);		// 雷达数据 header
	output->header.frame_id = destination_frame.c_str();	// 输出数据引用帧
	output->header.stamp = ros::Time::now();  				// ros时间戳
	output->angle_min = this->angle_min;					// 最小角度
	output->angle_max = this->angle_max;					// 最大角度
	output->angle_increment = this->angle_increment;		// 角度增量
	output->time_increment = this->time_increment;			// 时间增量
	output->scan_time = this->scan_time;					// 扫描时间
	output->range_min = this->range_min;					// 最小距离
	output->range_max = this->range_max;	        		// 最大距离
	// std::ceil向上取整函数
	uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
	// 容器赋值？
	output->ranges.assign(ranges_size, output->range_max + 1.0);
	// 矩阵运算
	for(int i=0; i<points.cols(); i++)
	{
		const float &x = points(0,i);	// x轴
		const float &y = points(1,i);	// y轴
		const float &z = points(2,i);	// z轴
		// std::isnan()用于判断是否为非法数字(not a number)
		if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
		{
			// ROS_DEBUG
			ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
			continue;
		}
		// 由x、y轴值计算距离的平方
		double range_sq = y*y+x*x;
		// 计算最小距离range的平方
		double range_min_sq_ = output->range_min * output->range_min;
		// 判断是否小于最小距离，小于则跳出本次循环
		if (range_sq < range_min_sq_) {
			ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
			continue;
		}
		// 由x、y轴计算角度，atan2是y/x的反正切，单位是弧度，其值介于-Pi~Pi之间
		double angle = atan2(y, x);
		// 判断点云的角度是否在最小、最大角度范围内；不在跳出本次循环
		if (angle < output->angle_min || angle > output->angle_max)
		{
			ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
			continue;
		}
		// 角度差除以角度增量
		int index = (angle - output->angle_min) / output->angle_increment;
		// 
		if (output->ranges[index] * output->ranges[index] > range_sq)
			output->ranges[index] = sqrt(range_sq)/10;
	}
	// 发布转换后的激光雷达数据
	laser_scan_publisher_.publish(output);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "laser_multi_merger");

    LaserscanMerger _laser_merger;

    dynamic_reconfigure::Server<laserscan_multi_mergerConfig> server;
    dynamic_reconfigure::Server<laserscan_multi_mergerConfig>::CallbackType f;

    f = boost::bind(&LaserscanMerger::reconfigureCallback,&_laser_merger, _1, _2);
	server.setCallback(f);

	ros::spin();

	return 0;
}