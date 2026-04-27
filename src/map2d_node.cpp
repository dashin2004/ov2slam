#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class MapperNode : public rclcpp::Node {
public:
	MapperNode() : Node("ov2slam mapper 2d") {
			pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
				"/pointcloud", 10, std::bind(&MapperNode::point_cloud_callback,this, std::placeholders::_1));
			
			}
			pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
				"/vo_pose",10,std::bind(&MapperNode::pose_callback,this,std::placeholders::_1));
				grid_map.resize(width_map*height_map);
		}
private:
		void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
			pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

			pcl::fromROSMsg(*msg, pcl_cloud);
			
		}
		void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
			pose_x = msg->pose.position.x;
			pose_y = msg->pose.position.y;
		}
		void mapping(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
			for(auto& point : cloud->points){
				if (point.z > 1 || point.z < 0.1) {
					continue;
				}
				
				int grid_x = static_cast<int>((point.x + (map_size / 2.0)) / resolution);
				int grid_y = static_cast<int>((point.y + (map_size / 2.0)) / resolution);
				
				if (grid_x < 0 || grid_x >= width_map || grid_y < 0 || grid_y >= height_map) {
    				continue;
				}
				
				int index = (grid_y * width_map) + grid_x;
				grid_map[index] += 1;
				

			}

		}
			
		
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
		const int width_map = 66;
		const int height_map = 66;
		const float map_size = 2.0; //2m na mape
		const float resolution = map_size/width_map; //3cm na komorke
		std::vector<int> grid_map;
		float pose_x;
		float pose_y;