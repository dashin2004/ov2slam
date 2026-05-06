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
	MapperNode() : Node("mapper2d") {
			pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
				"/point_cloud", 10, std::bind(&MapperNode::point_cloud_callback,this, std::placeholders::_1));
			
			
			pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
				"/vo_pose",10,std::bind(&MapperNode::pose_callback,this,std::placeholders::_1));
				grid_map.resize(width_map*height_map);
			}
			void draw_gui(){
				ImGui::Begin("map");
				//drawing
				ImDrawList* draw_list = ImGui::GetWindowDrawList();
				ImVec2 p = ImGui::GetCursorScreenPos();
				for(int y = 0; y < height_map; y++){
					for(int x = 0; x < width_map; x++){
						int index = (y*66) + x;
						int hits = grid_map[index];
						ImU32 color;
						int redness = hits*5;

						if (redness < 255 && redness > 0){
							color = IM_COL32(redness, 0, 0, 255);
						}
						else if(redness == 0){
							color = IM_COL32(255,255,255,255);
						}
						else{
							color = IM_COL32(255, 0, 0, 255);
						}
						draw_list->AddRectFilled(ImVec2(p.x+ (x*5), p.y +(y*5)),
						ImVec2(p.x + ((x + 1) * 5), p.y + ((y + 1) * 5)),  
						color );
					}
				}
				int robot_grid_x = static_cast<int>((pose_x + (map_size / 2.0f)) / resolution);
				int robot_grid_y = static_cast<int>((pose_y + (map_size / 2.0f)) / resolution);
				draw_list->AddCircleFilled(ImVec2(p.x + robot_grid_x*5,p.y + robot_grid_y*5),
				3.0f,
				IM_COL32(0, 255, 0, 255));
				ImGui::Dummy(ImVec2(width_map * 5, height_map * 5));
				ImGui::End();
			}
		
private:
		void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
			auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
			pcl::fromROSMsg(*msg, *pcl_cloud);
			mapping(pcl_cloud);
			
		}
		void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
			pose_x = msg->pose.position.x;
			pose_y = msg->pose.position.z;
		}
		void mapping(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
			for(auto& point : cloud->points){
				if (point.y < -0.5 || point.y > 0.5) {
					continue;
				}
				
				int grid_x = static_cast<int>((point.x + (map_size / 2.0)) / resolution);
				int grid_y = static_cast<int>((point.z + (map_size / 2.0)) / resolution);
				
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
		const float map_size = 20.0; //2m na mape
		const float resolution = map_size/width_map; //3cm na komorke
		std::vector<int> grid_map;
		float pose_x;
		float pose_y;
	};
int main(int argc, char**  argv){
	rclcpp::init(argc,argv);
	auto node = std::make_shared<MapperNode>();
	if(!glfwInit()) return -1;
	GLFWwindow* window = glfwCreateWindow(800, 800, "OV2SLAM MAP", NULL, NULL);
	glfwMakeContextCurrent(window);
	ImGui::CreateContext();
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init("#version 130");

	while(!glfwWindowShouldClose(window)){
		glfwPollEvents();
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();
		rclcpp::spin_some(node);
		node->draw_gui();
		ImGui::Render();
		int display_w, display_h;
		glfwGetFramebufferSize(window, &display_w, &display_h);
		glViewport(0, 0, display_w, display_h);
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		glfwSwapBuffers(window);
	}
	    
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    
    
    glfwDestroyWindow(window);
    glfwTerminate();
    
    
    rclcpp::shutdown();
    return 0;

}
