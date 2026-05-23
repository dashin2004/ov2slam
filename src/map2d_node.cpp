#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include "std_msgs/msg/float32.hpp"

class MapperNode : public rclcpp::Node {
public:
    MapperNode() : Node("mapper2d") {
        pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/point_cloud", 10, std::bind(&MapperNode::point_cloud_callback, this, std::placeholders::_1));

        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&MapperNode::odom_callback, this, std::placeholders::_1));

        
        scale_pub_ = this->create_publisher<std_msgs::msg::Float32>("/slam_scale", 10);
        grid_map.assign(width_map * height_map, 0);
        tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    }

    void draw_gui() {
        ImGui::Begin("map");
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec2 p = ImGui::GetCursorScreenPos();

        for (int y = 0; y < height_map; y++) {
            for (int x = 0; x < width_map; x++) {
                int index = (y * width_map) + x;
                int hits = grid_map[index];
                ImU32 color;
                int redness = hits * 10;

                if (redness > 0) {
                    color = IM_COL32(std::min(redness, 255), 0, 0, 255);
                } else {
                    color = IM_COL32(255, 255, 255, 255);
                }

                draw_list->AddRectFilled(
                    ImVec2(p.x + (x * 5), p.y + (y * 5)),
                    ImVec2(p.x + ((x + 1) * 5), p.y + ((y + 1) * 5)),
                    color);
            }
        }

        int robot_grid_x = static_cast<int>((pose_x + (map_size / 2.0f)) / resolution);
        int robot_grid_y = static_cast<int>((pose_y + (map_size / 2.0f)) / resolution);

        if (robot_grid_x >= 0 && robot_grid_x < width_map && robot_grid_y >= 0 && robot_grid_y < height_map) {
            draw_list->AddCircleFilled(
                ImVec2(p.x + robot_grid_x * 5 + 2.5f, p.y + robot_grid_y * 5 + 2.5f),
                3.0f,
                IM_COL32(0, 255, 0, 255));
        }

        ImGui::Dummy(ImVec2(width_map * 5, height_map * 5));
        ImGui::End();
    }

private:
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*msg, *pcl_cloud);
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    
    
        transform.rotate(Eigen::AngleAxisf(-M_PI/2.0, Eigen::Vector3f::UnitZ()));
        transform.rotate(Eigen::AngleAxisf(-M_PI/2.0, Eigen::Vector3f::UnitX()));

        auto transformed_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::transformPointCloud(*pcl_cloud, *transformed_cloud, transform);
        // Scale the point cloud using current_scale to apply metric thresholds in RANSAC
        pcl::PointCloud<pcl::PointXYZ>::Ptr scaled_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto& p : transformed_cloud->points) {
            pcl::PointXYZ sp;
            sp.x = p.x * current_scale;
            sp.y = p.y * current_scale;
            sp.z = p.z * current_scale;
            scaled_cloud->points.push_back(sp);
        }

        // Filter points below the camera to find the floor candidates
        pcl::PointCloud<pcl::PointXYZ>::Ptr floor_candidates(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto& p : scaled_cloud->points) {
            if (p.z < -0.02) { 
                floor_candidates->points.push_back(p);
            }
        }

        if (floor_candidates->points.size() > 20) {
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            pcl::SACSegmentation<pcl::PointXYZ> seg;

            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(100);
            seg.setDistanceThreshold(0.05); // 5cm metric threshold
            seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0)); 
            seg.setEpsAngle(20.0f * (M_PI / 180.0f)); 

            seg.setInputCloud(floor_candidates);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size() > 10) {
                float h_metric = std::abs(coefficients->values[3]);
                float h_slam = h_metric / current_scale;
                
                if (h_slam > 0.001f) {
                    float measured_scale = cam_height / h_slam;
                    current_scale = (0.99f * current_scale) + (0.01f * measured_scale);
                    auto scale_msg = std_msgs::msg::Float32();
                    scale_msg.data = current_scale;
                    scale_pub_->publish(scale_msg);
                }
            }
        }
        mapping(transformed_cloud);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        pose_x = msg->pose.pose.position.x;
        pose_y = msg->pose.pose.position.y; 
    }

   

    void mapping(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        std::fill(grid_map.begin(), grid_map.end(), 0);
        for (auto& point : cloud->points) {
            float px = point.x * current_scale;
            float py = point.y * current_scale;
            float pz = point.z * current_scale;
            
            if (pz < -0.08 || pz > 0.1) {
                continue;
            }

            int grid_x = static_cast<int>((px + (map_size / 2.0)) / resolution);
            int grid_y = static_cast<int>((py + (map_size / 2.0)) / resolution);

            if (grid_x < 0 || grid_x >= width_map || grid_y < 0 || grid_y >= height_map) {
                continue;
            }

            int index = (grid_y * width_map) + grid_x;
            
                grid_map[index] += 10;
            
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr scale_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    float slam_height_est = -1.0f;
    const int width_map = 66;
    const int height_map = 66;
    const float map_size = 8.0f;
    const float resolution = map_size / width_map;
    std::vector<int> grid_map;
    float pose_x = 0.0f;
    float pose_y = 0.0f;
    float current_scale = 1.0f;
    const float cam_height = 0.092f; 
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapperNode>();

    if (!glfwInit()) return -1;
    GLFWwindow* window = glfwCreateWindow(800, 800, "OV2SLAM MAP", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    while (!glfwWindowShouldClose(window)) {
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
