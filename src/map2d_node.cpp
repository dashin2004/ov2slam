#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <mutex>
#include <vector>
#include <cmath>
#include <algorithm>

// ImGui + backends
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>

// ========================================================
//  2D Grid data structure
// ========================================================

struct Grid2D {
    float resolution = 0.03f;  // 3cm per cell
    int width = 67;            // approx 2m (67 * 0.03 = 2.01m)
    int height = 67;           // approx 2m (67 * 0.03 = 2.01m)
    std::vector<bool> cells;

    Grid2D() {
        cells.resize(width * height, false);
    }

    void addPoint(float wx, float wz) {
        // Shift so (0,0) is center of the grid
        int gx = static_cast<int>(std::floor(wx / resolution)) + width / 2;
        int gz = static_cast<int>(std::floor(wz / resolution)) + height / 2;
        
        // Only mark cell if within our 2mx2m boundaries
        if (gx >= 0 && gx < width && gz >= 0 && gz < height) {
            cells[gz * width + gx] = true;
        }
    }

    void clear() {
        std::fill(cells.begin(), cells.end(), false);
    }
};

// ========================================================
//  ROS2 Node
// ========================================================

class Map2DNode : public rclcpp::Node {
public:
    Map2DNode() : Node("map2d_node") {
        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "point_cloud", 10,
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                processPointCloud(msg);
            });

        RCLCPP_INFO(this->get_logger(), "Map2D node started. Waiting for point_cloud...");
    }

    std::mutex data_mutex_;
    Grid2D grid_;

private:
    void processPointCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        grid_.clear();

        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            float px = *iter_x;
            float py = *iter_y;
            float pz = *iter_z;

            // Filter by height (Y axis in OV2SLAM world frame) to avoid floors/ceilings
            if (py >= -0.5f && py <= 0.5f) {
                grid_.addPoint(px, pz);
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
};

// ========================================================
//  Main
// ========================================================

static void glfw_error_callback(int error, const char* description) {
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Map2DNode>();

    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to init GLFW");
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    GLFWwindow* window = glfwCreateWindow(800, 800, "OV2SLAM - Simple 2D Grid Map", nullptr, nullptr); 
    if (!window) {
        RCLCPP_ERROR(node->get_logger(), "Failed to create GLFW window");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        
        // Spin ROS (non-blocking)
        rclcpp::spin_some(node);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowPos(ImVec2(0, 0));
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        ImGui::SetNextWindowSize(ImVec2((float)display_w, (float)display_h));

        // Create a single borderless window spanning the full screen
        ImGui::Begin("2D Map Canvas", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoBringToFrontOnFocus);
        {
            ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
            ImVec2 canvas_size = ImGui::GetContentRegionAvail();
            ImVec2 canvas_center(canvas_pos.x + canvas_size.x * 0.5f,
                                 canvas_pos.y + canvas_size.y * 0.5f);

            ImDrawList* draw = ImGui::GetWindowDrawList();

            // Dark background for map area
            draw->AddRectFilled(canvas_pos,
                ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
                IM_COL32(15, 15, 25, 255));

            std::lock_guard<std::mutex> lock(node->data_mutex_);
            
            // Calculate a uniform scaling to draw the maximum size on screen with 90% margin
            float cell_px = std::min(canvas_size.x / node->grid_.width,
                                     canvas_size.y / node->grid_.height) * 0.9f;

            float grid_pixel_w = node->grid_.width * cell_px;
            float grid_pixel_h = node->grid_.height * cell_px;
            
            float start_x = canvas_center.x - grid_pixel_w / 2.0f;
            float start_y = canvas_center.y - grid_pixel_h / 2.0f;

            // Render each cell of the grid map
            for (int gz = 0; gz < node->grid_.height; ++gz) {
                for (int gx = 0; gx < node->grid_.width; ++gx) {
                    ImVec2 p0(start_x + gx * cell_px, start_y + gz * cell_px);
                    ImVec2 p1(p0.x + cell_px, p0.y + cell_px);
                    
                    // If occupied space, draw as solid white block
                    if (node->grid_.cells[gz * node->grid_.width + gx]) {
                        draw->AddRectFilled(p0, p1, IM_COL32(230, 230, 230, 255));
                    }
                    
                    // Draw cell outlines perfectly aligned
                    draw->AddRect(p0, p1, IM_COL32(40, 40, 50, 255));
                }
            }
        }
        ImGui::End();

        // Render
        ImGui::Render();
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.06f, 0.06f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    rclcpp::shutdown();

    return 0;
}
