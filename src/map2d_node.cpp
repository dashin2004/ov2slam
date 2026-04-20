#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <mutex>
#include <vector>
#include <cmath>
#include <algorithm>
#include <unordered_map>

// ImGui + backends
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>

// ========================================================
//  2D Grid data structure
// ========================================================

struct GridCell {
    int count = 0;  // number of points that fell into this cell
};

struct Grid2D {
    float resolution = 0.05f;  // meters per cell
    std::unordered_map<int64_t, GridCell> cells;
    int max_count = 1;

    // Bounds tracking
    float x_min = 1e9f, x_max = -1e9f;
    float z_min = 1e9f, z_max = -1e9f;

    int64_t key(int gx, int gz) const {
        return (static_cast<int64_t>(gx) << 32) | (static_cast<int64_t>(gz) & 0xFFFFFFFF);
    }

    void addPoint(float wx, float wz) {
        int gx = static_cast<int>(std::floor(wx / resolution));
        int gz = static_cast<int>(std::floor(wz / resolution));
        int64_t k = key(gx, gz);
        cells[k].count++;
        if (cells[k].count > max_count)
            max_count = cells[k].count;

        if (wx < x_min) x_min = wx;
        if (wx > x_max) x_max = wx;
        if (wz < z_min) z_min = wz;
        if (wz > z_max) z_max = wz;
    }

    void clear() {
        cells.clear();
        max_count = 1;
        x_min = 1e9f; x_max = -1e9f;
        z_min = 1e9f; z_max = -1e9f;
    }
};

// ========================================================
//  ROS2 Node
// ========================================================

class Map2DNode : public rclcpp::Node {
public:
    Map2DNode() : Node("map2d_node") {
        // Subscribe to OV2SLAM topics
        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "point_cloud", 10,
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                processPointCloud(msg);
            });

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "vo_pose", 10,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                // OV2SLAM camera convention: X=right, Y=down, Z=forward
                // For top-down: we use X as horizontal, Z as vertical in the bird's eye view
                cam_x_ = msg->pose.position.x;
                cam_y_ = msg->pose.position.y;
                cam_z_ = msg->pose.position.z;
                traj_.push_back({cam_x_, cam_z_});
                has_pose_ = true;
            });

        RCLCPP_INFO(this->get_logger(), "Map2D node started. Waiting for point_cloud and vo_pose...");
    }

    // Public accessors for the render loop
    std::mutex data_mutex_;
    Grid2D grid_;
    float cam_x_ = 0.f, cam_y_ = 0.f, cam_z_ = 0.f;
    bool has_pose_ = false;
    std::vector<std::pair<float,float>> traj_;

    // Height slice parameters (Y axis = height in OV2SLAM camera convention)
    float y_min_ = -0.5f;
    float y_max_ =  0.5f;

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

            // Filter by height (Y axis in OV2SLAM world frame)
            if (py >= y_min_ && py <= y_max_) {
                // Top-down projection: X → horizontal, Z → vertical
                grid_.addPoint(px, pz);
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};

// ========================================================
//  ImGui Render Helpers
// ========================================================

static ImU32 heatColor(float t) {
    // 0.0 = dark blue → 1.0 = bright yellow
    t = std::clamp(t, 0.0f, 1.0f);
    float r, g, b;
    if (t < 0.25f) {
        float s = t / 0.25f;
        r = 0.0f; g = s; b = 0.5f + 0.5f * s;
    } else if (t < 0.5f) {
        float s = (t - 0.25f) / 0.25f;
        r = 0.0f; g = 0.5f + 0.5f * s; b = 1.0f - s;
    } else if (t < 0.75f) {
        float s = (t - 0.5f) / 0.25f;
        r = s; g = 1.0f; b = 0.0f;
    } else {
        float s = (t - 0.75f) / 0.25f;
        r = 1.0f; g = 1.0f - 0.5f * s; b = 0.0f;
    }
    return IM_COL32((int)(r*255), (int)(g*255), (int)(b*255), 255);
}

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

    // ---- GLFW + OpenGL setup ----
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to init GLFW");
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    GLFWwindow* window = glfwCreateWindow(1280, 900, "OV2SLAM - 2D Map", nullptr, nullptr);
    if (!window) {
        RCLCPP_ERROR(node->get_logger(), "Failed to create GLFW window");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // ---- ImGui setup ----
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();

    // Make it look nicer
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 8.0f;
    style.FrameRounding = 4.0f;
    style.GrabRounding = 4.0f;
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.08f, 0.08f, 0.12f, 1.0f);

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    // ---- Map view state ----
    float zoom = 200.0f;  // pixels per meter
    ImVec2 pan_offset(0.0f, 0.0f);
    bool dragging = false;
    ImVec2 drag_start(0,0);
    ImVec2 pan_start(0,0);

    // ---- Main loop ----
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Spin ROS for a bit (non-blocking)
        rclcpp::spin_some(node);

        // Start ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // =====================
        // Control Panel Window
        // =====================
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(320, 300), ImGuiCond_FirstUseEver);
        ImGui::Begin("Map Controls");
        {
            ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "OV2SLAM 2D Map Viewer");
            ImGui::Separator();

            ImGui::Text("Height Slice (Y axis)");
            ImGui::SliderFloat("Y min", &node->y_min_, -5.0f, 5.0f, "%.2f m");
            ImGui::SliderFloat("Y max", &node->y_max_, -5.0f, 5.0f, "%.2f m");
            ImGui::Separator();

            ImGui::Text("View");
            ImGui::SliderFloat("Zoom", &zoom, 10.0f, 2000.0f, "%.0f px/m");
            if (ImGui::Button("Reset View")) {
                zoom = 200.0f;
                pan_offset = ImVec2(0, 0);
            }
            ImGui::SameLine();
            if (ImGui::Button("Center on Camera") && node->has_pose_) {
                pan_offset = ImVec2(0, 0);
            }
            ImGui::Separator();

            ImGui::Text("Grid resolution");
            float res = node->grid_.resolution;
            if (ImGui::SliderFloat("Cell size", &res, 0.01f, 0.5f, "%.3f m")) {
                std::lock_guard<std::mutex> lock(node->data_mutex_);
                node->grid_.resolution = res;
            }
            ImGui::Separator();

            {
                std::lock_guard<std::mutex> lock(node->data_mutex_);
                ImGui::Text("Grid cells: %zu", node->grid_.cells.size());
                ImGui::Text("Max hits/cell: %d", node->grid_.max_count);
                ImGui::Text("Trajectory pts: %zu", node->traj_.size());
                if (node->has_pose_)
                    ImGui::Text("Camera: (%.2f, %.2f, %.2f)", node->cam_x_, node->cam_y_, node->cam_z_);
                else
                    ImGui::TextColored(ImVec4(1,0.3f,0.3f,1), "No pose received yet");
            }
        }
        ImGui::End();

        // =====================
        // Map Canvas Window
        // =====================
        ImGui::SetNextWindowPos(ImVec2(340, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(920, 880), ImGuiCond_FirstUseEver);
        ImGui::Begin("2D Map", nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        {
            ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
            ImVec2 canvas_size = ImGui::GetContentRegionAvail();
            ImVec2 canvas_center(canvas_pos.x + canvas_size.x * 0.5f,
                                  canvas_pos.y + canvas_size.y * 0.5f);

            ImDrawList* draw = ImGui::GetWindowDrawList();

            // Dark background
            draw->AddRectFilled(canvas_pos,
                ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
                IM_COL32(15, 15, 25, 255));

            // Handle mouse panning & zooming in canvas
            ImGui::InvisibleButton("canvas", canvas_size, ImGuiButtonFlags_MouseButtonLeft);
            bool canvas_hovered = ImGui::IsItemHovered();

            if (canvas_hovered && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
                ImVec2 delta = io.MouseDelta;
                pan_offset.x += delta.x;
                pan_offset.y += delta.y;
            }
            if (canvas_hovered && io.MouseWheel != 0.0f) {
                float scale_factor = (io.MouseWheel > 0) ? 1.15f : 0.87f;
                zoom *= scale_factor;
                zoom = std::clamp(zoom, 10.0f, 5000.0f);
            }

            // World-to-screen transform:
            // screen_x = canvas_center.x + (world_x - cam_x) * zoom + pan_offset.x
            // screen_y = canvas_center.y + (world_z - cam_z) * zoom + pan_offset.y
            // (Z-forward becomes screen-Y going down)
            auto world_to_screen = [&](float wx, float wz) -> ImVec2 {
                float sx = canvas_center.x + (wx - node->cam_x_) * zoom + pan_offset.x;
                float sy = canvas_center.y + (wz - node->cam_z_) * zoom + pan_offset.y;
                return ImVec2(sx, sy);
            };

            // Draw grid cells as heatmap
            {
                std::lock_guard<std::mutex> lock(node->data_mutex_);
                float cell_px = node->grid_.resolution * zoom;
                float inv_max = 1.0f / std::max(1, node->grid_.max_count);

                for (auto& [k, cell] : node->grid_.cells) {
                    int gx = static_cast<int>(k >> 32);
                    int gz = static_cast<int>(k & 0xFFFFFFFF);
                    // Sign-extend gz
                    if (gz & 0x80000000) gz |= ~0xFFFFFFFF;

                    float wx = gx * node->grid_.resolution;
                    float wz = gz * node->grid_.resolution;

                    ImVec2 p0 = world_to_screen(wx, wz);
                    ImVec2 p1(p0.x + cell_px, p0.y + cell_px);

                    // Clip to canvas
                    if (p1.x < canvas_pos.x || p0.x > canvas_pos.x + canvas_size.x ||
                        p1.y < canvas_pos.y || p0.y > canvas_pos.y + canvas_size.y)
                        continue;

                    float t = std::pow(static_cast<float>(cell.count) * inv_max, 0.4f);
                    draw->AddRectFilled(p0, p1, heatColor(t));
                }

                // Draw trajectory
                if (node->traj_.size() > 1) {
                    for (size_t i = 1; i < node->traj_.size(); i++) {
                        ImVec2 a = world_to_screen(node->traj_[i-1].first, node->traj_[i-1].second);
                        ImVec2 b = world_to_screen(node->traj_[i].first, node->traj_[i].second);
                        // Clip very distant segments
                        if (std::abs(a.x - b.x) > 2000 || std::abs(a.y - b.y) > 2000)
                            continue;
                        draw->AddLine(a, b, IM_COL32(80, 160, 255, 180), 2.0f);
                    }
                }

                // Draw camera position
                if (node->has_pose_) {
                    ImVec2 cam_screen = world_to_screen(node->cam_x_, node->cam_z_);
                    draw->AddCircleFilled(cam_screen, 7.0f, IM_COL32(255, 60, 60, 255));
                    draw->AddCircle(cam_screen, 9.0f, IM_COL32(255, 200, 200, 200), 0, 2.0f);
                }
            }

            // Scale bar
            {
                float bar_m = 1.0f;
                if (zoom < 50.0f) bar_m = 5.0f;
                else if (zoom < 100.0f) bar_m = 2.0f;
                else if (zoom > 500.0f) bar_m = 0.5f;
                else if (zoom > 1000.0f) bar_m = 0.2f;

                float bar_px = bar_m * zoom;
                ImVec2 bar_start(canvas_pos.x + 20, canvas_pos.y + canvas_size.y - 30);
                ImVec2 bar_end(bar_start.x + bar_px, bar_start.y);
                draw->AddLine(bar_start, bar_end, IM_COL32(255,255,255,200), 3.0f);
                char buf[32];
                snprintf(buf, sizeof(buf), "%.1f m", bar_m);
                draw->AddText(ImVec2(bar_start.x, bar_start.y - 18), IM_COL32(255,255,255,200), buf);
            }

            // Axis labels
            draw->AddText(ImVec2(canvas_pos.x + canvas_size.x - 60, canvas_center.y - 15),
                          IM_COL32(150,150,150,180), "X ->");
            draw->AddText(ImVec2(canvas_center.x + 5, canvas_pos.y + 5),
                          IM_COL32(150,150,150,180), "Z v");
        }
        ImGui::End();

        // Render
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
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
