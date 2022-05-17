#pragma once
#include <rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class AStarNode : public rclcpp::Node {
public:
    AStarNode();

private:
    inline void grid_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        grid_ = msg;
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_grid_;
    nav_msgs::msg::OccupancyGrid::SharedPtr grid_;
};
