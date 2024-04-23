#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class MarkerPublisher : public rclcpp::Node {
public:
    MarkerPublisher()
        : Node("marker_publisher_node")
    {
	auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

        marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(
		"vis_marker", qos);
	
	odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom", qos, std::bind(&MarkerPublisher::odomCallback, this, std::placeholders::_1));


	laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"scan", \
		rclcpp::SensorDataQoS(), \
		std::bind(
			&MarkerPublisher::laserCallback, \
			this, \
			std::placeholders::_1));
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double roll, pitch, yaw;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        robot_angle = yaw;
    }

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
	uint16_t scan_angle[3] = {0, 315, 270};

	for (int num = 0; num < 3; num++)
	{
		if (std::isinf(msg->ranges.at(scan_angle[num])))
		{
			scan_data_[num] = msg->range_max;
		}
		else
		{
			scan_data_[num] = msg->ranges.at(scan_angle[num]);
		}
	}
    }

    void publishMarker() {
        visualization_msgs::msg::Marker marker;

        // Set the marker properties
        marker.header.frame_id = "/map";
        marker.header.stamp = node->now();
        marker.ns = "points";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

	std::cout << "marker published" << std::endl;

        marker_pub->publish(marker);

    }

private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    double robot_angle = 0.0;
    double r = 0.0;
    double x = 0.0, y = 0.0;
    double scan_data_[3];
};

int main(int argc, char** argv) {
    std::cout << "it works" << std::endl;
    rclcpp::init(argc, argv);
    MarkerPublisher marker_publisher;
    marker_publisher.publishMarker();
    rclcpp::spin(std::make_shared<MarkerPublisher>());
    rclcpp::shutdown();
    std::cout << "before the return" << std::endl;
    return 0;
}
