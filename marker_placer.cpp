#include "marker_placer/marker_placer.hpp"

MarkerPlacer::MarkerPlacer()
    : Node("marker_placer"),
      awaiting_scan_data(false),
      marker_count(0) {

    marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("vis_marker", 10);

    blob_sub = this->create_subscription<std_msgs::msg::String>(
	"blob_points", 10, std::bind(&MarkerPlacer::blob_callback, this, std::placeholders::_1));

    laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
	"scan", rclcpp::SensorDataQoS(), std::bind(&MarkerPlacer::laser_callback, this, std::placeholders::_1));

    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    this->declare_parameter("robot_should_stop", false);

    RCLCPP_INFO(this->get_logger(), "Initializing MarkerPlacer Node");
}

void MarkerPlacer::blob_callback(const std_msgs::msg::String::SharedPtr msg) {
    	RCLCPP_INFO(this->get_logger(), "Blob callback triggered: %s", msg->data.c_str());
	auto data = split(msg->data, ';');
	if (data.size() != 3) {
	    RCLCPP_ERROR(this->get_logger(), "Received string does not contain expected format 'color1;color2;angle'");
	    return;
	}

	color1 = data[0]; // std::string color1 = data[0];
	color2 = data[1];

	double d_angle;
	try {
	    d_angle = std::stod(data[2]);
	} catch (const std::exception& e) {
	    RCLCPP_ERROR(this->get_logger(), "Failed to convert angle to double: %s", e.what());
	    return;
	}

	scan_angle = int(d_angle);

	// Set the parameter that other nodes can check
	this->set_parameter(rclcpp::Parameter("robot_should_stop", true)); // Using paramaters
	RCLCPP_INFO(this->get_logger(), "Robot stop signal sent.");

	// This triggers laser_callback to read the laser at the extracted angle and begin creating markers, all is handled from there.
	// TODO: UNCOMMENT awaiting_scan_data = true;
	// Laser callback will now place the marker. Done.

	// #################FOR TESTING PURPOSES ONLY SKIPS OVER LASER_CALLBACK: #########################
	// #################FOR TESTING PURPOSES ONLY SKIPS OVER LASER_CALLBACK: #########################
	// #################FOR TESTING PURPOSES ONLY SKIPS OVER LASER_CALLBACK: #########################
	// #################FOR TESTING PURPOSES ONLY SKIPS OVER LASER_CALLBACK, remove everything after this after testing: #########################




	// Use the robot's current position and orientation from tf2 buffer
	// the scan_data, and scan_angle to calculate the global position for the marker
	double global_x, global_y = 0;
	double scan_data_fake = 5.5;
	
	// Uses pointers to modify the local ^ global_x and global_y?? 
	calculate_global_position(scan_data_fake, scan_angle, global_x, global_y);

	std::cout << "New global_x and global_y are: x: " << global_x << ", y: " << global_y << std::endl;
	std::cout << "Color1 = " << color1 << ", Color2 = " << color2 << std::endl;

	// Now we can publish the marker
	publish_marker(global_x, global_y, 0 /* assuming flat ground */, color1, color2);

}

std::vector<std::string> MarkerPlacer::split(const std::string &s, char delimiter) {
	std::vector<std::string> tokens;
	std::string token;
	std::istringstream tokenStream(s);
	while (std::getline(tokenStream, token, delimiter)) {
	    tokens.push_back(token);
	}
	return tokens;
}

void MarkerPlacer::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    	RCLCPP_INFO(this->get_logger(), "Laser callback triggered");
	if (awaiting_scan_data) {
		if (std::isinf(msg->ranges.at(scan_angle))) {
			scan_data = msg->range_max; // TODO: Make this just return and break out of function? Prevent placing marker incorrectly
		} else {
			scan_data = msg->ranges.at(scan_angle);
		}

		// Use the robot's current position and orientation from tf2 buffer
		// the scan_data, and scan_angle to calculate the global position for the marker
		double global_x, global_y = 0;
		
		// Uses pointers to modify the local ^ global_x and global_y?? 
		calculate_global_position(scan_data, scan_angle, global_x, global_y);

		// Now we can publish the marker
		publish_marker(global_x, global_y, 0 /* assuming flat ground */, color1, color2);

		awaiting_scan_data = false;  // Reset the flag
	}
}

void MarkerPlacer::publish_marker(double x, double y, double z, const std::string& color1, const std::string& color2) {
    // Publish the first cylinder marker (color2)
    auto marker1 = create_cylinder("map", marker_count++, x, y, z, color2);
    marker_pub->publish(marker1);

    // Publish the second cylinder marker (color1) on top of the first
    auto marker2 = create_cylinder("map", marker_count++, x, y, z + marker1.scale.z, color1);
    marker_pub->publish(marker2);

    RCLCPP_INFO(this->get_logger(), "Publishing markers at (%.2f, %.2f, %.2f)", x, y, z);

    // Set the parameter that other nodes can check to allow the robot to start moving again.
    this->set_parameter(rclcpp::Parameter("robot_should_stop", false)); // Using paramaters
    RCLCPP_INFO(this->get_logger(), "Markers Published all DONE. Robot start signal sent.");
}

// Helper function to set color based on the color name
void MarkerPlacer::set_color(visualization_msgs::msg::Marker& marker, const std::string& color_name) {
    if (color_name == "pink") {
	marker.color.r = 1.0;
	marker.color.g = 0.75;
	marker.color.b = 0.8;
    } else if (color_name == "blue") {
	marker.color.b = 1.0;
    } else if (color_name == "green") {
	marker.color.g = 1.0;
    } else if (color_name == "yellow") {
	marker.color.r = 1.0;
	marker.color.g = 1.0;
    }
    marker.color.a = 1.0; // Don't forget to set the alpha!
}

// Helper function to create a cylinder marker
visualization_msgs::msg::Marker MarkerPlacer::create_cylinder(const std::string& frame_id, int marker_id, double x, double y, double z, const std::string& color) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = this->now();
    marker.ns = "cylinder";
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1; // Cylinder diameter
    marker.scale.y = 0.1; // Cylinder diameter
    marker.scale.z = 0.2; // Cylinder height
    set_color(marker, color);
    return marker;
}

void MarkerPlacer::calculate_global_position(double distance, int angle, double& global_x, double& global_y) {
	geometry_msgs::msg::PointStamped local_point;
	local_point.header.frame_id = "base_link";
	local_point.header.stamp = this->now() - rclcpp::Duration::from_seconds(0.1);

	// Convert the scan angle and distance to a point in the robot's frame
	double rad_angle = angle * (M_PI / 180.0);
	local_point.point.x = distance * cos(rad_angle);
	local_point.point.y = distance * sin(rad_angle);
	local_point.point.z = 0.0;  // Assuming ground plane


	// Transform this point to the global frame
	geometry_msgs::msg::PointStamped global_point;
	global_point = tf_buffer->transform(local_point, "map", tf2::durationFromSec(0.1));

	global_x = global_point.point.x;
	global_y = global_point.point.y;
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MarkerPlacer>();
    RCLCPP_INFO(node->get_logger(), "Spinning MarkerPlacer Node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

