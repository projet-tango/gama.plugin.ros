/**
* Name: Messages are maps
* Author: Baptiste Lesquoy
* Description: How a ROS message and a GAML map correspond. Everything here is a loopback, so the model
* 				runs on its own.
*
*            	The rule: the keys of the map are the field names of the message type, a nested message
*              	is a nested map, a sequence is a list, and a field left out of the map keeps its ROS
*              	default. Reading gives back exactly that shape.
* Tags: ros, ros2, network
*/
model ros_messages_are_maps

global {

	ros_node bridge <- ros_node("gama_maps");

	// A flat message: two ints, and nothing nested
	ros_topic time_topic <- ros_topic("/demo/time", "builtin_interfaces/msg/Time");

	// A nested one: a Twist holds two Vector3, each holding three floats
	ros_topic twist_topic <- ros_topic("/demo/twist", "geometry_msgs/msg/Twist");

	// One with a sequence and a header: a LaserScan holds a float sequence of ranges
	ros_topic scan_topic <- ros_topic("/demo/scan", "sensor_msgs/msg/LaserScan");

	ros_publisher time_out <- ros_publisher(bridge, time_topic);
	ros_publisher twist_out <- ros_publisher(bridge, twist_topic);
	ros_publisher scan_out <- ros_publisher(bridge, scan_topic);

	ros_subscription time_in <- ros_subscription(bridge, time_topic);
	ros_subscription twist_in <- ros_subscription(bridge, twist_topic);
	ros_subscription scan_in <- ros_subscription(bridge, scan_topic);

	init {
		// ros_fields lists the first level of a message type. A nested message counts as one field:
		// its own fields show up in the map that reading returns.
		write "===== what the types look like =====";
		write "Time  : " + ros_fields(time_topic);   // [nanosec, sec]
		write "Twist : " + ros_fields(twist_topic);  // [angular, linear]
		write "Scan  : " + ros_fields(scan_topic);   // [angleIncrement, angleMax, ..., ranges, ...]

		bool ready <- ros_wait_for_subscriber(twist_out, 2000);
		write "\nloopback up : " + ready;
	}

	reflex publish_everything when: cycle = 1 {
		write "\n===== flat: every field given =====";
		bool a <- ros_publish(time_out, ["sec"::42, "nanosec"::500000000]);

		write "===== nested: a map per sub-message, and only what is set =====";
		// A robot going forward at 1 m/s and turning left. The five other components of the Twist are
		// not named, so they stay at 0.0 -- this is the usual way of writing a command.
		bool b <- ros_publish(twist_out, ["linear"::["x"::1.0], "angular"::["z"::0.5]]);

		write "===== sequences are lists, fixed-size arrays are lists of the right length =====";
		bool c <- ros_publish(scan_out, [
			"header"::["frameId"::"laser", "stamp"::["sec"::42, "nanosec"::0]],
			"angleMin"::-1.57, "angleMax"::1.57, "angleIncrement"::0.785,
			"rangeMin"::0.1, "rangeMax"::10.0,
			"ranges"::[2.0, 2.5, 3.0, 2.5, 2.0]
		]);
	}

	reflex read_everything when: cycle = 3 {
		write "\n===== and back, same shape =====";
		write "Time  : " + ros_read_latest(time_in);
		write "Twist : " + ros_read_latest(twist_in);

		map<string, unknown> scan <- ros_read_latest(scan_in);
		write "Scan  : " + scan;

		// Reading gives GAML types: floats for float32 and float64, ints for the integer widths,
		// strings for strings, so the values go straight into the rest of the model.
		if scan != nil {
			list<float> ranges <- list<float>(scan["ranges"]);
			write "\nnearest obstacle : " + min(ranges) + " m";
			write "frame            : " + map<string, unknown>(scan["header"])["frameId"];
		}
	}

	// Publishing a key that is not a field of the type, or a value that does not fit it, is an error
	// naming the field rather than a message quietly sent with default values. Uncomment to see:
	//
	//   'linaer' is not a field of the ROS2 message geometry_msgs.Twist. Its fields are [angular, linear].
	//   The field 'x' expects a double, but got fast (String)
	//
	reflex mistakes when: cycle = 5 {
		// bool typo <- ros_publish(twist_out, ["linaer"::["x"::1.0]]);
		// bool wrong <- ros_publish(twist_out, ["linear"::["x"::"fast"]]);
		write "\nsee the commented lines of the 'mistakes' reflex for what a wrong map reports";
		do pause();
	}
}

experiment maps type: gui { }
