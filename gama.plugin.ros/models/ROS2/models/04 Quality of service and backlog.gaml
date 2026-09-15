/**
* Name: Quality of service and backlog
* Author: Baptiste Lesquoy
* Description: The two things that make a bridge silent or late -- a quality of service that does not
*              match, and a queue that fills faster than the model reads it -- and the operators that
*              tell you which of the two is happening. Self-contained: GAMA plays the fast sensor.
*
*              A ROS driver publishing scans or images almost always uses best effort, and a
*              subscription asking for reliable simply never matches it: no error, no message, just
*              silence. ros_received staying at 0 is how that shows up.
* Tags: ros, ros2, network
*/
model ros_qos_and_backlog

global {

	float step <- 1.0 #s;

	ros_node sensor_node <- ros_node("gama_sensor");
	ros_node reader_node <- ros_node("gama_reader");

	// The same topic name, declared three times with three qualities of service. Only a subscription
	// whose QoS is compatible with the publisher's receives anything.
	ros_topic scan_be <- ros_topic("/scan", "sensor_msgs/msg/LaserScan", "best_effort");
	ros_topic scan_reliable <- ros_topic("/scan", "sensor_msgs/msg/LaserScan", "reliable");

	// The sensor: best effort, like a real lidar driver
	ros_publisher sensor <- ros_publisher(sensor_node, scan_be);

	// Three ways of listening to it
	ros_subscription matching <- ros_subscription(reader_node, scan_be, 1);      // best effort, keep last
	ros_subscription backlog <- ros_subscription(reader_node, scan_be, 200);     // best effort, keep 200
	ros_subscription mismatched <- ros_subscription(reader_node, scan_reliable); // reliable: never matches

	init {
		write "waiting for the sensor to be discovered: " + ros_wait_for_publisher(matching, 2000);
	}

	// The sensor publishes 20 scans per cycle while the model reads once per cycle: this is the usual
	// ratio between a driver and a simulation, compressed so that it shows within a few cycles.
	reflex sense {
		loop i from: 0 to: 19 {
			bool sent <- ros_publish(sensor, [
				"header"::["frameId"::"laser"],
				"angleMin"::-1.57, "angleMax"::1.57, "angleIncrement"::0.785,
				"rangeMin"::0.1, "rangeMax"::10.0,
				"ranges"::[1.0 + i, 2.0 + i, 3.0 + i, 4.0 + i, 5.0 + i]
			]);
		}
	}

	reflex observe {
		write "\n===== cycle " + cycle + " =====";

		// A queue of 1 with ros_read_latest: always the freshest scan, never a backlog, and the count of
		// dropped messages is exactly the ones the model chose not to look at. This is what a sensor
		// should be read with.
		map<string, unknown> fresh <- ros_read_latest(matching);
		write "freshest scan  : " + (fresh = nil ? "nothing yet" : "ranges " + fresh["ranges"]);
		write "  received " + ros_received(matching) + ", dropped " + ros_dropped(matching)
			+ ", waiting " + ros_queue_size(matching);

		// A large queue with ros_read: nothing is lost until the queue fills, and then the oldest go.
		// This is what an event topic should be read with -- and the moment ros_dropped starts moving
		// is the moment the model has stopped keeping up.
		list<map<string, unknown>> events <- ros_read_all(backlog);
		write "backlog drained: " + length(events) + " scans at once";
		write "  received " + ros_received(backlog) + ", dropped " + ros_dropped(backlog);

		// The one that will never see anything, because reliable does not match best effort
		write "mismatched QoS : received " + ros_received(mismatched)
			+ (ros_received(mismatched) = 0 ? "  <- silence, not an error" : "");
	}

	reflex stop when: cycle = 5 {
		// Closing early is optional: the experiment closes every node when it is disposed. It is worth
		// doing to leave the domain before the end of a long run, or to give a node up so that naming it
		// again builds a fresh one.
		write "\nclosed the mismatched subscription : " + ros_close(mismatched);
		write "closed the sensor node             : " + ros_close(sensor_node);
		write "nodes closed in total              : " + ros_close_all("");
		do pause();
	}
}

experiment qos type: gui { }
