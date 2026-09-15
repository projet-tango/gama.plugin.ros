/**
* Name: Hello ROS
* Author: Baptiste Lesquoy
* Description: The whole plugin in one model, it doesn't need anythng installed: GAMA
*              publishes on /chatter and subscribes to it at the same time, so the messages come back
*              through the DDS network without any other ROS process being involved.
*
*              Run it as it is to check that the network works. Then, with ROS2 installed, open a
*              terminal and watch the same messages arrive:
*                  ros2 topic echo /chatter
*              or send some yourself and watch GAMA print them:
*                  ros2 topic pub /chatter std_msgs/msg/String "{data: 'from the terminal'}"
* Tags: ros, ros2, network
*/
model hello_ros

global {

	// A node is a participant on the network. It belongs to the experiment: naming it again anywhere in
	// the model gives back this very node, and it is closed when the experiment is disposed, so nothing
	// is left advertised on the domain after the run.
	ros_node bridge <- ros_node("gama");

	// A topic is a name plus the type carried on it. Nothing is opened here, and the type is checked at
	// once: mistyping "std_msgs/msg/Strong" fails on this line rather than at the first publication.
	// Both spellings work: "std_msgs/msg/String" and "std_msgs.String_".
	ros_topic chatter <- ros_topic("/chatter", "std_msgs/msg/String");

	// A publisher and a subscription are network resources owned by the node. Create them once, here,
	// and keep them: one created inside a reflex would spend its whole life discovering its peers.
	ros_publisher talker   <- ros_publisher(bridge, chatter);
	ros_subscription listener <- ros_subscription(bridge, chatter);

	init {
		// Discovery is asynchronous, and anything published before a subscriber is matched is dropped.
		// A model that publishes continuously does not care; one that sends a handful of messages does.
		bool ready <- ros_wait_for_subscriber(talker, 2000);
		write ready ? "the loopback is up" : "nobody subscribed to /chatter -- check ROS_DOMAIN_ID";

		// The fields of the message type are the keys of the map, and ros_fields lists them
		write "a std_msgs/String has the fields " + ros_fields(chatter);
	}

	// ros_publish is an operator, not a statement, so it is used in an assignment. What it returns is
	// always true: a failure raises an error instead.
	reflex talk {
		bool sent <- ros_publish(talker, ["data"::"hello from cycle " + cycle]);
	}

	// The subscription buffers on its own thread whatever arrived since the last cycle. ros_read_all
	// takes the whole backlog, which is what a model that must not miss a message wants.
	reflex listen {
		loop msg over: ros_read_all(listener) {
			write "cycle " + cycle + " received: " + msg["data"];
		}
	}

	reflex report when: cycle = 20 {
		write "\n" + ros_received(listener) + " messages received, " + ros_dropped(listener) + " dropped";
		do pause();
	}
}

experiment hello type: gui { }
