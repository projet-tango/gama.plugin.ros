/**
* Name: GAMA in RViz
* Author: Baptiste Lesquoy
* Description: Showing a GAMA simulation inside the ROS tooling. 
* 				RViz draws whatever arrives on a MarkerArray topic, so GAMA publishes one
*             	marker per agent and RViz becomes a second view of the simulation.
*
*              SETUP in one terminal with ros2 loaded, see includes/README.md:
*                  rviz2 -d /path/to/includes/gama_markers.rviz
*              Then run this experiment. With a bare `rviz2`, set Fixed Frame to "map" and add a
*              MarkerArray display on /gama/markers by hand.
* Tags: ros, ros2, rviz, network, display
*/
model gama_in_rviz

global {

	float step <- 0.05 #s;

	int nb_agents <- 40 min: 1 max: 400;

	float world_size <- 40.0;
	geometry shape <- square(world_size);

	ros_node bridge <- ros_node("gama_rviz");

	// One publisher for the whole population, not one per agent: a MarkerArray carries as many markers as we want
	ros_publisher markers_out <- ros_publisher(bridge, ros_topic("/gama/markers", "visualization_msgs/msg/MarkerArray"));

	// visualization_msgs/Marker declares these as constants; GAML cannot reach Java constants, so they
	// are repeated here. They are part of the message definition and will not change.
	int marker_arrow <- 0;
	int marker_line_strip <- 4;
	int marker_text <- 9;
	int action_add <- 0;       
	int action_deleteall <- 3;

	list<rgb> palette <- [#steelblue, #tomato, #seagreen, #orange, #mediumpurple, #teal];

	// Where the GAMA world sits in ROS. This is the default and could be left out: the centre of the world
	// at the origin of "map", which is the centre of the RViz grid. Every ros_pose and ros_point below, and
	// every header left out of a message, use it.
	string frame <- ros_world_frame("map", {0.0, 0.0});

	// RViz resolves every marker against its fixed frame, so that frame has to exist in TF -- with no TF at
	// all, "map" is an unknown frame and nothing is drawn. ros_world_tf makes it exist, as a static transform
	// to a "gama_world" frame at the centre of the world. "latched" keeps the last message for subscribers
	// that join later, which is how /tf_static works; the publisher must therefore live as long as the
	// simulation, hence a global.
	ros_publisher tf_static_out <- ros_publisher(bridge, ros_topic("/tf_static", "tf2_msgs/msg/TFMessage", "latched"));

	init {
		// Created one by one so that each agent gets a marker id that is stable for the whole run
		loop i from: 0 to: nb_agents - 1 {
			create wanderer {
				marker_id <- i;
				colour <- palette[i mod length(palette)];
			}
		}

		// Sent once: the frame does not move, and the latched topic hands it to an RViz started later
		bool placed <- ros_publish(tf_static_out, ros_world_tf("gama_world"));

		// A previous run may have left markers on the topic with ids this one will not reuse. One
		// DELETEALL clears the display rather than leaving ghosts behind.
		bool cleared <- ros_publish(markers_out, ["markers"::[[
			"ns"::"agents",
			"id"::0,
			"action"::action_deleteall
		]]]);
	}

	reflex draw_in_rviz {
		// No header anywhere below: one left out is stamped with the wall clock and given the frame of the
		// world when the message is published.
		// Half a second: long enough to survive between two cycles, short enough that the markers fade
		// out on their own when GAMA stops rather than freezing on screen forever
		map<string, unknown> lifetime <- ["sec"::0, "nanosec"::500000000];

		// One arrow per agent. An ARROW points along the +x of its own pose, so the heading is carried
		// by the quaternion of ros_pose and nothing needs to be drawn by hand.
		list<map<string, unknown>> batch <- wanderer collect ([
			"ns"::"agents",
			"id"::each.marker_id,
			"type"::marker_arrow,
			"action"::action_add,
			"pose"::ros_pose(each.location + {0, 0, 0.3}, each.heading),
			"scale"::["x"::1.6, "y"::0.5, "z"::0.5],   // length, width, height of the arrow
			// std_msgs/ColorRGBA is four floats in 0..1: a GAML rgb is converted when published
			"color"::each.colour,
			"lifetime"::lifetime
		]);

		// The boundary of the GAMA world, as a closed LINE_STRIP. Its 'points' are a list of maps, which is
		// how any ROS sequence of sub-messages is written, and ros_point turns each GAMA corner into one.
		batch << [
			"ns"::"arena",
			"id"::0,
			"type"::marker_line_strip,
			"action"::action_add,
			"pose"::["orientation"::["w"::1.0]],
			"scale"::["x"::0.15],                      // for a line strip, scale.x is the line width
			"color"::["r"::0.6, "g"::0.6, "b"::0.6, "a"::1.0],
			"points"::[{0, 0}, {world_size, 0}, {world_size, world_size}, {0, world_size}, {0, 0}]
				collect ros_point(each),
			"lifetime"::lifetime
		];

		// A label, so the RViz window says which cycle it is showing
		batch << [
			"ns"::"status",
			"id"::0,
			"type"::marker_text,
			"action"::action_add,
			// just above the top edge of the GAMA display, which is north in ROS
			"pose"::ros_pose({world_size / 2, -2.0, 1.0}, 0.0),
			"scale"::["z"::1.5],                       // for text, scale.z is the character height
			"color"::["r"::0.2, "g"::0.2, "b"::0.2, "a"::1.0],
			"text"::"GAMA cycle " + cycle + " -- " + length(wanderer) + " agents",
			"lifetime"::lifetime
		];

		bool sent <- ros_publish(markers_out, ["markers"::batch]);
	}

	// Saying out loud whether anything is listening: publishing to nobody is not an error, and an RViz
	// that was never given the topic looks exactly like a working one from here.
	reflex watch_rviz when: cycle mod 40 = 0 {
		write ros_subscriber_count(markers_out) > 0
			? "RViz is subscribed to /gama/markers"
			: "Nobody is subscribed to /gama/markers. Is rviz2 running, with a MarkerArray display on that topic and Fixed Frame set to map?";
	}
}

species wanderer skills: [moving] {

	// The marker id has to be stable across cycles: RViz replaces a marker when the same ns and id come
	// back, and would otherwise accumulate one marker per agent per cycle.
	int marker_id;
	rgb colour;

	init {
		speed <- rnd(0.4, 1.0);
	}

	reflex move {
		do wander(amplitude: 10.0);
	}

	aspect default {
		draw triangle(0.9) rotate: heading + 90 color: colour;
	}
}

experiment rviz type: gui {

	parameter "Number of agents" var: nb_agents category: "Population";

	// Not needed for correctness here, unlike the Gazebo version: a marker carries a pose, so the two
	// views cannot drift apart however fast GAMA runs. It only keeps the RViz window watchable.
	float minimum_cycle_duration <- 0.05 #s;

	output {
		display "What GAMA simulates (RViz shows the same)" type: 2d {
			graphics "arena" {
				draw world.shape color: #white border: #lightgray;
			}
			species wanderer aspect: default;
		}

		monitor "subscribers on /gama/markers" value: ros_subscriber_count(markers_out);
		monitor "markers published per cycle" value: length(wanderer) + 2;
	}
}
