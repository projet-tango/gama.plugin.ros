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

	// RViz will not draw anything whose frame it cannot resolve.
	// So GAMA publishes the transform that makes its world a frame of the ROS world
	ros_publisher tf_out <- ros_publisher(bridge, ros_topic("/tf", "tf2_msgs/msg/TFMessage"));

	// visualization_msgs/Marker declares these as constants; GAML cannot reach Java constants, so they
	// are repeated here. They are part of the message definition and will not change.
	int marker_arrow <- 0;
	int marker_line_strip <- 4;
	int marker_text <- 9;
	int action_add <- 0;       
	int action_deleteall <- 3;

	// The frame GAMA's coordinates are expressed in, placed in "map" by the transform below
	string gama_frame <- "gama_world";

	list<rgb> palette <- [#steelblue, #tomato, #seagreen, #orange, #mediumpurple, #teal];

	// returns a timestamp for ros. It can only move forward
	map<string, unknown> ros_stamp (){
		return ["sec"::int(gama.machine_time/1000), "nanosec"::int((gama.machine_time mod 1000) * 1000000)];
	}

	init {
		// Created one by one so that each agent gets a marker id that is stable for the whole run
		loop i from: 0 to: nb_agents - 1 {
			create wanderer {
				marker_id <- i;
				colour <- palette[i mod length(palette)];
			}
		}

		// A previous run may have left markers on the topic with ids this one will not reuse. One
		// DELETEALL clears the display rather than leaving ghosts behind.
		bool cleared <- ros_publish(markers_out, ["markers"::[[
			"header"::["frameId"::gama_frame],
			"ns"::"agents",
			"id"::0,
			"action"::action_deleteall
		]]]);
	}

	// RViz resolves every marker against its fixed frame, so that frame has to exist in TF -- with no
	// TF at all, "map" is an unknown frame and nothing is drawn. This one transform does two jobs: it
	// makes "map" exist, and it places GAMA's corner origin at the centre of the RViz grid, so no
	// coordinate anywhere in this model has to be converted.
	reflex place_the_world {
		bool sent <- ros_publish(tf_out, ["transforms"::[[
			"header"::["frameId"::"map", "stamp"::ros_stamp()],
			"childFrameId"::gama_frame,
			"transform"::[
				"translation"::["x"::-(world_size / 2), "y"::-(world_size / 2), "z"::0.0],
				"rotation"::["w"::1.0]   // identity: the two frames differ by a translation only
			]
		]]]);
	}

	reflex draw_in_rviz {
		map<string, unknown> header <- ["frameId"::gama_frame, "stamp"::ros_stamp()];
		// Half a second: long enough to survive between two cycles, short enough that the markers fade
		// out on their own when GAMA stops rather than freezing on screen forever
		map<string, unknown> lifetime <- ["sec"::0, "nanosec"::500000000];

		// One arrow per agent. An ARROW points along the +x of its own pose, so the heading is carried
		// by the quaternion and nothing needs to be drawn by hand.
		list<map<string, unknown>> batch <- wanderer collect ([
			"header"::header,
			"ns"::"agents",
			"id"::each.marker_id,
			"type"::marker_arrow,
			"action"::action_add,
			"pose"::[
				"position"::["x"::world_size-each.location.x, "y"::each.location.y, "z"::0.3],
				// a rotation of 'heading' around z, in the (0, 0, sin(a/2), cos(a/2)) form GAMA's
				// degree-based cos and sin write directly
				"orientation"::["z"::cos(each.heading / 2), "w"::sin(each.heading / 2)]
			],
			"scale"::["x"::1.6, "y"::0.5, "z"::0.5],   // length, width, height of the arrow
			// std_msgs/ColorRGBA is four floats in 0..1, a GAML rgb is three ints in 0..255
			"color"::["r"::each.colour.red / 255.0, "g"::each.colour.green / 255.0,
				"b"::each.colour.blue / 255.0, "a"::1.0],
			"lifetime"::lifetime
		]);

		// The boundary of the GAMA world, as a closed LINE_STRIP. Its 'points' are a list of maps,
		// which is how any ROS sequence of sub-messages is written.
		batch << [
			"header"::header,
			"ns"::"arena",
			"id"::0,
			"type"::marker_line_strip,
			"action"::action_add,
			"pose"::["orientation"::["w"::1.0]],
			"scale"::["x"::0.15],                      // for a line strip, scale.x is the line width
			"color"::["r"::0.6, "g"::0.6, "b"::0.6, "a"::1.0],
			"points"::[
				["x"::0.0, "y"::0.0, "z"::0.0],
				["x"::world_size, "y"::0.0, "z"::0.0],
				["x"::world_size, "y"::world_size, "z"::0.0],
				["x"::0.0, "y"::world_size, "z"::0.0],
				["x"::0.0, "y"::0.0, "z"::0.0]
			],
			"lifetime"::lifetime
		];

		// A label, so the RViz window says which cycle it is showing
		batch << [
			"header"::header,
			"ns"::"status",
			"id"::0,
			"type"::marker_text,
			"action"::action_add,
			"pose"::[
				"position"::["x"::world_size / 2, "y"::world_size + 2.0, "z"::1.0],
				"orientation"::["w"::1.0]
			],
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
