/**
* Name: Conversions, profiles and skill
* Author: Arthur Brugière
* Description: Unit tests of the ROS2 plugin that need no ROS installation and no robot: the conversions
*              between GAMA's frame and ROS's, the robot profiles, the claim that ros_moving is a drop-in
*              replacement of moving, and how an embodied robot is configured. Run the 'tests'
*              experiment; every assertion is reported in the test view.
* Tags: test, ros, ros2, skill
*/
model ros2_conversions_profiles_skill

global {

	float world_size <- 20.0;
	geometry shape <- square(world_size);

	init {
		create checker;
		create plain {
			location <- {5.0, 5.0};
			heading <- 0.0;
		}
		create mirrored {
			location <- {5.0, 5.0};
			heading <- 0.0;
		}
		// An embodied AlphaBot2: no ROS installation is needed to create its topics, only to reach a robot
		create robot with: [ros_name::"ab", ros_profile::"alphabot2"];
		create robot with: [ros_name::"", ros_profile::"alphabot2"];
	}
}

species plain skills: [moving] {}

species mirrored skills: [ros_moving] {}

species robot skills: [ros_moving] {}

species checker {

	bool close (float a, float b) {
		return abs(a - b) < 1.0e-6;
	}

	bool same (point a, point b) {
		return a distance_to b < 1.0e-6;
	}

	// --- GAMA <-> ROS --------------------------------------------------------------------------------------

	test centre_of_world_is_ros_origin {
		map<string, unknown> p <- ros_point(world.location);
		assert close(float(p["x"]), 0.0) and close(float(p["y"]), 0.0);
	}

	test top_of_screen_is_north {
		// GAMA's y points down the screen, ROS's y points north: the top edge is at +y in ROS
		map<string, unknown> p <- ros_point({world_size / 2, 0.0});
		assert close(float(p["x"]), 0.0);
		assert close(float(p["y"]), world_size / 2);
	}

	test location_round_trip {
		assert same(ros_location(ros_point({3.0, 7.0})), {3.0, 7.0});
	}

	test heading_round_trip {
		loop h over: [0.0, 45.0, 90.0, 135.0, 180.0, 270.0, 359.0] {
			assert close(ros_heading(ros_orientation(h)), h);
		}
	}

	test clockwise_heading_is_negative_yaw {
		// heading 90 points down the screen, i.e. south: a yaw of -90 degrees, so a negative z in the quaternion
		assert float(ros_orientation(90.0)["z"]) < 0;
	}

	test twist_units_and_direction {
		map<string, unknown> t <- ros_twist(1.0, 90.0);
		assert close(float(map(t["linear"])["x"]), 1.0);
		assert close(float(map(t["angular"])["z"]), -#pi / 2);
		assert close(ros_speed(t), 1.0);
		assert close(ros_turn_rate(t), 90.0);
	}

	test readers_dig_into_odometry {
		map<string, unknown> odom <- [
			"pose"::["pose"::ros_pose({1.0, 2.0}, 30.0)],
			"twist"::["twist"::ros_twist(0.4, 20.0)]
		];
		assert same(ros_location(odom), {1.0, 2.0});
		assert close(ros_heading(odom), 30.0);
		assert close(ros_speed(odom), 0.4);
		assert close(ros_turn_rate(odom), 20.0);
	}

	test world_frame_placement {
		string frame <- ros_world_frame("world", {10.0, 5.0});
		assert frame = "world";
		map<string, unknown> p <- ros_point(world.location);
		bool placed <- close(float(p["x"]), 10.0) and close(float(p["y"]), 5.0);
		bool back <- same(ros_location(["x"::10.0, "y"::5.0]), world.location);
		frame <- ros_world_frame("map");   // put the default back for the other tests
		assert placed;
		assert back;
	}

	test nil_message_is_an_error {
		bool raised <- false;
		try {
			point p <- ros_location(map(nil));
		} catch {
			raised <- true;
		}
		assert raised;
	}

	// --- Profiles --------------------------------------------------------------------------------------------

	test built_in_profiles {
		assert float(ros_profile("alphabot2")["max_speed"]) = 0.3;
		assert string(ros_profile("alphabot2")["odom_topic"]) = "";
		assert string(ros_profile("gazebo")["cmd_topic"]) = "/model/{name}/cmd_vel";
		assert float(ros_profile("generic")["max_speed"]) = 0.5;
	}

	test derived_profile {
		assert ros_define_profile("slow_ab", "alphabot2", ["max_speed"::0.1]) = "slow_ab";
		assert float(ros_profile("slow_ab")["max_speed"]) = 0.1;
		assert string(ros_profile("slow_ab")["cmd_topic"]) = string(ros_profile("alphabot2")["cmd_topic"]);
		assert float(ros_profile("alphabot2")["max_speed"]) = 0.3;   // the base is left alone
	}

	test misspelt_setting_is_an_error {
		bool raised <- false;
		try {
			string p <- ros_define_profile("bad", ["max_sped"::1.0]);
		} catch {
			raised <- true;
		}
		assert raised;
	}

	test unknown_profile_is_an_error {
		bool raised <- false;
		try {
			map<string, unknown> p <- ros_profile("turtlebot");
		} catch {
			raised <- true;
		}
		assert raised;
	}

	// --- ros_moving is a drop-in replacement of moving -----------------------------------------------------------

	test mirror_move_is_moving_move {
		ask plain { do move speed: 1.0 heading: 45.0; }
		ask mirrored { do move speed: 1.0 heading: 45.0; }
		assert same(first(plain).location, first(mirrored).location);
		assert close(first(plain).heading, first(mirrored).heading);
	}

	test mirror_goto_is_moving_goto {
		ask plain { do goto target: {15.0, 12.0} speed: 2.0; }
		ask mirrored { do goto target: {15.0, 12.0} speed: 2.0; }
		assert same(first(plain).location, first(mirrored).location);
	}

	test mirror_binds_no_robot {
		assert !first(mirrored).ros_estimated;
		assert first(mirrored).ros_cmd_publisher = nil;
	}

	// --- An embodied robot --------------------------------------------------------------------------------------

	test alphabot2_topics_and_estimated_pose {
		robot ab <- robot first_with (each.ros_name = "ab");
		assert string(ab.ros_config["cmd_topic"]) = "/ab/cmd_vel";
		assert ab.ros_estimated;
		assert ab.ros_cmd_publisher != nil;
		assert ab.ros_pose_subscription = nil;
		assert ab.ros_odom_subscription = nil;
	}

	test empty_name_means_no_namespace {
		robot bare <- robot first_with (each.ros_name = "");
		assert string(bare.ros_config["cmd_topic"]) = "/cmd_vel";
	}

	test settings_override_the_profile {
		robot ab <- robot first_with (each.ros_name = "ab");
		ask ab { ros_settings <- ["max_speed"::0.12]; }
		assert float(ab.ros_config["max_speed"]) = 0.12;
		assert float(ab.ros_config["max_turn_rate"]) = float(ros_profile("alphabot2")["max_turn_rate"]);
		ask ab { ros_settings <- nil; }
	}

	test misspelt_agent_setting_is_an_error {
		robot ab <- robot first_with (each.ros_name = "ab");
		ask ab { ros_settings <- ["max_sped"::0.12]; }
		bool raised <- false;
		try {
			map<string, unknown> c <- ab.ros_config;
		} catch {
			raised <- true;
		}
		ask ab { ros_settings <- nil; }
		assert raised;
	}
}

experiment tests type: test autorun: true until: cycle = 1 {}
