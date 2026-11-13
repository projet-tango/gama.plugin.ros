/**
* Name: Skill loopback
* Author: Arthur Brugière
* Description: Integration test of the ros_moving skill in embodied mode, with no ROS installation: GAMA
*              plays a robot base with the raw operators (it obeys /loop/cmd_vel and reports /loop/odom,
*              with inertia), and an agent with the skill drives it to a target over the DDS network.
*              The checks run at every step; each one is strict only during its phase:
*                  0-250    drive to a target, and get there
*                  270      drive again, towards a far target
*                  280      stop calling actions while moving: the skill must send a stop by itself
*                  290      drive again
*                  295      the pilot dies while moving: the skill must stop the base
*              Run the 'loopback' experiment.
* Tags: test, ros, ros2, skill, network
*/
model ros2_skill_loopback

global {

	float step <- 0.1 #s;
	float world_size <- 20.0;
	geometry shape <- square(world_size);

	point target <- {15.0, 6.0};
	bool driving <- true;

	reflex phases {
		if cycle = 270 { target <- {2.0, 18.0}; }
		if cycle = 280 { driving <- false; }
		if cycle = 290 { driving <- true; }
		if cycle = 295 { ask pilot { do die; } }
	}

	ros_node base_node <- ros_node("loop_base");

	init {
		create base;
		create pilot with: [ros_name::"loop", location::{world_size / 2, world_size / 2}];
		create checker;
	}
}

species base {

	point position <- {world_size / 2, world_size / 2};
	float yaw <- 0.0;
	float speed <- 0.0;
	float turn_rate <- 0.0;
	float asked_speed <- 0.0;
	float asked_turn <- 0.0;

	ros_subscription commands <- ros_subscription(base_node, ros_topic("/loop/cmd_vel", "geometry_msgs/msg/Twist", "reliable"), 1);
	ros_publisher odom <- ros_publisher(base_node, ros_topic("/loop/odom", "nav_msgs/msg/Odometry"));

	reflex obey {
		map<string, unknown> cmd <- ros_read_latest(commands);
		if cmd != nil {
			asked_speed <- ros_speed(cmd);
			asked_turn <- ros_turn_rate(cmd);
		}
		speed <- speed + (asked_speed - speed) * 0.3;
		turn_rate <- turn_rate + (asked_turn - turn_rate) * 0.5;
		yaw <- yaw + turn_rate * step;
		position <- position + {cos(yaw), sin(yaw)} * speed * step;
		bool sent <- ros_publish(odom, [
			"header"::["frameId"::"odom"],
			"pose"::["pose"::ros_pose(position, yaw)],
			"twist"::["twist"::ros_twist(speed, turn_rate)]
		]);
	}
}

species pilot skills: [ros_moving] {

	init {
		speed <- 1.0;
		ros_settings <- ["max_speed"::1.0, "arrival"::0.3];
	}

	reflex drive when: world.driving and ros_connected {
		do goto target: world.target;
	}
}

species checker {

	pilot p -> first(pilot);
	base b -> first(base);
	bool alive -> p != nil;

	test configured_from_the_generic_profile {
		if alive {
			assert string(p.ros_config["cmd_topic"]) = "/loop/cmd_vel";
			assert string(p.ros_config["odom_topic"]) = "/loop/odom";
			assert !p.ros_estimated;
		}
	}

	test commands_reach_the_base {
		if cycle >= 20 and alive { assert ros_subscriber_count(p.ros_cmd_publisher) > 0; }
	}

	test pose_comes_back {
		if cycle >= 20 and alive {
			assert p.ros_connected;
			// a step of lag at most: the pose is read at the end of the step it was published in, or the next
			assert p.location distance_to b.position < 0.5;
		}
	}

	test speed_is_capped_by_the_settings {
		if alive { assert p.ros_cmd_speed <= 1.0 + 1.0e-9; }
	}

	test reaches_the_target {
		if cycle >= 250 and cycle < 270 { assert b.position distance_to world.target < 0.6; }
	}

	test moving_towards_the_far_target {
		if (cycle >= 274 and cycle < 280) or (cycle >= 293 and cycle < 295) {
			assert b.asked_speed > 0.0 or abs(b.asked_turn) > 0.0;
		}
	}

	test no_action_is_a_stop {
		if cycle >= 284 and cycle < 290 { assert abs(b.asked_speed) < 1.0e-9 and abs(b.asked_turn) < 1.0e-9; }
	}

	test death_is_a_stop {
		if cycle >= 298 { assert abs(b.asked_speed) < 1.0e-9 and abs(b.asked_turn) < 1.0e-9; }
	}
}

experiment loopback type: test until: cycle = 300 {
	// DDS is asynchronous: give each command a moment to arrive
	float minimum_cycle_duration <- 0.02 #s;
}
