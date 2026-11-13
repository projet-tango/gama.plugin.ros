/**
* Name: Skill - a robot of its own
* Author: Arthur Brugière
* Description: The ros_moving skill driving a robot, with nothing installed: GAMA plays the robot too.
*
*              Two species, two nodes, real DDS traffic between them, as in model 03:
*              - 'fake_base' is what a robot driver does, written with the raw operators of models 01 to
*                04: it obeys /fake/cmd_vel with some inertia, and reports /fake/odom.
*              - 'pilot' has the ros_moving skill and ros_name "fake". Its profile, 'generic', expects
*                {ns}/cmd_vel and {ns}/odom: nothing else is configured. `do goto` sends commands,
*                and location, heading and real_speed are what came back on /fake/odom.
*
*              Delete fake_base and set ros_name to the namespace of a real robot, and the pilot drives
*              it. To watch the traffic from outside:
*                  ros2 topic echo /fake/cmd_vel
*                  ros2 topic echo /fake/odom --field pose.pose.position
* Tags: ros, ros2, skill, network, display
*/
model skill_a_robot_of_its_own

global {

	float step <- 0.1 #s;
	float world_size <- 20.0;
	geometry shape <- square(world_size);

	// The node of the fake robot. The pilot needs none: the skill uses the node named by its profile.
	ros_node robot_node <- ros_node("fake_base");

	init {
		create fake_base;
		create pilot(ros_name:"fake");
	}
}

/**
 * The robot. Nothing here uses the skill: this is the other side of the network, which in a real setup is the
 * driver of the base, or Gazebo.
 */
species fake_base {

	point position <- {world_size / 2, world_size / 2};
	float yaw <- 0.0;          // degrees, GAMA's convention
	float speed <- 0.0;        // m/s, what the wheels do
	float turn_rate <- 0.0;    // degrees/s
	float asked_speed <- 0.0;
	float asked_turn <- 0.0;

	ros_subscription commands <- ros_subscription(robot_node,
		ros_topic("/fake/cmd_vel", "geometry_msgs/msg/Twist", "reliable"), 1);
	ros_publisher odom <- ros_publisher(robot_node, ros_topic("/fake/odom", "nav_msgs/msg/Odometry"));

	reflex obey {
		map<string, unknown> cmd <- ros_read_latest(commands);
		if cmd != nil {
			asked_speed <- ros_speed(cmd);
			asked_turn <- ros_turn_rate(cmd);
		}
		// A first-order lag: a base does not jump to the speed it is asked, which is what makes
		// "commanded" and "measured" differ in the chart below
		speed <- speed + (asked_speed - speed) * 0.15;
		turn_rate <- turn_rate + (asked_turn - turn_rate) * 0.3;
		yaw <- yaw + turn_rate * step;
		position <- position + {cos(yaw), sin(yaw)} * speed * step;
	}

	reflex report {
		bool sent <- ros_publish(odom, [
			"header"::["frameId"::"odom"],
			"childFrameId"::"base_link",
			"pose"::["pose"::ros_pose(position, yaw)],
			"twist"::["twist"::ros_twist(speed, turn_rate)]
		]);
	}
}

/**
 * The model's side. Written as if it moved with the moving skill.
 */
species pilot skills: [ros_moving] {

	point target <- {rnd(2.0, world_size - 2.0), rnd(2.0, world_size - 2.0)};
	bool driving <- true;
	list<point> travelled;

	init {
		speed <- 1.5;    // the profile caps it anyway: max_speed is 0.5 in 'generic'...
		// ...unless this one robot is allowed more. ros_settings overrides the profile, for this agent only.
		ros_settings <- ["max_speed"::1.5, "max_turn_rate"::120.0, "arrival"::0.3];
	}

	// ros_connected turns true with the first odometry message
	reflex drive when: driving and ros_connected {
		if location distance_to target < 0.5 {
			target <- {rnd(2.0, world_size - 2.0), rnd(2.0, world_size - 2.0)};
		}
		do goto(target: target);
		travelled << location;
	}

	aspect default {
		if length(travelled) > 1 { draw line(copy(travelled)) color: #lightsteelblue; }
		draw circle(0.3) at: target color: #tomato;
		draw triangle(0.8) rotate: heading + 90 color: #steelblue;
	}
}

experiment exp_drive type: gui {

	float minimum_cycle_duration <- 0.1 #s;

	// Calling no action is a stop, as with the moving skill: the skill sends one at the end of the step
	user_command "Stop / resume driving" {
		ask pilot { driving <- !driving; }
	}

	output {
		display "World" type: 2d {
			graphics "arena" { draw world.shape color: #white border: #lightgray; }
			species pilot aspect: default;
		}

		display "Commanded vs measured" type: 2d {
			chart "Forward speed (m/s)" kind: series {
				data "sent on /fake/cmd_vel" value: first(pilot).ros_cmd_speed color: #tomato;
				data "back on /fake/odom" value: first(pilot).real_speed color: #steelblue;
			}
		}

		monitor "connected" value: first(pilot).ros_connected;
		monitor "commands to" value: first(pilot).ros_config["cmd_topic"];
		monitor "pose from" value: first(pilot).ros_config["odom_topic"];
		monitor "max speed in force (m/s)" value: first(pilot).ros_config["max_speed"];
		monitor "odometry received" value: ros_received(first(pilot).ros_odom_subscription);
	}
}
