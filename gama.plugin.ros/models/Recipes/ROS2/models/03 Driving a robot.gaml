/**
* Name: Driving a robot
* Author: Baptiste Lesquoy
* Description: Commands out on /cmd_vel, state in on /odom -- written so that it runs on its own: 
*				GAMA plays both parts, a controller and a robot, on two separate
*              	nodes talking over the real DDS network.
*
*              To drive a real one instead, delete the 'robot' species, keep the controller, and
*              point it at the robot's node: nothing else changes. To watch the traffic from outside:
*                  ros2 topic echo /odom
*                  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
* Tags: ros, ros2, network, display
*/
model driving_a_robot

global {

	float step <- 0.1 #s;
	float world_size <- 20.0;
	geometry shape <- envelope(square(world_size));

	// Two nodes on purpose: this is what two processes would look like, and DDS does not care that they
	// happen to be in the same JVM. Both are closed when the experiment is disposed.
	ros_node controller_node <- ros_node("gama_controller");
	ros_node robot_node <- ros_node("gama_robot");

	// The two standard topics of a ROS2 mobile base
	ros_topic cmd_vel <- ros_topic("/cmd_vel", "geometry_msgs/msg/Twist");
	ros_topic odom <- ros_topic("/odom", "nav_msgs/msg/Odometry");

	init {
		create robot;
		create controller;
	}
}

/**
 * The robot: reads commands, integrates them, and reports where it ended up. In a real setup this whole
 * species is replaced by hardware or by Gazebo, and the model does not change.
 */
species robot {

	float yaw <- 0.0;        // degrees, GAMA's convention for cos, sin and rotate: clockwise on screen
	float speed <- 0.0;      // m/s, forward
	float turn_rate <- 0.0;  // degrees/s

	ros_subscription commands;
	ros_publisher odom_out;

	init {
		location <- {world_size / 2, world_size / 2};
		// A queue of 1: only the most recent command matters, older ones describe a past that is gone
		commands <- ros_subscription(robot_node, cmd_vel, 1);
		odom_out <- ros_publisher(robot_node, odom);
	}

	reflex obey {
		map<string, unknown> cmd <- ros_read_latest(commands);
		if cmd != nil {
			// ROS turns in radians per second and counter-clockwise, GAMA in degrees and clockwise on screen:
			// ros_turn_rate does both conversions, ros_speed reads linear.x
			speed <- ros_speed(cmd);
			turn_rate <- ros_turn_rate(cmd);
		}
	}

	reflex move {
		yaw <- yaw + turn_rate * step;
		location <- location + {cos(yaw) * speed * step, sin(yaw) * speed * step};
	}

	reflex report {
		// ros_pose converts a GAMA location and heading to a ROS pose: the world is placed with its centre at
		// the origin of the frame, where a real base starts its odometry, and its y axis turned the ROS way.
		// The stamp of the header is left out: it is filled with the wall-clock time when published.
		bool sent <- ros_publish(odom_out, [
			"header"::["frameId"::"odom"],
			"childFrameId"::"base_link",
			"pose"::["pose"::ros_pose(location, yaw)],
			"twist"::["twist"::ros_twist(speed, turn_rate)]
		]);
	}

	aspect default {
		draw triangle(1.2) rotate: yaw + 90 color: #steelblue;
	}
}

/**
 * The controller: the part a GAMA model would normally keep. It knows nothing of the robot except what
 * arrives on /odom, and says nothing except what it writes on /cmd_vel.
 */
species controller {

	point believed_position <- {0, 0};
	float believed_heading <- 0.0;

	// A waypoint to steer towards, moved on every arrival
	point target <- {rnd(2.0, world_size - 2.0), rnd(2.0, world_size - 2.0)};

	ros_subscription feedback;
	ros_publisher commands;

	init {
		feedback <- ros_subscription(controller_node, odom, 1);
		commands <- ros_publisher(controller_node, cmd_vel);
	}

	reflex perceive {
		map<string, unknown> msg <- ros_read_latest(feedback);
		if msg != nil {
			// nav_msgs/Odometry nests the pose three levels deep (pose.pose.position): ros_location and
			// ros_heading dig it out, and convert it back to GAMA's frame
			believed_position <- ros_location(msg);
			believed_heading <- ros_heading(msg);
		}
	}

	reflex decide {
		// A plain proportional controller: turn towards the target, slow down when nearly there
		float bearing <- atan2(target.y - believed_position.y, target.x - believed_position.x);
		// wrapped into [-180, 180] so that turning left by 10 does not read as turning right by 350
		float turn_error <- float(int((bearing - believed_heading) + 540) mod 360 - 180);
		float distance <- believed_position distance_to target;

		if distance < 0.5 {
			target <- {rnd(2.0, world_size - 2.0), rnd(2.0, world_size - 2.0)};
		}

		// speed in m/s, rate of turn in degrees per second: ros_twist converts to what ROS expects
		bool sent <- ros_publish(commands, ros_twist(min([1.5, distance]) * (abs(turn_error) < 45 ? 1.0 : 0.2),
			turn_error * 0.03));
	}
}

experiment drive type: gui {

	output {
		display "World" type: 2d {
			graphics "target" {
				draw world.shape color: #white border: #lightgray;
				draw circle(0.4) at: first(controller).target color: #tomato;
			}
			species robot aspect: default;
		}

		display "What the controller believes" type: 2d {
			chart "Position reported on /odom" kind: series {
				data "x" value: first(controller).believed_position.x color: #steelblue;
				data "y" value: first(controller).believed_position.y color: #seagreen;
			}
		}

		monitor "commands received by the robot" value: ros_received(first(robot).commands);
		monitor "odometry dropped by the controller" value: ros_dropped(first(controller).feedback);
	}
}
