/**
* Name: Gazebo in the loop
* Author: Baptiste Lesquoy
* Description: The full loop. GAMA picks a waypoint and sends a velocity command, Gazebo executes it
*              with real wheels, real friction and real inertia, GAMA reads back the odometry, draws
*              where the robot actually is, and decides the next command from that -- not from what it
*              had asked for.
*
*              The difference with model 05 is the whole point. There, GAMA was the authority and
*              Gazebo drew what it was told. Here Gazebo is the authority: the robot accelerates more
*              slowly than commanded, slides in turns, and stops dead against a wall GAMA does not know
*              about. The "commanded vs measured" chart is where that shows, and the dead-reckoning
*              trace shows how far a model that trusted its own commands would have drifted.
*
*              SETUP -- three terminals, see includes/README.md for the details:
*                  gz sim -r ../includes/gama_loop.sdf
*                  ros2 run ros_gz_bridge parameter_bridge --ros-args \
*                      -p config_file:=../includes/gama_loop_bridge.yaml
*                  (then run this experiment)
*
*              Without Gazebo running the model does not crash: it waits, says so, and nothing moves,
*              because there is no odometry to move on.
* Tags: ros, ros2, gazebo, network, display
*/
model gazebo_in_the_loop

global {

	float step <- 0.05 #s;

	// The arena of gama_loop.sdf: walls at +/- 6, so 12 by 12 centred on the origin
	float arena <- 12.0;
	geometry shape <- square(arena);

	// Gazebo puts the origin at the centre of the arena, GAMA at a corner. The two frames differ by
	// this translation and nothing else, and it is applied at one single place: when odometry comes in.
	// Nothing needs converting on the way out, because a velocity is the same in both frames.
	point gz_origin <- {arena / 2, arena / 2};

	ros_node bridge <- ros_node("gama_loop");

	// Reliable, like every ROS command topic: a command that is dropped is a command the robot never
	// hears, and unlike a sensor reading it will not be superseded a moment later.
	ros_topic cmd_topic <- ros_topic("/model/gama_bot/cmd_vel", "geometry_msgs/msg/Twist", "reliable");
	ros_topic odom_topic <- ros_topic("/model/gama_bot/odometry", "nav_msgs/msg/Odometry");

	ros_publisher commands <- ros_publisher(bridge, cmd_topic);

	// A queue of 1: the robot is somewhere now, and where it was thirty odometry messages ago is of no
	// use to a controller. ros_dropped will keep climbing, and that is correct, not a problem.
	ros_subscription feedback <- ros_subscription(bridge, odom_topic, 1);

	// --- what GAMA asks for -------------------------------------------------------------------------
	float cmd_speed <- 0.0;      // m/s
	float cmd_turn <- 0.0;       // degrees/s

	// --- what Gazebo reports ------------------------------------------------------------------------
	bool connected <- false;
	point measured_position <- gz_origin;  // converted to GAMA's frame as it is read
	float measured_yaw <- 0.0;             // degrees; a translation does not change an angle
	float measured_speed <- 0.0;           // m/s, as the wheels actually turned
	list<point> travelled <- [];

	// --- what GAMA would have believed if it had trusted its own commands ---------------------------
	point dead_reckoned <- gz_origin;
	float dead_reckoned_yaw <- 0.0;

	// Written as Gazebo coordinates plus the origin, so the numbers stay the ones you would type into
	// `gz topic` or read in the 3D view
	point target <- gz_origin + {3.5, -3.5};

	init {
		// Say plainly whether the other side is there. Without this a wrong topic name, a bridge that
		// is not running and a mismatched ROS_DOMAIN_ID all look the same: a robot that never moves.
		connected <- ros_wait_for_publisher(feedback, 5000);
		write connected
			? "odometry is flowing -- Gazebo is in the loop"
			: "no publisher on /model/gama_bot/odometry after 5 s.\n"
				+ "  Is 'gz sim -r gama_loop.sdf' running, unpaused?\n"
				+ "  Is ros_gz_bridge running with gama_loop_bridge.yaml?\n"
				+ "  Does ROS_DOMAIN_ID match on both sides?";
	}

	// 1. PERCEIVE -- read the truth Gazebo produced
	reflex perceive {
		map<string, unknown> msg <- ros_read_latest(feedback);
		if msg != nil {
			connected <- true;

			// nav_msgs/Odometry nests: pose.pose.position and pose.pose.orientation
			map<string, unknown> pose <- map<string, unknown>(map<string, unknown>(msg["pose"])["pose"]);
			map<string, unknown> position <- map<string, unknown>(pose["position"]);
			map<string, unknown> orientation <- map<string, unknown>(pose["orientation"]);
			measured_position <- gz_origin + {float(position["x"]), float(position["y"])};
			// a rotation around z is the quaternion (0, 0, sin(yaw/2), cos(yaw/2)), so yaw = 2*atan2(z, w)
			measured_yaw <- 2 * atan2(float(orientation["z"]), float(orientation["w"]));

			// twist.twist.linear.x is the speed the wheels actually produced, not the one commanded
			map<string, unknown> twist <- map<string, unknown>(map<string, unknown>(msg["twist"])["twist"]);
			measured_speed <- float(map<string, unknown>(twist["linear"])["x"]);

			travelled << measured_position;
//			if length(travelled) > 400 { travelled >- first(travelled); }
		}
	}

	// 2. DECIDE -- from the measured pose, never from the commanded one
	reflex decide when: connected {
		float bearing <- atan2(target.y - measured_position.y, target.x - measured_position.x);
		// wrapped into [-180, 180] so that turning left by 10 does not read as turning right by 350
		float turn_error <- float(int((bearing - measured_yaw) + 540) mod 360 - 180);
		float distance <- measured_position distance_to target;

		if distance < 0.4 {
			// Arrived, according to Gazebo. Somewhere else to go, kept clear of the walls and the pillar.
			target <- one_of(([{3.5, 3.5}, {-3.5, 3.5}, {-3.5, -3.5}, {3.5, -3.5}, {0.0, 4.0}]
				collect (gz_origin + each)) where (each distance_to target > 1.0));
		}

		// A plain proportional controller: slow down when close, and when badly aimed turn on the spot
		// rather than drive off in the wrong direction
		cmd_speed <- min([0.9, distance * 0.6]) * (abs(turn_error) < 40 ? 1.0 : 0.15);
		cmd_turn <- max([-70.0, min([70.0, turn_error * 1.5])]);
	}

	// 3. ACT -- send it, and integrate the same command separately to see what trusting it would cost
	reflex act when: connected {
		bool sent <- ros_publish(commands, [
			"linear"::["x"::cmd_speed],
			// GAMA turns in degrees per second, ROS in radians per second
			"angular"::["z"::cmd_turn * #pi / 180.0]
		]);

		dead_reckoned_yaw <- dead_reckoned_yaw + cmd_turn * step;
		dead_reckoned <- dead_reckoned
			+ {cos(dead_reckoned_yaw) * cmd_speed * step, sin(dead_reckoned_yaw) * cmd_speed * step};
	}
}

experiment closed_loop type: gui {

	// Gazebo runs at a real time factor of 1, so GAMA is held to real time too: 20 commands a second,
	// which is a sane rate for a cmd_vel topic. Without it GAMA would spin as fast as it can and flood
	// the robot with commands computed from odometry it has not had time to receive. This belongs to
	// the experiment, not to global.
	float minimum_cycle_duration <- 0.05 #s;

	// Same trap as in model 05, and worse here because the robot has momentum: cmd_vel is a setpoint
	// that DiffDrive keeps applying, so stopping GAMA leaves the robot driving into a wall. Real bases
	// run a watchdog that zeroes the wheels after a fraction of a second without a command. Once the
	// experiment is closed this button is gone with the node, and the stop has to come from a terminal:
	//     ros2 topic pub --once /model/gama_bot/cmd_vel geometry_msgs/msg/Twist "{}"
	user_command "Stop the robot in Gazebo" {
		ask simulation {
			cmd_speed <- 0.0;
			cmd_turn <- 0.0;
			bool stopped <- ros_publish(commands, ["linear"::["x"::0.0], "angular"::["z"::0.0]]);
		}
	}

	output {
		display "Where Gazebo says the robot is" type: 2d {
			graphics "arena" {
				draw world.shape color: #white border: #lightgray;
				// the pillar of the SDF world, at (2, 2) in Gazebo's frame
				draw circle(0.6) at: world.gz_origin + {2, 2} color: #chocolate;

				draw circle(0.4) at: world.target color: #tomato;
				draw "target" at: world.target + {0.5, 0} color: #tomato font: font("Helvetica", 10);

				// The path actually travelled, as reported on /odom
				list<point> travelled_cpy <- copy(travelled);
				if length(travelled_cpy) > 1 {
					draw line(travelled_cpy) color: #steelblue width: 2;
				}

				// The real robot, and next to it where dead reckoning thought it would be
				draw triangle(0.6) at: world.measured_position rotate: world.measured_yaw + 90
					color: #steelblue;
				draw triangle(0.6) at: world.dead_reckoned rotate: world.dead_reckoned_yaw + 90
					color: rgb(180, 180, 180, 120);
			}
		}

		display "What GAMA asked vs what Gazebo did" type: 2d {
			chart "Forward speed (m/s)" type: series {
				data "commanded" value: world.cmd_speed color: #tomato;
				data "measured on /odom" value: world.measured_speed color: #steelblue;
			}
		}

		display "Cost of trusting your own commands" type: 2d {
			chart "Dead reckoning error (m)" type: series {
				data "drift" value: world.dead_reckoned distance_to world.measured_position color: #purple;
			}
		}

		monitor "connected to Gazebo" value: world.connected;
		monitor "odometry messages received" value: ros_received(world.feedback);
		monitor "odometry skipped (expected, queue of 1)" value: ros_dropped(world.feedback);
		monitor "distance to target (m)" value: world.measured_position distance_to world.target;
	}
}
