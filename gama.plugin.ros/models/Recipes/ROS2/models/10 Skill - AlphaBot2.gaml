/**
* Name: Skill - AlphaBot2
* Author: Arthur Brugière
* Description: Driving a real Waveshare AlphaBot2 around a square, with the ros_moving skill.
*
*              The AlphaBot2 has two N20 gear motors behind a TB6612FNG driver, and nothing else to tell
*              where it is: no wheel encoder, no IMU. And no ROS driver either. So:
*              - on the Pi, includes/alphabot2/alphabot2_bridge.py turns cmd_vel into motor PWM (see
*                includes/alphabot2/README.md to install and calibrate it);
*              - in GAMA, the 'alphabot2' profile names no pose and no odometry topic, so the skill
*                ESTIMATES the pose from the commands it sent. That is dead reckoning: it drifts with
*                every slip, every uneven battery, every wheel that is not quite the size it should be.
*                ros_estimated says so, and the model has to say where the robot was put down.
*              With a tracker that publishes the pose (a camera with ArUco tags, a mocap...), give its
*              topic in the profile below and the estimate is replaced by the measure.
*
*              SETUP:
*                  on the Pi:   python3 alphabot2_bridge.py --ros-args -r __ns:=/alphabot2
*                  here:        run this experiment, with the robot on the floor at the bottom-left
*                               corner of a 1 m square, facing right along its bottom side.
* Tags: ros, ros2, alphabot2, robot, skill, network, display
*/
model skill_alphabot2

global {

	float step <- 0.1 #s;

	// A patch of floor, 2 m by 2 m. Its centre is the origin of the ROS frame.
	float floor_size <- 2.0;
	geometry shape <- square(floor_size);

	// The square the robot drives around, 1 m a side, in GAMA coordinates
	list<point> corners <- [{0.5, 1.5}, {1.5, 1.5}, {1.5, 0.5}, {0.5, 0.5}];

	// Tuning per kind of robot belongs in a profile. This one starts from the built-in 'alphabot2' and
	// overrides what you measured on yours; every robot of this kind in the model then shares it.
	// write ros_profile("alphabot2") prints every setting there is.
	string profile <- ros_define_profile("my_alphabot2", "alphabot2", [
		"max_speed"::0.2,          // m/s: slower than the built-in 0.3, the estimate drifts less
		"max_turn_rate"::90.0,     // degrees/s
		"arrival"::0.03            // m: how close to a corner counts as there
		// With a tracker, the pose is measured instead of estimated:
		// , "pose_topic"::"/alphabot2/pose", "pose_type"::"geometry_msgs/msg/PoseStamped"
	]);

	init {
		// ros_name "alphabot2" is the namespace the bridge runs in: /alphabot2/cmd_vel
		create alphabot(ros_name:"alphabot2", ros_profile:profile) {
			// Nothing on the robot knows where it is, so GAMA has to be told where it was put down. With an
			// estimated pose, writing location and heading is exactly that. Location first: setting it
			// turns the heading towards the move, as with the moving skill.
			location <- world.corners[0];
			heading <- 0.0;    // facing +x: right on the screen, along the first side
		}
	}
}

species alphabot skills: [ros_moving] {

	int next <- 1;
	bool driving <- true;
	list<point> travelled;

	init {
		speed <- 1.0;     // m/s, below max_speed
		ros_settings <- ["marker_scale"::[0.12, 0.06, 0.04]];
	}

	// For an estimated pose, ros_connected means "something listens to the commands": the bridge is up
	reflex drive when: driving and ros_connected {
		point corner <- world.corners[next];
		if location distance_to corner < float(ros_config["arrival"]) + 0.01 {
			next <- (next + 1) mod length(world.corners);
			corner <- world.corners[next];
		}
		do goto(target: corner);
		travelled << location;
	}

	aspect default {
		if length(travelled) > 1 { draw line(copy(travelled)) color: #lightsteelblue; }
		draw triangle(0.08) rotate: heading + 90 color: ros_estimated ? #orange : #steelblue;
	}
}

experiment exp_drive type: gui {

	// A real robot runs in real time
	float minimum_cycle_duration <- 0.1 #s;

	user_command "Stop" {
		ask alphabot {
			driving <- false;
			do ros_stop();
		}
	}

	user_command "Resume" {
		ask alphabot { driving <- true; }
	}

	// Picked the robot up and put it back on the first corner? Say so, the estimate cannot know.
	user_command "Robot is back on the first corner" {
		ask alphabot {
			location <- world.corners[0];
			heading <- 0.0;
			next <- 1;
			travelled <- [];
		}
	}

	output {
		display "Where GAMA thinks the AlphaBot2 is" type: 2d {
			graphics "floor" {
				draw world.shape color: #white border: #lightgray;
				draw polygon(world.corners) wireframe: true color: #tomato;
			}
			species alphabot aspect: default;
		}

		monitor "bridge listening" value: first(alphabot).ros_connected;
		monitor "pose is estimated" value: first(alphabot).ros_estimated;
		monitor "commands to" value: first(alphabot).ros_config["cmd_topic"];
		monitor "speed sent (m/s)" value: first(alphabot).ros_cmd_speed;
		monitor "turn sent (deg/s)" value: first(alphabot).ros_cmd_turn_rate;
	}
}
