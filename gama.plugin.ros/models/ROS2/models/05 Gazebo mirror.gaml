/**
* Name: Gazebo mirror
* Author: Baptiste Lesquoy
* Description: The simplest thing worth doing with Gazebo: GAMA simulates, Gazebo shows. Six agents
*              wander in GAMA, and six entities in Gazebo do exactly the same thing, because GAMA sends
*              each of them the very velocity it is integrating itself.
*
*              Nothing comes back. Gazebo has no opinion here: gravity and friction are off in the
*              world file, and gz-sim's VelocityControl applies the twist as the entity's own velocity.
*              The 3D view is a rendering of the GAMA simulation, not a second simulation that could
*              disagree with it. Model 06 is the one where Gazebo answers back.
*
*              SETUP -- three terminals, see includes/README.md for the details:
*                  gz sim -r ../includes/gama_mirror.sdf
*                  ros2 run ros_gz_bridge parameter_bridge --ros-args \
*                      -p config_file:=../includes/gama_mirror_bridge.yaml
*                  (then run this experiment)
*
*              It also runs with nothing started: GAMA publishes into the void, the agents still move,
*              and ros_subscriber_count stays at 0 to tell you nobody is listening.
* Tags: ros, ros2, gazebo, network, display
*/
model gazebo_mirror

global {

	// Publishing at the rate the simulation steps: one command per agent per cycle
	float step <- 0.05 #s;

	// Fixed by the world file: includes/gama_mirror.sdf declares gama_0 .. gama_5. Add entities there
	// and their bridge entries before raising this.
	int nb_agents <- 6;

	float world_size <- 12.0;
	geometry shape <- square(world_size);

	ros_node bridge <- ros_node("gama_mirror");

	// gama_0 gets the first colour, gama_1 the second, so that an agent can be picked out in both views
	list<rgb> palette <- [#steelblue, #tomato, #seagreen, #orange, #mediumpurple, #teal];

	// The x of each entity in gama_mirror.sdf, in Gazebo's frame. Both worlds have to start from the
	// same pose, otherwise the mirror is only ever parallel to the original.
	list<float> gz_start_x <- [-3.0, -1.5, 0.0, 1.5, 3.0, 4.5];

	init {
		// Created one by one rather than with number:, because each agent needs to know which entity of
		// the SDF world it drives, and that index is what names its topic
		loop i from: 0 to: nb_agents - 1 {
			create wanderer {
				gz_index <- i;
				colour <- palette[i];
				// Same starting pose as the matching model in gama_mirror.sdf, expressed in GAMA's
				// frame: Gazebo has its origin at the centre, GAMA at a corner, so the two frames
				// differ by a translation of half the world and nothing else.
				location <- {world_size / 2 + gz_start_x[i], world_size / 2};
				heading <- 0.0;
				commands <- ros_publisher(bridge,
					ros_topic("/model/gama_" + i + "/cmd_vel", "geometry_msgs/msg/Twist"));
			}
		}
	}

	// Whether the state of the bridge has already been announced, so that a healthy setup says so once
	// instead of every second
	bool bridge_reported <- false;

	// Publishing to nobody is not an error, so a missing bridge looks exactly like a working one. This
	// is re-checked rather than announced once: DDS discovery takes a moment, and starting the bridge
	// after the experiment is a perfectly normal thing to do.
	reflex watch_bridge when: cycle mod 20 = 0 {
		list<int> silent <- wanderer where (ros_subscriber_count(each.commands) = 0)
			collect each.gz_index;
		if empty(silent) {
			if !bridge_reported {
				write "the bridge is up: all " + nb_agents + " entities are listening";
				bridge_reported <- true;
			}
		} else {
			bridge_reported <- false;
			// Naming the silent ones matters: the usual cause is a bridge started with the wrong config
			// file, and a bridge that bridges other topics entirely looks perfectly healthy in its own
			// terminal. Six 'Creating ROS->GZ Bridge' lines is what a correct start prints.
			write "nobody is listening to gama_" + silent + ". Check that ros_gz_bridge is running with "
				+ "gama_mirror_bridge.yaml, and that it printed one 'Creating ROS->GZ Bridge' line per "
				+ "entity -- six of them, not two.";
		}
	}
}

/**
 * A GAMA agent that happens to have a body in Gazebo. It moves by its own rules, in GAMA, and sends the
 * same motion out so that the entity in the 3D view stays on top of it.
 */
species wanderer skills:[moving]{

//	float yaw <- 0.0;              // degrees, GAMA's convention for cos, sin and rotate
	float speed <- rnd(0.4, 1.0);  // m/s, the only thing that differs between agents at the start
	float turn_rate <- 0.0;        // degrees/s
	
	float header_old;
	
	

	// Which entity of the SDF world this agent drives: gz_index 0 is the model named gama_0
	int gz_index;
	rgb colour;
	ros_publisher commands;

	// Wander around the world, save the turn_rate
	reflex move {
		float heading_old <- heading;
		
		do wander(amplitude: 25.0);
		
		turn_rate <- float(int(heading - heading_old + 540) mod 360);
	}

	// The mirror itself. GAMA and gz-sim integrate the same unicycle from the same starting pose, so
	// sending the velocity is enough -- no pose is ever transmitted, and no correction is needed.
	reflex mirror_to_gazebo {
		bool sent <- ros_publish(commands, [
			"linear"::["x"::speed],
			// GAMA turns in degrees per second, ROS in radians per second
			"angular"::["z"::turn_rate * #pi / 180.0]
		]);
	}

	aspect default {
		draw triangle(0.7) rotate: heading + 90 color: colour;
	}
}

experiment mirror type: gui {

	// The mirror is only exact if the two clocks agree. GAMA integrates 'step' of simulated time per
	// cycle, while gz-sim runs at a real time factor of 1: without this, GAMA runs as fast as it can,
	// sends far more commands per real second than Gazebo has seconds to execute them in, and the
	// entities lag further behind on every cycle. Holding each cycle to 'step' of wall clock puts the
	// two on the same clock. This belongs to the experiment, not to global.
	float minimum_cycle_duration <- 0.05 #s;

	// A Twist is a setpoint, not an impulse: VelocityControl re-applies the last one it received at
	// every step of the physics, for as long as the world runs. Stopping GAMA only stops the messages,
	// and "no more messages" is indistinguishable from "the same command still holds", so the entities
	// carry on at their last velocity indefinitely. A real robot solves this with a watchdog that
	// zeroes the motors after a fraction of a second without a command; VelocityControl has none, so
	// the stop has to be sent on purpose.
	//
	// This only works while the experiment is still alive, paused or running. Once it is closed the
	// node is gone with it, and the stop has to come from a terminal:
	//     ros2 topic pub --once /model/gama_0/cmd_vel geometry_msgs/msg/Twist "{}"
	user_command "Stop the entities in Gazebo" {
		ask simulation {
			ask wanderer {
				speed <- 0.0;
				turn_rate <- 0.0;
				bool stopped <- ros_publish(commands, ["linear"::["x"::0.0], "angular"::["z"::0.0]]);
			}
		}
	}

	output {
		display "What GAMA simulates (Gazebo shows the same)" type: 2d {
			graphics "arena" {
				draw world.shape color: #white border: #lightgray;
			}
			species wanderer aspect: default;
		}

		monitor "entities listening in Gazebo"
			value: wanderer count (ros_subscriber_count(each.commands) > 0);
		monitor "commands sent per cycle" value: length(wanderer);
	}
}
