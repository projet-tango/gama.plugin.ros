/**
* Name: Skill - Gazebo and RViz
* Author: Arthur Brugière
* Description: Model 06 again, with the ros_moving skill, and RViz at the same time.
*
*              The robot of gama_loop.sdf is an agent with ros_name "gama_bot" and the 'gazebo'
*              profile, which knows its topics (/model/gama_bot/cmd_vel, /pose, /odometry), their QoS,
*              and what to say when nothing answers. `do goto` drives it; location and heading are
*              where Gazebo says it is. Next to it wander a few agents that exist in GAMA only:
*              mirrors, as in model 07. RViz shows all of them, the real robot included, at its
*              measured pose. Gazebo shows only the robot: putting GAMA-only agents into a Gazebo world
*              would need Gazebo services, which jros2 does not have.
*
*              Model 06 still does all of this by hand, and still works: the skill is a shortcut, not
*              a replacement.
*
*              SETUP -- four terminals, see includes/README.md:
*                  gz sim -r ../includes/gama_loop.sdf
*                  ros2 run ros_gz_bridge parameter_bridge --ros-args \
*                      -p config_file:=../includes/gama_loop_bridge.yaml
*                  rviz2 -d ../includes/gama_markers.rviz
*                  (then run this experiment)
*
*              Pausing or closing the experiment stops the robot: the skill sends a stop when GAMA has
*              been silent for longer than the 'watchdog' setting (0.5 s).
* Tags: ros, ros2, gazebo, rviz, skill, network, display
*/
model skill_gazebo_and_rviz

global {

	float step <- 0.05 #s;

	// The arena of gama_loop.sdf: walls at +/- 6, centred on the origin, which is where the GAMA world is
	// placed in ROS by default. Nothing to configure.
	float arena <- 12.0;
	geometry shape <- square(arena);

	int nb_ghosts <- 6 min: 0 max: 50;

	// A Gazebo position, as typed into `gz topic` or read in the 3D view, as a GAMA one
	point from_gz (float x, float y) {
		return ros_location(["x"::x, "y"::y]);
	}

	list<point> waypoints;

	init {
		waypoints <- [{3.5, 3.5}, {-3.5, 3.5}, {-3.5, -3.5}, {3.5, -3.5}, {0.0, 4.0}] collect from_gz(each.x, each.y);
		create bot (ros_name:"gama_bot", ros_profile:"gazebo");
		create ghost number: nb_ghosts {
			location <- any_location_in(world.shape - (world.from_gz(2.0, 2.0) buffer 1.0));
		}
	}
}

/**
 * The Gazebo robot. location, heading and real_speed are measured; goto only decides the command.
 */
species bot skills: [ros_moving] {

	point target;
	list<point> travelled;

	// What GAMA would believe if it trusted its own commands, integrated from what the skill actually sent
	point dead_reckoned;
	float dead_reckoned_heading <- 0.0;

	init {
		speed <- 0.9;    // a maximum: goto slows down near the target
		ros_color <- #steelblue;
		target <- world.from_gz(3.5, -3.5);
		dead_reckoned <- world.location;
	}

	// ros_connected stays false until Gazebo answers; after 5 s the 'gazebo' profile prints what to check
	reflex drive when: ros_connected {
		if location distance_to target < 0.4 {
			target <- one_of(world.waypoints where (each distance_to target > 1.0));
		}
		do goto(target: target);
		travelled << location;
	}

	reflex dead_reckon when: ros_connected {
		dead_reckoned_heading <- dead_reckoned_heading + ros_cmd_turn_rate * step;
		dead_reckoned <- dead_reckoned + {cos(dead_reckoned_heading), sin(dead_reckoned_heading)} * ros_cmd_speed * step;
	}

	aspect default {
		if length(travelled) > 1 { draw line(copy(travelled)) color: #steelblue width: 2; }
		draw circle(0.4) at: target color: #tomato;
		draw triangle(0.6) at: dead_reckoned rotate: dead_reckoned_heading + 90 color: rgb(180, 180, 180, 120);
		draw triangle(0.6) rotate: heading + 90 color: #steelblue;
	}
}

/**
 * Agents that exist in GAMA only. Same skill, no ros_name: they move as with the moving skill, and appear in RViz.
 */
species ghost skills: [ros_moving] {

	init {
		speed <- 0.3;
		ros_marker <- "sphere";
		ros_color <- #orange;
		ros_settings <- ["marker_scale"::[0.3, 0.3, 0.3]];
	}

	reflex move {
		do wander(amplitude: 30.0, bounds: world.shape - (world.from_gz(2.0, 2.0) buffer 0.8));
	}

	aspect default {
		draw circle(0.15) color: #orange;
	}
}

experiment closed_loop type: gui {

	parameter "GAMA-only agents" var: nb_ghosts;

	// Gazebo runs in real time, so GAMA is held to it: 20 commands a second
	float minimum_cycle_duration <- 0.05 #s;

	user_command "Stop the robot" {
		ask bot { do ros_stop(); }
	}

	output {
		display "Where Gazebo says the robot is" type: 2d {
			graphics "arena" {
				draw world.shape color: #white border: #lightgray;
				draw circle(0.6) at: world.from_gz(2.0, 2.0) color: #chocolate;   // the pillar of the SDF
			}
			species ghost aspect: default;
			species bot aspect: default;
		}

		display "What GAMA asked vs what Gazebo did" type: 2d {
			chart "Forward speed (m/s)" kind: series {
				data "commanded" value: first(bot).ros_cmd_speed color: #tomato;
				data "measured" value: first(bot).real_speed color: #steelblue;
			}
		}

		display "Cost of trusting your own commands" type: 2d {
			chart "Dead reckoning error (m)" kind: series {
				data "drift" value: first(bot).dead_reckoned distance_to first(bot).location color: #purple;
			}
		}

		monitor "connected to Gazebo" value: first(bot).ros_connected;
		monitor "RViz listening" value: empty(ghost) ? false : first(ghost).ros_connected;
		monitor "pose received" value: ros_received(first(bot).ros_pose_subscription);
		monitor "distance to target (m)" value: first(bot).location distance_to first(bot).target;
	}
}
