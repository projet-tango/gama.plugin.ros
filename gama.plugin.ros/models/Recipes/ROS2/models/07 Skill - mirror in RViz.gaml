/**
* Name: Skill - mirror in RViz
* Author: Arthur Brugière
* Description: Model 05 again, with the ros_moving skill instead of hand-written markers.
*
*              ros_moving is a drop-in replacement for the moving skill. An agent with no ros_name is
*              a "mirror": it moves exactly as it would with moving -- same actions, same results --
*              and the skill also draws it in RViz, as one marker per agent batched into a single
*              MarkerArray on /gama/markers. The world frame, the stamps, the stable marker ids and the
*              clean-up of a previous run are all done by the skill: compare with model 05, which does
*              each of them by hand and still works.
*
*              SETUP, as for model 05 (see includes/README.md):
*                  rviz2 -d /path/to/includes/gama_markers.rviz
* Tags: ros, ros2, rviz, skill, network, display
*/
model skill_mirror_in_rviz

global {

	float step <- 0.05 #s;

	int nb_agents <- 40 min: 1 max: 400;

	float world_size <- 40.0;
	geometry shape <- square(world_size);

	list<rgb> palette <- [#steelblue, #tomato, #seagreen, #orange, #mediumpurple, #teal];

	// Tuning a whole kind of agent at once. A profile is plain data: this one starts from 'generic' and only
	// changes the size of the markers, which the built-in profiles size for a robot, not for a 40 m world.
	// write ros_profile("generic") prints every setting there is.
	string big_world <- ros_define_profile("forty_metre_world", ["marker_scale"::[1.6, 0.5, 0.5]]);

	init {
		create wanderer number: nb_agents {
			ros_profile <- world.big_world;
			ros_color <- world.palette[int(self) mod length(world.palette)];
		}
	}

	// Publishing to nobody is not an error, so say it out loud
	reflex watch_rviz when: cycle mod 40 = 0 {
		write first(wanderer).ros_connected
			? "RViz is subscribed to /gama/markers"
			: "Nobody is subscribed to /gama/markers. Is rviz2 running, with a MarkerArray display on that topic and Fixed Frame set to map?";
	}
}

// Was `skills: [moving]`. Nothing else in the species changes.
species wanderer skills: [ros_moving] {
	
	float speed <- rnd(0.4, 1.0);

	reflex move {
		do wander(amplitude: 10.0);
	}

	aspect default {
		draw triangle(0.9) rotate: heading + 90 color: ros_color;
	}
}

experiment rviz type: gui {

	parameter "Number of agents" var: nb_agents category: "Population";

	float minimum_cycle_duration <- 0.05 #s;

	output {
		display "What GAMA simulates (RViz shows the same)" type: 2d {
			graphics "arena" {
				draw world.shape color: #white border: #lightgray;
			}
			species wanderer aspect: default;
		}

		monitor "RViz listening" value: first(wanderer).ros_connected;
		monitor "marker topic" value: first(wanderer).ros_config["marker_topic"];
	}
}
