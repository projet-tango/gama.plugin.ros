/*******************************************************************************************************
 *
 * RosProfileOperators.java, in gama.plugin.ros, is part of the source code of the GAMA modeling and simulation
 * platform.
 *
 * (c) 2007-2026 UMI 209 UMMISCO IRD/SU & Partners (IRIT, MIAT, ESPACE-DEV, CTU)
 *
 * Visit https://github.com/TANGO-projet/gama.plugin.ros for license information and contacts.
 *
 ********************************************************************************************************/
package gama.plugin.ros;

import gama.annotations.doc;
import gama.annotations.example;
import gama.annotations.no_test;
import gama.annotations.operator;
import gama.annotations.support.Reason;
import gama.api.runtime.scope.IScope;
import gama.api.types.map.IMap;
import gama.plugin.ros.utils.RosProfiles;

/**
 * The GAML operators that read and define the robot profiles of the {@code ros_moving} skill. A profile is a plain map:
 * these operators only look one up, or store a new one for the rest of the experiment.
 */
public class RosProfileOperators {

	private RosProfileOperators() {}

	/**
	 * Returns a profile.
	 *
	 * @param scope
	 *            the scope
	 * @param name
	 *            its name
	 * @return a copy, holding every setting
	 */
	@operator (
			value = "ros_profile",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Returns a copy of a robot profile of the ros_moving skill, with a value for every setting: topics, QoS, speed limits, controller gains, watchdog, markers. The built-in ones are 'generic', 'gazebo' and 'alphabot2'; an unknown name is an error listing those that exist.",
			examples = { @example (
					value = "write ros_profile(\"alphabot2\")[\"max_speed\"];",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static IMap<String, Object> rosProfile(final IScope scope, final String name) {
		return RosProfiles.get(scope, name);
	}

	/**
	 * Defines a profile from generic.
	 *
	 * @param scope
	 *            the scope
	 * @param name
	 *            its name
	 * @param settings
	 *            what differs from generic
	 * @return the name
	 */
	@operator (
			value = "ros_define_profile",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Defines a robot profile for the rest of the experiment, starting from 'generic' and overriding the given settings, and returns its name. Agents of the ros_moving skill then name it in ros_profile. Redefining a built-in name replaces it.",
			examples = { @example (
					value = "string p <- ros_define_profile(\"my_bot\", [\"cmd_topic\"::\"{ns}/base/cmd_vel\", \"max_speed\"::0.8]);",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static String rosDefineProfile(final IScope scope, final String name, final IMap<?, ?> settings) {
		return RosProfiles.define(scope, name, RosProfiles.GENERIC, settings);
	}

	/**
	 * Defines a profile from another one.
	 *
	 * @param scope
	 *            the scope
	 * @param name
	 *            its name
	 * @param base
	 *            the profile it starts from
	 * @param settings
	 *            what differs from it
	 * @return the name
	 */
	@operator (
			value = "ros_define_profile",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Defines a robot profile for the rest of the experiment, starting from another one: the way to tune a kind of robot, for every agent that uses it, without touching the agents.",
			examples = { @example (
					value = "string p <- ros_define_profile(\"slow_alphabot\", \"alphabot2\", [\"max_speed\"::0.15]);",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static String rosDefineProfile(final IScope scope, final String name, final String base,
			final IMap<?, ?> settings) {
		return RosProfiles.define(scope, name, base, settings);
	}

}
