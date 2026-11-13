/*******************************************************************************************************
 *
 * RosProfiles.java, in gama.plugin.ros, is part of the source code of the GAMA modeling and simulation platform.
 *
 * (c) 2007-2026 UMI 209 UMMISCO IRD/SU & Partners (IRIT, MIAT, ESPACE-DEV, CTU)
 *
 * Visit https://github.com/TANGO-projet/gama.plugin.ros for license information and contacts.
 *
 ********************************************************************************************************/
package gama.plugin.ros.utils;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeSet;

import gama.api.exceptions.GamaRuntimeException;
import gama.api.gaml.types.Types;
import gama.api.kernel.simulation.IExperimentAgent;
import gama.api.runtime.scope.IScope;
import gama.api.types.list.GamaListFactory;
import gama.api.types.list.IList;
import gama.api.types.map.GamaMapFactory;
import gama.api.types.map.IMap;

/**
 * The robot profiles of the {@code ros_moving} skill: everything the skill needs to know about a kind of robot, as
 * plain data that a model can read, copy, override and extend.
 *
 * <h3>Three levels, the last one wins</h3>
 * <ol>
 * <li>the built-in profiles below, {@code generic} giving the default of every key;</li>
 * <li>the profiles a model defines with {@code ros_define_profile}, kept by the experiment;</li>
 * <li>the {@code ros_settings} map of an agent, for one robot only.</li>
 * </ol>
 *
 * <p>
 * Keys are checked: a key that {@code generic} does not have is an error listing the ones that exist, since a
 * misspelt setting would otherwise be ignored without a word. In topic names, {@code {name}} is replaced by the
 * {@code ros_name} of the agent and {@code {ns}} by {@code "/" + ros_name}, or by nothing when the name is empty.
 * </p>
 */
public final class RosProfiles {

	/** The key under which the profiles a model defines are stored on the experiment agent. */
	private static final String DEFINED = "$ros2_profiles";

	/** The profile every other one is laid over, and the one used when an agent names none. */
	public static final String GENERIC = "generic";

	/** The built-in profiles, by name. Each holds only what differs from {@link #GENERIC}. */
	private static final Map<String, Map<String, Object>> BUILT_IN = new LinkedHashMap<>();

	static {
		final Map<String, Object> g = new LinkedHashMap<>();
		// --- the node -------------------------------------------------------------------------------------------
		g.put("node", "gama");                                  // shared by every robot that names no ros_node
		g.put("domain", 0);                                     // the ROS domain id, as in ros_node(name, domain)
		// --- commands out ---------------------------------------------------------------------------------------
		g.put("cmd_topic", "{ns}/cmd_vel");
		g.put("cmd_type", "geometry_msgs/msg/Twist");           // or geometry_msgs/msg/TwistStamped (ros2_control)
		g.put("cmd_qos", "reliable");
		// --- state in: the pose comes from pose_topic, else from odom_topic, else it is estimated from the commands
		g.put("pose_topic", "");
		g.put("pose_type", "geometry_msgs/msg/PoseStamped");    // any message holding a pose: Pose, Odometry...
		g.put("pose_qos", "default");
		g.put("odom_topic", "{ns}/odom");                       // nav_msgs/msg/Odometry; also gives the real speed
		g.put("odom_qos", "default");
		// --- what the robot can do ------------------------------------------------------------------------------
		g.put("max_speed", 0.5);                                // m/s
		g.put("max_turn_rate", 90.0);                           // degrees/s
		// --- how goto, move, wander and follow steer it ---------------------------------------------------------
		g.put("turn_gain", 1.5);                                // degrees/s of turn per degree of heading error
		g.put("slowdown_gain", 0.6);                            // goto: speed <= distance left * slowdown_gain
		g.put("turn_in_place_angle", 40.0);                     // degrees: beyond, slow down and turn first
		g.put("turn_in_place_speed", 0.15);                     // the fraction of the speed kept while doing so
		g.put("arrival", 0.2);                                  // metres: closer than this, goto has arrived
		g.put("lookahead", 0.5);                                // metres: how far ahead along a path to aim
		// --- safety and diagnostics -----------------------------------------------------------------------------
		g.put("watchdog", 0.5);                                 // s without command before a stop is sent; 0: never
		g.put("timeout", 2.0);                                  // s without pose before ros_connected turns false
		g.put("hint_after", 5.0);                               // s without pose before the hint is printed
		g.put("hint", "Is the driver of the robot running? Does ROS_DOMAIN_ID match on both sides?");
		// --- how the agent is shown in RViz ---------------------------------------------------------------------
		g.put("marker_topic", "/gama/markers");
		g.put("marker_scale", floats(1.0, 0.3, 0.3));           // length, width, height of the marker
		g.put("marker_lifetime", 0.5);                          // s: markers fade out when GAMA stops
		g.put("tf_frame", "gama_world");                        // published once on /tf_static; "" for none
		BUILT_IN.put(GENERIC, g);

		// Gazebo Sim through ros_gz_bridge, with the DiffDrive and PosePublisher systems of includes/gama_loop.sdf
		final Map<String, Object> gz = new LinkedHashMap<>();
		gz.put("cmd_topic", "/model/{name}/cmd_vel");
		gz.put("pose_topic", "/model/{name}/pose");
		gz.put("pose_type", "geometry_msgs/msg/Pose");
		gz.put("odom_topic", "/model/{name}/odometry");
		gz.put("max_speed", 0.9);
		gz.put("max_turn_rate", 70.0);
		gz.put("arrival", 0.4);
		gz.put("lookahead", 0.6);
		gz.put("marker_scale", floats(0.5, 0.15, 0.15));
		gz.put("hint", "Is 'gz sim -r' running, unpaused? Is ros_gz_bridge running with the right config file? "
				+ "Does ROS_DOMAIN_ID match on both sides?");
		BUILT_IN.put("gazebo", gz);

		// Waveshare AlphaBot2 (Pi or Pi Zero): N20 1:30 gear motors, 42 mm wheels, TB6612FNG driver, no encoder and
		// no IMU, and no ROS driver: includes/alphabot2/alphabot2_bridge.py is the node it needs on the Pi. With
		// nothing to measure the pose, it is estimated from the commands, unless pose_topic names a tracker.
		final Map<String, Object> ab = new LinkedHashMap<>();
		ab.put("cmd_topic", "{ns}/cmd_vel");
		ab.put("odom_topic", "");
		ab.put("pose_topic", "");
		ab.put("max_speed", 0.3);        // ~1.3 m/s with no load at 6 V; kept low so the estimate stays honest
		ab.put("max_turn_rate", 120.0);
		ab.put("turn_gain", 2.0);
		ab.put("arrival", 0.05);
		ab.put("lookahead", 0.15);
		ab.put("marker_scale", floats(0.12, 0.06, 0.04));
		ab.put("hint", "The AlphaBot2 has no ROS driver of its own: is alphabot2_bridge.py running on the Pi, "
				+ "with the same ROS_DOMAIN_ID?");
		BUILT_IN.put("alphabot2", ab);
	}

	private RosProfiles() {}

	/**
	 * Returns a profile, laid over {@code generic}: a fresh map holding a value for every key.
	 *
	 * @param scope
	 *            the scope
	 * @param name
	 *            the name of a built-in or defined profile; null or empty for generic
	 * @return a new map the caller may modify
	 */
	public static IMap<String, Object> get(final IScope scope, final String name) {
		final String key = name == null || name.isBlank() ? GENERIC : name;
		final Map<String, Object> profile = definedOf(scope).getOrDefault(key, BUILT_IN.get(key));
		if (profile == null) throw GamaRuntimeException.error("'" + key + "' is not a known robot profile. Known: "
				+ new TreeSet<>(names(scope)) + ". ros_define_profile adds one.", scope);
		final IMap<String, Object> result = copy(BUILT_IN.get(GENERIC));
		result.putAll(copy(profile));
		return result;
	}

	/**
	 * Defines, or redefines, a profile for the rest of the experiment.
	 *
	 * @param scope
	 *            the scope
	 * @param name
	 *            the new name
	 * @param base
	 *            the profile it starts from
	 * @param overrides
	 *            the keys that differ
	 * @return the name
	 */
	public static String define(final IScope scope, final String name, final String base,
			final Map<?, ?> overrides) {
		if (name == null || name.isBlank()) throw GamaRuntimeException.error("A robot profile needs a name", scope);
		final IMap<String, Object> profile = get(scope, base);
		applyOverrides(scope, profile, overrides, "the profile '" + name + "'");
		definedOf(scope).put(name, profile);
		return name;
	}

	/**
	 * Resolves the settings of one robot: profile, then its own overrides, then the templates of its topic names.
	 *
	 * @param scope
	 *            the scope
	 * @param profile
	 *            the name of its profile
	 * @param overrides
	 *            its ros_settings, may be null
	 * @param robotName
	 *            its ros_name, may be null for an agent bound to no robot
	 * @return the settings
	 */
	public static IMap<String, Object> resolve(final IScope scope, final String profile, final Map<?, ?> overrides,
			final String robotName) {
		final IMap<String, Object> result = get(scope, profile);
		applyOverrides(scope, result, overrides, "ros_settings");
		final String name = robotName == null ? "" : robotName;
		final String ns = name.isEmpty() ? "" : "/" + name;
		for (final Map.Entry<String, Object> e : result.entrySet()) {
			if (e.getValue() instanceof final String s && s.contains("{")) {
				e.setValue(s.replace("{ns}", ns).replace("{name}", name));
			}
		}
		return result;
	}

	/**
	 * @param scope
	 *            the scope
	 * @return the names of every profile known to the experiment
	 */
	public static List<String> names(final IScope scope) {
		final TreeSet<String> all = new TreeSet<>(BUILT_IN.keySet());
		all.addAll(definedOf(scope).keySet());
		return List.copyOf(all);
	}

	// ---------------------------------------------------------------------------------------------------------------

	/**
	 * Lays a map of settings over a profile, refusing the keys a profile does not have.
	 *
	 * @param scope
	 *            the scope
	 * @param target
	 *            the profile being built
	 * @param overrides
	 *            the settings, may be null
	 * @param where
	 *            what is being configured, for the error message
	 */
	private static void applyOverrides(final IScope scope, final Map<String, Object> target, final Map<?, ?> overrides,
			final String where) {
		if (overrides == null) return;
		for (final Map.Entry<?, ?> e : overrides.entrySet()) {
			final String key = String.valueOf(e.getKey());
			if (!BUILT_IN.get(GENERIC).containsKey(key)) throw GamaRuntimeException.error("'" + key
					+ "' is not a robot setting, in " + where + ". The settings are "
					+ new TreeSet<>(BUILT_IN.get(GENERIC).keySet()) + ".", scope);
			target.put(key, e.getValue());
		}
	}

	/**
	 * @param scope
	 *            the scope
	 * @return the profiles defined by the current experiment, created on first use
	 */
	@SuppressWarnings ("unchecked")
	private static Map<String, Map<String, Object>> definedOf(final IScope scope) {
		final IExperimentAgent exp = scope == null ? null : scope.getExperiment();
		if (exp == null) return new LinkedHashMap<>();
		synchronized (exp) {
			Map<String, Map<String, Object>> defined = (Map<String, Map<String, Object>>) exp.getAttribute(DEFINED);
			if (defined == null) {
				defined = new LinkedHashMap<>();
				exp.setAttribute(DEFINED, defined);
			}
			return defined;
		}
	}

	/**
	 * @param source
	 *            a profile
	 * @return a GAML map holding a copy of it, lists included
	 */
	private static IMap<String, Object> copy(final Map<String, Object> source) {
		final IMap<String, Object> m = GamaMapFactory.create(Types.STRING, Types.NO_TYPE);
		source.forEach((k, v) -> m.put(k, v instanceof final List<?> l ? listCopy(l) : v));
		return m;
	}

	/**
	 * @param l
	 *            a list
	 * @return a mutable GAML copy
	 */
	private static IList<Object> listCopy(final List<?> l) {
		final IList<Object> copy = GamaListFactory.create(Types.NO_TYPE);
		copy.addAll(l);
		return copy;
	}

	/**
	 * Plain Java on purpose: the built-in profiles are created when the class loads, which must not depend on the GAML
	 * types being ready. They become GAML lists when a profile is copied out.
	 *
	 * @param values
	 *            numbers
	 * @return them as an immutable list
	 */
	private static List<Double> floats(final double... values) {
		return java.util.Arrays.stream(values).boxed().toList();
	}

	// ---------------------------------------------------------------------------------------------------------------
	// Typed reads, used by the skill
	// ---------------------------------------------------------------------------------------------------------------

	/**
	 * @param settings
	 *            resolved settings
	 * @param key
	 *            a key
	 * @return its value as a number
	 */
	public static double num(final Map<String, Object> settings, final String key) {
		return settings.get(key) instanceof final Number n ? n.doubleValue() : 0;
	}

	/**
	 * @param settings
	 *            resolved settings
	 * @param key
	 *            a key
	 * @return its value as a string, empty when absent or nil
	 */
	public static String str(final Map<String, Object> settings, final String key) {
		final Object v = settings.get(key);
		return v == null ? "" : v.toString().trim();
	}

	/**
	 * @param settings
	 *            resolved settings
	 * @param key
	 *            a key holding three numbers
	 * @return them, missing ones being 0.1
	 */
	public static double[] triple(final Map<String, Object> settings, final String key) {
		final double[] r = { 0.1, 0.1, 0.1 };
		if (settings.get(key) instanceof final List<?> l) {
			for (int i = 0; i < Math.min(3, l.size()); i++) { if (l.get(i) instanceof final Number n) { r[i] = n.doubleValue(); } }
		}
		return r;
	}

}
