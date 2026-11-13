/*******************************************************************************************************
 *
 * RosGeometryOperators.java, in gama.plugin.ros, is part of the source code of the GAMA modeling and simulation
 * platform.
 *
 * (c) 2007-2026 UMI 209 UMMISCO IRD/SU & Partners (IRIT, MIAT, ESPACE-DEV, CTU)
 *
 * Visit https://github.com/TANGO-projet/gama.plugin.ros for license information and contacts.
 *
 ********************************************************************************************************/
package gama.plugin.ros;

import java.util.List;
import java.util.Map;
import java.util.TreeSet;

import gama.annotations.doc;
import gama.annotations.example;
import gama.annotations.no_test;
import gama.annotations.operator;
import gama.annotations.support.Reason;
import gama.api.exceptions.GamaRuntimeException;
import gama.api.gaml.types.Types;
import gama.api.runtime.scope.IScope;
import gama.api.types.geometry.IPoint;
import gama.api.types.list.GamaListFactory;
import gama.api.types.map.GamaMapFactory;
import gama.api.types.map.IMap;
import gama.plugin.ros.utils.RosFrame;
import gama.plugin.ros.utils.RosFrame.Placement;

/**
 * The GAML operators that translate between GAMA's geometry and ROS's: locations, headings and velocities.
 *
 * <p>
 * GAMA and ROS do not only differ by an origin and a unit: GAMA's y axis points down and its angles turn clockwise,
 * ROS is right-handed. Writing the conversion by hand in every model is the most common source of a robot that steers
 * the mirror image of what it was told, so these operators do it, and do it the same way everywhere. See
 * {@link RosFrame} for the exact convention.
 * </p>
 *
 * <pre>
 * global {
 *     string frame &lt;- ros_world_frame("world");          // optional: "map", centred on the origin, by default
 *
 *     reflex perceive {
 *         map&lt;string, unknown&gt; odom &lt;- ros_read_latest(odometry);
 *         if odom != nil {
 *             location &lt;- ros_location(odom);             // digs pose.pose.position out by itself
 *             heading  &lt;- ros_heading(odom);
 *         }
 *     }
 *     reflex act {
 *         bool sent &lt;- ros_publish(command, ros_twist(0.5, 30.0));   // m/s, degrees/s
 *     }
 * }
 * </pre>
 */
public class RosGeometryOperators {

	/** Where a position can be found in a message, tried in this order. */
	private static final List<String> TOWARDS_POSITION = List.of("position", "translation", "pose", "transform");

	/** Where an orientation can be found in a message, tried in this order. */
	private static final List<String> TOWARDS_ORIENTATION = List.of("orientation", "rotation", "pose", "transform");

	/** How deep the search for a pose or a twist goes: Odometry, the deepest common message, needs three levels. */
	private static final int MAX_DEPTH = 6;

	private RosGeometryOperators() {}

	// ---------------------------------------------------------------------------------------------------------------
	// Placement of the world
	// ---------------------------------------------------------------------------------------------------------------

	/**
	 * Places the world of the current simulation in a ROS frame, its centre at the origin of that frame.
	 *
	 * @param scope
	 *            the scope
	 * @param frameId
	 *            the ROS frame
	 * @return the frame id, so that it can be kept in a variable
	 */
	@operator (
			value = "ros_world_frame",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Says which ROS frame the GAMA world is placed in, with its centre at the origin of that frame, and returns the frame id. Every conversion of this plugin (ros_point, ros_pose, ros_location...) and every header left out of a published message then use it. Without it the world is placed at the centre of 'map', which is where a Gazebo world and the RViz grid put their origin.",
			examples = { @example (
					value = "string frame <- ros_world_frame(\"world\");",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static String rosWorldFrame(final IScope scope, final String frameId) {
		return RosFrame.place(scope, frameId, null).frameId();
	}

	/**
	 * Places the world of the current simulation in a ROS frame, its centre at a given point of that frame.
	 *
	 * @param scope
	 *            the scope
	 * @param frameId
	 *            the ROS frame
	 * @param centre
	 *            where, in ROS coordinates, the centre of the GAMA world goes
	 * @return the frame id
	 */
	@operator (
			value = "ros_world_frame",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Says which ROS frame the GAMA world is placed in, and where in that frame (in ROS coordinates, metres) the centre of the GAMA world goes. Returns the frame id.",
			examples = { @example (
					value = "string frame <- ros_world_frame(\"map\", {10.0, 5.0});",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static String rosWorldFrame(final IScope scope, final String frameId, final IPoint centre) {
		return RosFrame.place(scope, frameId, centre).frameId();
	}

	/**
	 * Builds the transform that makes the frame of the world exist in TF, and gives the centre of the GAMA world a
	 * frame of its own.
	 *
	 * @param scope
	 *            the scope
	 * @param childFrame
	 *            the name of the frame placed at the centre of the GAMA world
	 * @return a map in the shape of tf2_msgs/TFMessage
	 */
	@operator (
			value = "ros_world_tf",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Returns a tf2_msgs/TFMessage with one transform, from the frame of the world to a frame of the given name at the centre of the GAMA world. RViz draws nothing whose fixed frame is absent from TF, and nothing else may publish that frame when GAMA is the only node: publishing this once on /tf_static, with the 'latched' quality of service, makes it exist for as long as the node lives.",
			examples = { @example (
					value = "bool placed <- ros_publish(ros_publisher(bridge, ros_topic(\"/tf_static\", \"tf2_msgs/msg/TFMessage\", \"latched\")), ros_world_tf(\"gama_world\"));",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static IMap<String, Object> rosWorldTf(final IScope scope, final String childFrame) {
		if (childFrame == null || childFrame.isBlank())
			throw GamaRuntimeException.error("ros_world_tf needs the name of the frame to create", scope);
		final Placement p = RosFrame.placement(scope);
		if (childFrame.equals(p.frameId())) throw GamaRuntimeException
				.error("ros_world_tf cannot place the frame '" + childFrame + "' in itself; choose another name", scope);
		final IMap<String, Object> rotation = RosFrame.xyz(0, 0, 0);
		rotation.put("w", 1.0);
		final IMap<String, Object> transform = map();
		transform.put("translation", RosFrame.xyz(p.x(), p.y(), p.z()));
		transform.put("rotation", rotation);
		final IMap<String, Object> header = map();
		header.put("frameId", p.frameId());   // the stamp is filled in when the message is published
		final IMap<String, Object> stamped = map();
		stamped.put("header", header);
		stamped.put("childFrameId", childFrame);
		stamped.put("transform", transform);
		final IMap<String, Object> result = map();
		result.put("transforms", GamaListFactory.wrap(Types.MAP, List.of(stamped)));
		return result;
	}

	// ---------------------------------------------------------------------------------------------------------------
	// GAMA -> ROS
	// ---------------------------------------------------------------------------------------------------------------

	/**
	 * Converts a GAMA location to a ROS point.
	 *
	 * @param scope
	 *            the scope
	 * @param location
	 *            the location
	 * @return a map in the shape of geometry_msgs/Point
	 */
	@operator (
			value = "ros_point",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Converts a GAMA location to a geometry_msgs/Point in the frame of the world: placed, and with its y axis turned the ROS way. Use it wherever a message expects a position, such as the points of a line marker.",
			examples = { @example (
					value = "\"points\"::(world.shape.points collect ros_point(each))",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static IMap<String, Object> rosPoint(final IScope scope, final IPoint location) {
		return RosFrame.toRosPoint(scope, location);
	}

	/**
	 * Converts a GAMA heading to a ROS quaternion.
	 *
	 * @param scope
	 *            the scope
	 * @param heading
	 *            degrees, GAMA's convention
	 * @return a map in the shape of geometry_msgs/Quaternion
	 */
	@operator (
			value = "ros_orientation",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Converts a GAMA heading, in degrees and clockwise on screen, to the geometry_msgs/Quaternion of the same direction in ROS.",
			examples = { @example (
					value = "\"orientation\"::ros_orientation(heading)",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static IMap<String, Object> rosOrientation(final IScope scope, final Double heading) {
		return RosFrame.toRosQuaternion(heading == null ? 0 : heading);
	}

	/**
	 * Converts a GAMA location and heading to a ROS pose.
	 *
	 * @param scope
	 *            the scope
	 * @param location
	 *            the location
	 * @param heading
	 *            degrees, GAMA's convention
	 * @return a map in the shape of geometry_msgs/Pose
	 */
	@operator (
			value = "ros_pose",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Converts a GAMA location and heading to a geometry_msgs/Pose in the frame of the world. It is what a marker, a PoseStamped or a goal for a navigation stack expects.",
			examples = { @example (
					value = "\"pose\"::ros_pose(each.location, each.heading)",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static IMap<String, Object> rosPose(final IScope scope, final IPoint location, final Double heading) {
		final IMap<String, Object> pose = map();
		pose.put("position", RosFrame.toRosPoint(scope, location));
		pose.put("orientation", RosFrame.toRosQuaternion(heading == null ? 0 : heading));
		return pose;
	}

	/**
	 * Builds the velocity command of a mobile base.
	 *
	 * @param scope
	 *            the scope
	 * @param speed
	 *            forward speed, metres per second
	 * @param turnRate
	 *            degrees per second, GAMA's convention
	 * @return a map in the shape of geometry_msgs/Twist
	 */
	@operator (
			value = "ros_twist",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Builds a geometry_msgs/Twist, the command of a mobile base, from a forward speed in m/s and a rate of turn in degrees per second, positive clockwise like GAMA headings. The conversion to radians and to the counter-clockwise ROS convention is done here.",
			examples = { @example (
					value = "bool sent <- ros_publish(cmd_vel, ros_twist(0.5, 20.0));",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static IMap<String, Object> rosTwist(final IScope scope, final Double speed, final Double turnRate) {
		final IMap<String, Object> twist = map();
		twist.put("linear", RosFrame.xyz(speed == null ? 0 : speed, 0, 0));
		twist.put("angular", RosFrame.xyz(0, 0, RosFrame.toRosAngularZ(turnRate == null ? 0 : turnRate)));
		return twist;
	}

	// ---------------------------------------------------------------------------------------------------------------
	// ROS -> GAMA
	// ---------------------------------------------------------------------------------------------------------------

	/**
	 * Finds the position held by a message and converts it to a GAMA location.
	 *
	 * @param scope
	 *            the scope
	 * @param msg
	 *            any message holding a position
	 * @return the location
	 */
	@operator (
			value = "ros_location",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Finds the position held by a message and returns it as a GAMA location, converted from the frame of the world. It looks through position, translation, pose and transform, so the same call reads a Point, a Pose, a PoseStamped, an Odometry (pose.pose.position) or a TransformStamped.",
			examples = { @example (
					value = "location <- ros_location(ros_read_latest(odometry));",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static IPoint rosLocation(final IScope scope, final IMap<?, ?> msg) {
		final Map<?, ?> p = find(scope, msg, "ros_location", "a position", RosGeometryOperators::isPoint,
				TOWARDS_POSITION, MAX_DEPTH);
		return RosFrame.toGamaPoint(scope, RosFrame.number(p, "x"), RosFrame.number(p, "y"), RosFrame.number(p, "z"));
	}

	/**
	 * Finds the orientation held by a message and converts it to a GAMA heading.
	 *
	 * @param scope
	 *            the scope
	 * @param msg
	 *            any message holding an orientation
	 * @return the heading, degrees in [0, 360)
	 */
	@operator (
			value = "ros_heading",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Finds the orientation held by a message and returns it as a GAMA heading, in degrees in [0, 360) and clockwise on screen. Like ros_location, it reads a Quaternion, a Pose, a PoseStamped, an Odometry or a TransformStamped. Roll and pitch are ignored.",
			examples = { @example (
					value = "heading <- ros_heading(ros_read_latest(odometry));",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static Double rosHeading(final IScope scope, final IMap<?, ?> msg) {
		final Map<?, ?> q = find(scope, msg, "ros_heading", "an orientation", RosGeometryOperators::isQuaternion,
				TOWARDS_ORIENTATION, MAX_DEPTH);
		return RosFrame.quaternionToGamaHeading(RosFrame.number(q, "x"), RosFrame.number(q, "y"),
				RosFrame.number(q, "z"), RosFrame.number(q, "w"));
	}

	/**
	 * Finds the velocity held by a message and returns its forward component.
	 *
	 * @param scope
	 *            the scope
	 * @param msg
	 *            a Twist, a TwistStamped or an Odometry
	 * @return metres per second
	 */
	@operator (
			value = "ros_speed",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Returns the forward speed, in m/s, of the velocity held by a Twist, a TwistStamped or an Odometry (twist.twist.linear.x). On odometry this is the speed the wheels actually produced, not the commanded one.",
			examples = { @example (
					value = "float measured <- ros_speed(ros_read_latest(odometry));",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static Double rosSpeed(final IScope scope, final IMap<?, ?> msg) {
		final Map<?, ?> twist = findTwist(scope, msg, "ros_speed");
		return twist.get("linear") instanceof final Map<?, ?> linear ? RosFrame.number(linear, "x") : 0.0;
	}

	/**
	 * Finds the velocity held by a message and returns its rate of turn.
	 *
	 * @param scope
	 *            the scope
	 * @param msg
	 *            a Twist, a TwistStamped or an Odometry
	 * @return degrees per second, GAMA's convention
	 */
	@operator (
			value = "ros_turn_rate",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Returns the rate of turn, in degrees per second and positive clockwise like GAMA headings, of the velocity held by a Twist, a TwistStamped or an Odometry. The inverse of the second argument of ros_twist.",
			examples = { @example (
					value = "float turning <- ros_turn_rate(ros_read_latest(odometry));",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static Double rosTurnRate(final IScope scope, final IMap<?, ?> msg) {
		final Map<?, ?> twist = findTwist(scope, msg, "ros_turn_rate");
		return twist.get("angular") instanceof final Map<?, ?> angular
				? RosFrame.toGamaTurnRate(RosFrame.number(angular, "z")) : 0.0;
	}

	// ---------------------------------------------------------------------------------------------------------------
	// Search in nested messages
	// ---------------------------------------------------------------------------------------------------------------

	/**
	 * A test on a candidate sub-message.
	 */
	@FunctionalInterface
	private interface Shape {
		/**
		 * @param m
		 *            a sub-message
		 * @return whether it is the one looked for
		 */
		boolean matches(Map<?, ?> m);
	}

	/**
	 * Looks for a sub-message of a given shape, following the given keys depth first.
	 *
	 * @param scope
	 *            the scope
	 * @param msg
	 *            the message
	 * @param operator
	 *            the name of the calling operator, for error messages
	 * @param what
	 *            what is looked for, for error messages
	 * @param shape
	 *            the test identifying the sub-message
	 * @param keys
	 *            the fields to follow
	 * @param depth
	 *            how many more levels may be visited
	 * @return the sub-message, never null
	 */
	private static Map<?, ?> find(final IScope scope, final Map<?, ?> msg, final String operator, final String what,
			final Shape shape, final List<String> keys, final int depth) {
		if (msg == null) throw GamaRuntimeException.error(operator
				+ " was given nil. ros_read and ros_read_latest return nil when nothing has arrived: test the message against nil first.",
				scope);
		final Map<?, ?> found = search(msg, shape, keys, depth);
		if (found == null) throw GamaRuntimeException.error(operator + " found no " + what.replaceFirst("^an? ", "")
				+ " in a message whose fields are " + new TreeSet<>(msg.keySet().stream().map(String::valueOf).toList())
				+ ".", scope);
		return found;
	}

	/**
	 * The recursive part of {@link #find}.
	 *
	 * @param msg
	 *            the message
	 * @param shape
	 *            the test identifying the sub-message
	 * @param keys
	 *            the fields to follow
	 * @param depth
	 *            how many more levels may be visited
	 * @return the sub-message, or null
	 */
	private static Map<?, ?> search(final Map<?, ?> msg, final Shape shape, final List<String> keys, final int depth) {
		if (shape.matches(msg)) return msg;
		if (depth <= 0) return null;
		for (final String key : keys) {
			if (msg.get(key) instanceof final Map<?, ?> sub) {
				final Map<?, ?> found = search(sub, shape, keys, depth - 1);
				if (found != null) return found;
			}
		}
		return null;
	}

	/**
	 * Looks for a Twist in a message.
	 *
	 * @param scope
	 *            the scope
	 * @param msg
	 *            the message
	 * @param operator
	 *            the calling operator, for error messages
	 * @return the twist
	 */
	private static Map<?, ?> findTwist(final IScope scope, final Map<?, ?> msg, final String operator) {
		return find(scope, msg, operator, "a velocity",
				m -> m.get("linear") instanceof Map && m.get("angular") instanceof Map, List.of("twist"), MAX_DEPTH);
	}

	/**
	 * @param m
	 *            a candidate
	 * @return whether it has the shape of a Point or a Vector3, and not of a Quaternion
	 */
	private static boolean isPoint(final Map<?, ?> m) {
		return m.get("x") instanceof Number && m.get("y") instanceof Number && !m.containsKey("w");
	}

	/**
	 * @param m
	 *            a candidate
	 * @return whether it has the shape of a Quaternion
	 */
	private static boolean isQuaternion(final Map<?, ?> m) {
		return m.get("z") instanceof Number && m.get("w") instanceof Number;
	}

	/**
	 * @return a new map keyed by field name, the shape of every message of this plugin
	 */
	private static IMap<String, Object> map() {
		return GamaMapFactory.create(Types.STRING, Types.NO_TYPE);
	}

}
