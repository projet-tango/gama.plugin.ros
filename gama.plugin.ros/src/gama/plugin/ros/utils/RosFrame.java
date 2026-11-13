/*******************************************************************************************************
 *
 * RosFrame.java, in gama.plugin.ros, is part of the source code of the GAMA modeling and simulation platform.
 *
 * (c) 2007-2026 UMI 209 UMMISCO IRD/SU & Partners (IRIT, MIAT, ESPACE-DEV, CTU)
 *
 * Visit https://github.com/TANGO-projet/gama.plugin.ros for license information and contacts.
 *
 ********************************************************************************************************/
package gama.plugin.ros.utils;

import java.util.Map;

import gama.api.exceptions.GamaRuntimeException;
import gama.api.gaml.types.Types;
import gama.api.kernel.simulation.ISimulationAgent;
import gama.api.runtime.scope.IScope;
import gama.api.types.geometry.GamaPointFactory;
import gama.api.types.geometry.IPoint;
import gama.api.types.map.GamaMapFactory;
import gama.api.types.map.IMap;
import gama.api.utils.geometry.IEnvelope;

/**
 * The single place where GAMA coordinates become ROS ones, and back.
 *
 * <h3>Why this cannot be left to TF</h3>
 *
 * <p>
 * GAMA's y axis points down the screen and its headings turn clockwise; ROS (REP 103) is right-handed, with y to the
 * left of x and yaws turning counter-clockwise. The two frames therefore differ by a <b>reflection</b>, and a
 * reflection is not a rigid transform: no TF, static or not, can express it. Models used to compensate by hand, each in
 * its own way (one flipped y, another mirrored x and swapped the quaternion, a third did nothing), so the conversion is
 * done here once and every operator goes through it.
 * </p>
 *
 * <h3>The convention</h3>
 *
 * <p>
 * The GAMA world is placed in one ROS frame, the <i>parent frame</i> ({@code "map"} by default), with the centre of its
 * envelope at a given point of that frame ({@code {0, 0, 0}} by default, which is where both a Gazebo world and the
 * RViz grid put their origin). Then, with {@code (cx, cy)} the centre of the GAMA envelope and {@code (ox, oy, oz)} the
 * point it is placed at:
 * </p>
 *
 * <pre>
 *     x_ros = ox + (x - cx)        y_ros = oy - (y - cy)        z_ros = oz + z
 *     yaw_ros = -heading            (radians vs degrees)
 * </pre>
 *
 * <p>
 * Velocities expressed in the body of a robot are not placed but still reflected: forward stays forward, a rate of
 * turn changes sign, and so does a lateral speed.
 * </p>
 *
 * <p>
 * The placement belongs to the <b>simulation</b>, since each simulation has its own world. It is read on every
 * conversion rather than cached, so it follows a world whose shape is set after the placement is.
 * </p>
 */
public final class RosFrame {

	/** The key under which the placement is stored on the simulation agent. */
	private static final String PLACEMENT = "$ros2_world_frame";

	/** The frame GAMA's world is placed in when the model does not say otherwise. */
	public static final String DEFAULT_FRAME = "map";

	/**
	 * Where the GAMA world sits in ROS.
	 *
	 * @param frameId
	 *            the ROS frame the coordinates are expressed in
	 * @param x
	 *            the x, in that frame, of the centre of the GAMA world
	 * @param y
	 *            the y, in that frame, of the centre of the GAMA world
	 * @param z
	 *            the z, in that frame, of the ground of the GAMA world
	 */
	public record Placement(String frameId, double x, double y, double z) {}

	/** The placement used until a model sets one. */
	private static final Placement DEFAULT = new Placement(DEFAULT_FRAME, 0, 0, 0);

	private RosFrame() {}

	// ---------------------------------------------------------------------------------------------------------------
	// Placement
	// ---------------------------------------------------------------------------------------------------------------

	/**
	 * Places the GAMA world of the current simulation in a ROS frame.
	 *
	 * @param scope
	 *            the scope
	 * @param frameId
	 *            the ROS frame, not empty
	 * @param centre
	 *            where, in that frame, the centre of the GAMA world goes; null for the origin
	 * @return the placement now in force
	 */
	public static Placement place(final IScope scope, final String frameId, final IPoint centre) {
		if (frameId == null || frameId.isBlank())
			throw GamaRuntimeException.error("The ROS frame of the world cannot be empty", scope);
		final Placement p = centre == null ? new Placement(frameId, 0, 0, 0)
				: new Placement(frameId, centre.getX(), centre.getY(), centre.getZ());
		simulationOf(scope).setAttribute(PLACEMENT, p);
		return p;
	}

	/**
	 * Returns the placement of the current simulation, the default one if the model did not set any.
	 *
	 * @param scope
	 *            the scope
	 * @return the placement, never null
	 */
	public static Placement placement(final IScope scope) {
		final ISimulationAgent sim = scope == null ? null : scope.getSimulation();
		if (sim == null) return DEFAULT;
		return sim.getAttribute(PLACEMENT) instanceof final Placement p ? p : DEFAULT;
	}

	/**
	 * Returns the frame the coordinates of the current simulation are expressed in.
	 *
	 * @param scope
	 *            the scope, may be null outside of a simulation
	 * @return the frame id
	 */
	public static String frameId(final IScope scope) {
		return placement(scope).frameId();
	}

	// ---------------------------------------------------------------------------------------------------------------
	// Positions
	// ---------------------------------------------------------------------------------------------------------------

	/**
	 * Converts a GAMA location to a ROS position in the parent frame.
	 *
	 * @param scope
	 *            the scope
	 * @param p
	 *            the location
	 * @return a map with x, y and z, the shape of geometry_msgs/Point
	 */
	public static IMap<String, Object> toRosPoint(final IScope scope, final IPoint p) {
		if (p == null) throw GamaRuntimeException.error("Cannot convert a nil location to ROS", scope);
		final Placement pl = placement(scope);
		final IEnvelope env = envelopeOf(scope);
		return xyz(pl.x() + p.getX() - centreX(env), pl.y() - (p.getY() - centreY(env)), pl.z() + p.getZ());
	}

	/**
	 * Converts a ROS position in the parent frame to a GAMA location.
	 *
	 * @param scope
	 *            the scope
	 * @param x
	 *            the ROS x
	 * @param y
	 *            the ROS y
	 * @param z
	 *            the ROS z
	 * @return the location
	 */
	public static IPoint toGamaPoint(final IScope scope, final double x, final double y, final double z) {
		final Placement pl = placement(scope);
		final IEnvelope env = envelopeOf(scope);
		return GamaPointFactory.create(centreX(env) + x - pl.x(), centreY(env) - (y - pl.y()), z - pl.z());
	}

	// ---------------------------------------------------------------------------------------------------------------
	// Orientations
	// ---------------------------------------------------------------------------------------------------------------

	/**
	 * Converts a GAMA heading to a ROS yaw.
	 *
	 * @param heading
	 *            degrees, clockwise on GAMA's screen
	 * @return radians, counter-clockwise
	 */
	public static double toRosYaw(final double heading) {
		return -Math.toRadians(heading);
	}

	/**
	 * Converts a ROS yaw to a GAMA heading, in [0, 360) like the headings of the moving skill.
	 *
	 * @param yaw
	 *            radians, counter-clockwise
	 * @return degrees, clockwise on GAMA's screen
	 */
	public static double toGamaHeading(final double yaw) {
		final double h = -Math.toDegrees(yaw) % 360;
		return h < 0 ? h + 360 : h;
	}

	/**
	 * Converts a GAMA heading to the quaternion of the corresponding rotation around the ROS z axis.
	 *
	 * @param heading
	 *            degrees, clockwise on GAMA's screen
	 * @return a map with x, y, z and w, the shape of geometry_msgs/Quaternion
	 */
	public static IMap<String, Object> toRosQuaternion(final double heading) {
		final double half = toRosYaw(heading) / 2;
		final IMap<String, Object> q = xyz(0, 0, Math.sin(half));
		q.put("w", Math.cos(half));
		return q;
	}

	/**
	 * Extracts the yaw of a quaternion and converts it to a GAMA heading. Roll and pitch are ignored: GAMA agents
	 * have a single heading.
	 *
	 * @param x
	 *            the quaternion x
	 * @param y
	 *            the quaternion y
	 * @param z
	 *            the quaternion z
	 * @param w
	 *            the quaternion w
	 * @return degrees, clockwise on GAMA's screen
	 */
	public static double quaternionToGamaHeading(final double x, final double y, final double z, final double w) {
		return toGamaHeading(Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)));
	}

	// ---------------------------------------------------------------------------------------------------------------
	// Velocities, in the body of the robot
	// ---------------------------------------------------------------------------------------------------------------

	/**
	 * Converts a GAMA rate of turn to a ROS angular velocity around z.
	 *
	 * @param turnRate
	 *            degrees per second, clockwise on GAMA's screen
	 * @return radians per second, counter-clockwise
	 */
	public static double toRosAngularZ(final double turnRate) {
		return -Math.toRadians(turnRate);
	}

	/**
	 * Converts a ROS angular velocity around z to a GAMA rate of turn.
	 *
	 * @param angularZ
	 *            radians per second, counter-clockwise
	 * @return degrees per second, clockwise on GAMA's screen
	 */
	public static double toGamaTurnRate(final double angularZ) {
		return -Math.toDegrees(angularZ);
	}

	// ---------------------------------------------------------------------------------------------------------------
	// Time
	// ---------------------------------------------------------------------------------------------------------------

	/**
	 * Returns the current wall-clock time as a ROS stamp.
	 *
	 * <p>
	 * Never GAMA's {@code time}: it restarts at 0 on every run, and tf2 cannot survive stamps that go backwards.
	 * </p>
	 *
	 * @return seconds and nanoseconds since the UNIX epoch
	 */
	public static long[] nowAsStamp() {
		final java.time.Instant now = java.time.Instant.now();
		return new long[] { now.getEpochSecond(), now.getNano() };
	}

	// ---------------------------------------------------------------------------------------------------------------
	// Helpers
	// ---------------------------------------------------------------------------------------------------------------

	/**
	 * Builds a map in the shape of geometry_msgs/Point or Vector3.
	 *
	 * @param x
	 *            the x
	 * @param y
	 *            the y
	 * @param z
	 *            the z
	 * @return a new mutable map
	 */
	public static IMap<String, Object> xyz(final double x, final double y, final double z) {
		final IMap<String, Object> m = GamaMapFactory.create(Types.STRING, Types.FLOAT);
		m.put("x", x);
		m.put("y", y);
		m.put("z", z);
		return m;
	}

	/**
	 * Reads a number from a message map, 0 when absent, the default ROS gives every numeric field.
	 *
	 * @param m
	 *            the map
	 * @param key
	 *            the field
	 * @return the value
	 */
	public static double number(final Map<?, ?> m, final String key) {
		return m.get(key) instanceof final Number n ? n.doubleValue() : 0;
	}

	/**
	 * Returns the simulation of the scope, which owns the placement.
	 *
	 * @param scope
	 *            the scope
	 * @return the simulation
	 */
	private static ISimulationAgent simulationOf(final IScope scope) {
		final ISimulationAgent sim = scope == null ? null : scope.getSimulation();
		if (sim == null) throw GamaRuntimeException
				.error("The ROS frame of the world can only be used from within a simulation, which owns it.", scope);
		return sim;
	}

	/**
	 * Returns the envelope of the world of the current simulation.
	 *
	 * @param scope
	 *            the scope
	 * @return the envelope
	 */
	private static IEnvelope envelopeOf(final IScope scope) {
		return simulationOf(scope).getGeometry(scope).getEnvelope();
	}

	/**
	 * @param env
	 *            an envelope
	 * @return the x of its centre
	 */
	private static double centreX(final IEnvelope env) {
		return (env.getMinX() + env.getMaxX()) / 2;
	}

	/**
	 * @param env
	 *            an envelope
	 * @return the y of its centre
	 */
	private static double centreY(final IEnvelope env) {
		return (env.getMinY() + env.getMaxY()) / 2;
	}

}
