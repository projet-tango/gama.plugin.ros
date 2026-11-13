/*******************************************************************************************************
 *
 * RosMovingSkill.java, in gama.plugin.ros, is part of the source code of the GAMA modeling and simulation platform.
 *
 * (c) 2007-2026 UMI 209 UMMISCO IRD/SU & Partners (IRIT, MIAT, ESPACE-DEV, CTU)
 *
 * Visit https://github.com/TANGO-projet/gama.plugin.ros for license information and contacts.
 *
 ********************************************************************************************************/
package gama.plugin.ros.skills;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.linearref.LengthIndexedLine;

import gama.annotations.action;
import gama.annotations.arg;
import gama.annotations.doc;
import gama.annotations.example;
import gama.annotations.getter;
import gama.annotations.setter;
import gama.annotations.skill;
import gama.annotations.variable;
import gama.annotations.vars;
import gama.annotations.constants.IKeyword;
import gama.annotations.support.IConcept;
import gama.api.exceptions.GamaRuntimeException;
import gama.api.gaml.types.IType;
import gama.api.kernel.agent.IAgent;
import gama.api.runtime.scope.IScope;
import gama.api.types.color.IColor;
import gama.api.types.geometry.GamaPointFactory;
import gama.api.types.geometry.IPoint;
import gama.api.types.geometry.IShape;
import gama.api.types.graph.IPath;
import gama.api.types.map.GamaMapFactory;
import gama.api.types.map.IMap;
import gama.api.gaml.types.Types;
import gama.gaml.skills.MovingSkill;
import gama.plugin.ros.RosConstants;
import gama.plugin.ros.types.GamaROSNode;
import gama.plugin.ros.types.GamaROSPublisher;
import gama.plugin.ros.types.GamaROSSubscription;
import gama.plugin.ros.utils.RosRobot;
import gama.plugin.ros.utils.RosRobots;

/**
 * A drop-in replacement for the moving skill that can be bound to a ROS robot.
 *
 * <p>
 * Without a {@code ros_name}, an agent is a <b>mirror</b>: every action is the one of the moving skill, so the model
 * behaves exactly as before, and the agent is also drawn in RViz. With a {@code ros_name}, it is <b>embodied</b>: the
 * same actions compute a velocity command for the robot instead of moving the agent, and {@code location},
 * {@code heading} and {@code real_speed} report what the robot actually did.
 * </p>
 *
 * <h3>How the actions steer a robot</h3>
 * <p>
 * Each action produces a point to aim at, then one proportional controller turns it into a command: the rate of turn
 * is the heading error times {@code turn_gain}, the speed is the asked one, both are clamped by the profile, and a
 * robot aimed more than {@code turn_in_place_angle} away first turns, nearly on the spot. {@code goto} also slows down
 * near its target and stops within {@code arrival}. {@code goto ... on:} and {@code wander ... on:} let the moving
 * skill compute where it would go along the graph, and aim there, so graphs, grids and weights keep working.
 * </p>
 *
 * <p>
 * The command is sent at the end of the step. An agent that calls no action during a step is sent a stop, as an
 * agent of the moving skill does not move when it is not asked to.
 * </p>
 */
@vars ({ @variable (
		name = IKeyword.LOCATION,
		type = IType.POINT,
		depends_on = IKeyword.SHAPE,
		doc = @doc ("Represents the current position of the agent. For an agent bound to a robot, the position the robot reports, updated at the end of every step.")),
		@variable (
		name = RosRobot.NAME,
		type = IType.STRING,
		init = "nil",
		doc = @doc ("The name of the ROS robot this agent drives, used in the topic names of its profile ({name} and {ns}). nil, the default, binds no robot: the agent moves like with the moving skill and is only shown in ROS. The empty string binds a robot whose topics are not namespaced, such as /cmd_vel.")),
		@variable (
				name = RosRobot.PROFILE,
				type = IType.STRING,
				init = "'generic'",
				doc = @doc ("The kind of robot: 'generic' (cmd_vel and odom), 'gazebo' (Gazebo Sim through ros_gz_bridge), 'alphabot2' (Waveshare AlphaBot2), or any profile defined with ros_define_profile. ros_profile(name) shows what one holds.")),
		@variable (
				name = RosRobot.SETTINGS,
				type = IType.MAP,
				init = "nil",
				doc = @doc ("Settings of this agent that override its profile, e.g. [\"max_speed\"::0.2, \"cmd_topic\"::\"/bot/cmd_vel_safe\"]. A key that is not a setting is an error listing those that exist.")),
		@variable (
				name = RosRobot.NODE,
				type = RosConstants.NODE_ID,
				init = "nil",
				doc = @doc ("The ROS node the topics of this agent are created on. nil uses the node named by the 'node' setting, shared by every agent.")),
		@variable (
				name = RosRobot.MARKER,
				type = IType.STRING,
				init = "'arrow'",
				doc = @doc ("How the agent is drawn in RViz, on the marker_topic of its profile: 'arrow', 'cube', 'sphere', 'cylinder', or 'none'.")),
		@variable (
				name = RosRobot.COLOR,
				type = IType.COLOR,
				init = "nil",
				doc = @doc ("The colour of the marker of the agent in RViz. nil gives steelblue.")),
		@variable (
				name = "ros_connected",
				type = IType.BOOL,
				init = "false",
				doc = @doc ("Read only. For an embodied agent, whether a pose arrived less than 'timeout' seconds ago, or, when its pose is estimated, whether anything listens to its commands. For a mirror agent, whether anything listens to its markers.")),
		@variable (
				name = "ros_estimated",
				type = IType.BOOL,
				init = "false",
				doc = @doc ("Read only. Whether the pose of the robot is estimated from the commands sent, because its profile names neither a pose nor an odometry topic. An estimated pose can be set: writing location and heading says where the robot was put down.")),
		@variable (
				name = "ros_cmd_speed",
				type = IType.FLOAT,
				init = "0.0",
				doc = @doc ("Read only. The forward speed last sent to the robot, in m/s. Compare with real_speed, which is the measured one.")),
		@variable (
				name = "ros_cmd_turn_rate",
				type = IType.FLOAT,
				init = "0.0",
				doc = @doc ("Read only. The rate of turn last sent to the robot, in degrees/s, positive clockwise like headings.")),
		@variable (
				name = "ros_config",
				type = IType.MAP,
				init = "nil",
				doc = @doc ("Read only. The settings in force for this agent: its profile, its ros_settings, and the topic names resolved.")),
		@variable (
				name = "ros_cmd_publisher",
				type = RosConstants.PUBLISHER_ID,
				init = "nil",
				doc = @doc ("Read only. The publisher of the commands, for the operators of the plugin (ros_subscriber_count...). nil for a mirror agent.")),
		@variable (
				name = "ros_pose_subscription",
				type = RosConstants.SUBSCRIPTION_ID,
				init = "nil",
				doc = @doc ("Read only. The subscription giving the pose, nil if the profile names no pose_topic.")),
		@variable (
				name = "ros_odom_subscription",
				type = RosConstants.SUBSCRIPTION_ID,
				init = "nil",
				doc = @doc ("Read only. The subscription giving the odometry, nil if the profile names no odom_topic.")) })
@skill (
		name = RosMovingSkill.NAME,
		concept = { IConcept.SKILL, IConcept.AGENT_MOVEMENT, RosConstants.CONCEPT })
@doc ("A drop-in replacement for the moving skill, bound to a ROS robot when the agent has a ros_name. Without one, the agent moves exactly as with moving and is drawn in RViz. With one, move, wander, goto and follow drive the robot through its velocity command, and location, heading and real_speed report where it really is. What the robot is, its topics, limits and controller gains, comes from its ros_profile, overridden by ros_settings.")
@SuppressWarnings ({ "rawtypes" })
public class RosMovingSkill extends MovingSkill {

	/** The GAML name of the skill. */
	public static final String NAME = "ros_moving";

	// ---------------------------------------------------------------------------------------------------------------
	// Configuration variables
	// ---------------------------------------------------------------------------------------------------------------

	/**
	 * Registers the agent as soon as it is created, so that a mirror agent that never calls an action is still drawn.
	 */
	@Override
	@getter (
			value = IKeyword.LOCATION,
			initializer = true)
	public IPoint getLocation(final IAgent agent) {
		if (agent == null) return null;
		RosRobot.of(agent);
		return super.getLocation(agent);
	}

	/**
	 * @param agent
	 *            the agent
	 * @return its ros_name
	 */
	@getter (RosRobot.NAME)
	public String getRosName(final IAgent agent) {
		return (String) agent.getAttribute(RosRobot.NAME);
	}

	/**
	 * @param agent
	 *            the agent
	 * @param name
	 *            the new ros_name
	 */
	@setter (RosRobot.NAME)
	public void setRosName(final IAgent agent, final String name) {
		configure(agent, RosRobot.NAME, name);
	}

	/**
	 * @param agent
	 *            the agent
	 * @return its ros_profile
	 */
	@getter (RosRobot.PROFILE)
	public String getRosProfile(final IAgent agent) {
		return (String) agent.getAttribute(RosRobot.PROFILE);
	}

	/**
	 * @param agent
	 *            the agent
	 * @param profile
	 *            the new ros_profile
	 */
	@setter (RosRobot.PROFILE)
	public void setRosProfile(final IAgent agent, final String profile) {
		configure(agent, RosRobot.PROFILE, profile);
	}

	/**
	 * @param agent
	 *            the agent
	 * @return its ros_settings
	 */
	@getter (RosRobot.SETTINGS)
	public IMap getRosSettings(final IAgent agent) {
		return (IMap) agent.getAttribute(RosRobot.SETTINGS);
	}

	/**
	 * @param agent
	 *            the agent
	 * @param settings
	 *            the new ros_settings
	 */
	@setter (RosRobot.SETTINGS)
	public void setRosSettings(final IAgent agent, final IMap settings) {
		configure(agent, RosRobot.SETTINGS, settings);
	}

	/**
	 * @param agent
	 *            the agent
	 * @return its ros_node
	 */
	@getter (RosRobot.NODE)
	public GamaROSNode getRosNode(final IAgent agent) {
		return (GamaROSNode) agent.getAttribute(RosRobot.NODE);
	}

	/**
	 * @param agent
	 *            the agent
	 * @param node
	 *            the new ros_node
	 */
	@setter (RosRobot.NODE)
	public void setRosNode(final IAgent agent, final GamaROSNode node) {
		configure(agent, RosRobot.NODE, node);
	}

	/**
	 * @param agent
	 *            the agent
	 * @return its ros_marker
	 */
	@getter (RosRobot.MARKER)
	public String getRosMarker(final IAgent agent) {
		return (String) agent.getAttribute(RosRobot.MARKER);
	}

	/**
	 * @param agent
	 *            the agent
	 * @param marker
	 *            the new ros_marker
	 */
	@setter (RosRobot.MARKER)
	public void setRosMarker(final IAgent agent, final String marker) {
		agent.setAttribute(RosRobot.MARKER, marker);
		RosRobot.of(agent);
	}

	/**
	 * @param agent
	 *            the agent
	 * @return its ros_color
	 */
	@getter (RosRobot.COLOR)
	public IColor getRosColor(final IAgent agent) {
		return (IColor) agent.getAttribute(RosRobot.COLOR);
	}

	/**
	 * @param agent
	 *            the agent
	 * @param color
	 *            the new ros_color
	 */
	@setter (RosRobot.COLOR)
	public void setRosColor(final IAgent agent, final IColor color) {
		agent.setAttribute(RosRobot.COLOR, color);
	}

	/**
	 * Stores a configuration variable and says that the robot has to be rebuilt.
	 *
	 * @param agent
	 *            the agent
	 * @param key
	 *            the variable
	 * @param value
	 *            its value
	 */
	private static void configure(final IAgent agent, final String key, final Object value) {
		if (agent == null) return;
		agent.setAttribute(key, value);
		RosRobot.of(agent).markDirty();
	}

	/**
	 * @param agent
	 *            the agent
	 * @return its robot, ready to use
	 */
	private static RosRobot ready(final IAgent agent) {
		final RosRobot r = RosRobot.of(agent);
		r.ensureReady(agent.getScope());
		return r;
	}

	// ---------------------------------------------------------------------------------------------------------------
	// Read-only variables
	// ---------------------------------------------------------------------------------------------------------------

	/**
	 * @param agent
	 *            the agent
	 * @return whether the other side answers
	 */
	@getter ("ros_connected")
	public Boolean getRosConnected(final IAgent agent) {
		final RosRobot r = ready(agent);
		return r.isEmbodied() ? r.isConnected() : RosRobots.of(agent.getScope()).markersListened(r);
	}

	/**
	 * @param agent
	 *            the agent
	 * @return whether its pose is estimated
	 */
	@getter ("ros_estimated")
	public Boolean getRosEstimated(final IAgent agent) {
		return ready(agent).isEstimated();
	}

	/**
	 * @param agent
	 *            the agent
	 * @return the speed last sent
	 */
	@getter ("ros_cmd_speed")
	public Double getRosCmdSpeed(final IAgent agent) {
		return ready(agent).commandedSpeed();
	}

	/**
	 * @param agent
	 *            the agent
	 * @return the rate of turn last sent
	 */
	@getter ("ros_cmd_turn_rate")
	public Double getRosCmdTurnRate(final IAgent agent) {
		return ready(agent).commandedTurnRate();
	}

	/**
	 * @param agent
	 *            the agent
	 * @return a copy of the settings in force
	 */
	@getter ("ros_config")
	public IMap getRosConfig(final IAgent agent) {
		final IMap<String, Object> copy = GamaMapFactory.create(Types.STRING, Types.NO_TYPE);
		copy.putAll(ready(agent).settings());
		return copy;
	}

	/**
	 * @param agent
	 *            the agent
	 * @return the command publisher
	 */
	@getter ("ros_cmd_publisher")
	public GamaROSPublisher getRosCmdPublisher(final IAgent agent) {
		return ready(agent).commandPublisher();
	}

	/**
	 * @param agent
	 *            the agent
	 * @return the pose subscription
	 */
	@getter ("ros_pose_subscription")
	public GamaROSSubscription getRosPoseSubscription(final IAgent agent) {
		return ready(agent).poseSubscription();
	}

	/**
	 * @param agent
	 *            the agent
	 * @return the odometry subscription
	 */
	@getter ("ros_odom_subscription")
	public GamaROSSubscription getRosOdomSubscription(final IAgent agent) {
		return ready(agent).odomSubscription();
	}

	// ---------------------------------------------------------------------------------------------------------------
	// The actions of the moving skill
	// ---------------------------------------------------------------------------------------------------------------

	@Override
	@action (
			name = "move",
			args = { @arg (
					name = IKeyword.SPEED,
					type = IType.FLOAT,
					optional = true,
					doc = @doc ("the speed to use for this move (replaces the current value of speed)")),
					@arg (
							name = IKeyword.HEADING,
							type = IType.FLOAT,
							optional = true,
							doc = @doc ("the angle (in degree) of the target direction.")),
					@arg (
							name = IKeyword.BOUNDS,
							type = IType.GEOMETRY,
							optional = true,
							doc = @doc ("the geometry (the localized entity geometry) that restrains this move (the agent moves inside this geometry")) },
			doc = @doc (
					examples = { @example ("do move speed: speed - 10 heading: heading + rnd (30) bounds: agentA;") },
					value = "moves the agent forward, the distance being computed with respect to its speed and heading. The value of the corresponding variables are used unless arguments are passed. For an agent bound to a robot, sends the robot forward, turning towards the heading if one is given."))
	public IPath primMoveForward(final IScope scope) throws GamaRuntimeException {
		final IAgent agent = getCurrentAgent(scope);
		final RosRobot r = ready(agent);
		if (!r.isEmbodied()) return super.primMoveForward(scope);
		final double speed = askedSpeed(scope, agent);
		if (scope.hasArg(IKeyword.HEADING)) {
			steer(r, scope.getFloatArg(IKeyword.HEADING), speed);
		} else {
			r.command(speed, 0);
		}
		return null;
	}

	@Override
	@action (
			name = "wander",
			args = { @arg (
					name = IKeyword.SPEED,
					type = IType.FLOAT,
					optional = true,
					doc = @doc ("the speed to use for this move (replaces the current value of speed)")),
					@arg (
							name = "amplitude",
							type = IType.FLOAT,
							optional = true,
							doc = @doc ("a restriction placed on the random heading choice. The new heading is chosen in the range (heading - amplitude/2, heading+amplitude/2)")),
					@arg (
							name = IKeyword.BOUNDS,
							type = IType.GEOMETRY,
							optional = true,
							doc = @doc ("the geometry (the localized entity geometry) that restrains this move (the agent moves inside this geometry)")),
					@arg (
							name = IKeyword.ON,
							type = IType.GRAPH,
							optional = true,
							doc = @doc ("the graph that restrains this move (the agent moves on the graph")),
					@arg (
							name = "proba_edges",
							type = IType.MAP,
							optional = true,
							doc = @doc ("When the agent moves on a graph, the probability to choose another edge. If not defined, each edge has the same probability to be chosen")) },
			doc = @doc (
					examples = { @example ("do wander speed: speed - 10 amplitude: 120 bounds: square(50);") },
					value = "Moves the agent towards a random location at the maximum distance (with respect to its speed). The heading of the agent is chosen randomly if no amplitude is specified. This action changes the value of heading. For an agent bound to a robot, the robot steers towards a heading that wanders the same way, and turns back at the bounds, or at the edge of the world."))
	public boolean primMoveRandomly(final IScope scope) throws GamaRuntimeException {
		final IAgent agent = getCurrentAgent(scope);
		final RosRobot r = ready(agent);
		if (!r.isEmbodied()) return super.primMoveRandomly(scope);
		final double speed = askedSpeed(scope, agent);
		if (scope.getArg(IKeyword.ON, IType.NONE) != null) {
			steerToPoint(r, carrot(scope, agent, () -> super.primMoveRandomly(scope)), speed, null);
			return true;
		}
		// The moving skill lets the heading do a random walk. The measured heading is overwritten at every step, so
		// the walk is kept apart and the robot is steered towards it.
		final double amplitude = scope.hasArg("amplitude") ? scope.getFloatArg("amplitude") : 359;
		double h = r.wanderHeading() + scope.getRandom().between(-amplitude / 2.0, amplitude / 2.0);
		final IShape bounds = scope.hasArg(IKeyword.BOUNDS) && scope.getArg(IKeyword.BOUNDS, IType.NONE) != null
				? (IShape) scope.getArg(IKeyword.BOUNDS, IType.GEOMETRY) : scope.getSimulation().getGeometry(scope);
		if (bounds != null) {
			final IPoint ahead = ahead(agent.getLocation(), h, r.num("lookahead"));
			if (bounds.euclidianDistanceTo(ahead) > 0) { h += 180; }
		}
		r.setWanderHeading(h);
		steer(r, h, speed);
		return true;
	}

	@Override
	@action (
			name = "follow",
			args = { @arg (
					name = IKeyword.SPEED,
					type = IType.FLOAT,
					optional = true,
					doc = @doc ("the speed to use for this move (replaces the current value of speed)")),
					@arg (
							name = "path",
							type = IType.PATH,
							optional = false,
							doc = @doc ("a path to be followed.")),
					@arg (
							name = "move_weights",
							type = IType.MAP,
							optional = true,
							doc = @doc ("Weights used for the moving.")),
					@arg (
							name = "return_path",
							type = IType.BOOL,
							optional = true,
							doc = @doc ("if true, return the path followed (by default: false)")) },
			doc = @doc (
					value = "moves the agent along a given path passed in the arguments. For an agent bound to a robot, the robot aims at the point of the path 'lookahead' metres beyond the point closest to it, and stops at its end.",
					returns = "optional: the path followed by the agent; nil for an agent bound to a robot.",
					examples = { @example ("do follow speed: speed * 2 path: road_path;") }))
	public IPath primFollow(final IScope scope) throws GamaRuntimeException {
		final IAgent agent = getCurrentAgent(scope);
		final RosRobot r = ready(agent);
		if (!r.isEmbodied()) return super.primFollow(scope);
		final IPath path = (IPath) scope.getArg("path", IType.PATH);
		final IShape line = path == null ? null : path.getGeometry();
		final Geometry g = line == null ? null : line.getInnerGeometry();
		if (g == null || g.isEmpty()) {
			r.command(0, 0);
			return null;
		}
		final LengthIndexedLine indexed = new LengthIndexedLine(g);
		final IPoint loc = agent.getLocation();
		final double at = indexed.project(new Coordinate(loc.getX(), loc.getY()));
		final double left = indexed.getEndIndex() - at;
		if (left < r.num("arrival")) {
			r.command(0, 0);
			return null;
		}
		final Coordinate c = indexed.extractPoint(Math.min(indexed.getEndIndex(), at + r.num("lookahead")));
		steerToPoint(r, GamaPointFactory.create(c.x, c.y), askedSpeed(scope, agent), left);
		return null;
	}

	@Override
	@action (
			name = "goto",
			args = { @arg (
					name = "target",
					type = IType.GEOMETRY,
					optional = false,
					doc = @doc ("the location or entity towards which to move.")),
					@arg (
							name = IKeyword.SPEED,
							type = IType.FLOAT,
							optional = true,
							doc = @doc ("the speed to use for this move (replaces the current value of speed)")),
					@arg (
							name = "on",
							type = IType.NONE,
							optional = true,
							doc = @doc ("graph, topology, list of geometries or map of geometries that restrain this move")),
					@arg (
							name = "recompute_path",
							type = IType.BOOL,
							optional = true,
							doc = @doc ("if false, the path is not recompute even if the graph is modified (by default: true)")),
					@arg (
							name = "return_path",
							type = IType.BOOL,
							optional = true,
							doc = @doc ("if true, return the path followed (by default: false)")),
					@arg (
							name = "move_weights",
							type = IType.MAP,
							optional = true,
							doc = @doc ("Weights used for the moving.")) },
			doc = @doc (
					value = "moves the agent towards the target passed in the arguments. For an agent bound to a robot, the robot steers towards the target, or along the path the moving skill computes on 'on', slows down when close and stops within 'arrival' metres.",
					returns = "optional: the path followed by the agent; nil for an agent bound to a robot.",
					examples = { @example ("do goto target: (one_of road).location speed: speed * 2 on: road_network;") }))
	public IPath primGoto(final IScope scope) throws GamaRuntimeException {
		final IAgent agent = getCurrentAgent(scope);
		final RosRobot r = ready(agent);
		if (!r.isEmbodied()) return super.primGoto(scope);
		final IShape target = computeTarget(scope, agent);
		if (target == null) {
			r.command(0, 0);
			return null;
		}
		final IPoint goal = target.getLocation();
		final double distance = agent.getLocation().euclidianDistanceTo(goal);
		if (distance < r.num("arrival")) {
			r.command(0, 0);
			return null;
		}
		IPoint aim = goal;
		if (scope.getArg("on", IType.NONE) != null) {
			final IPoint along = carrot(scope, agent, () -> super.primGoto(scope));
			if (along != null && along.euclidianDistanceTo(agent.getLocation()) > 1e-6) { aim = along; }
		}
		steerToPoint(r, aim, askedSpeed(scope, agent), distance);
		return null;
	}

	/**
	 * Stops the robot now.
	 *
	 * @param scope
	 *            the scope
	 * @return true
	 */
	@action (
			name = "ros_stop",
			doc = @doc (
					value = "Sends a stop to the robot at once, without waiting for the end of the step. Does nothing for an agent bound to no robot.",
					examples = { @example ("ask bot { do ros_stop; }") }))
	public Boolean primStop(final IScope scope) {
		final RosRobot r = ready(getCurrentAgent(scope));
		if (r.isEmbodied()) { r.stopNow(scope); }
		return true;
	}

	// ---------------------------------------------------------------------------------------------------------------
	// Steering
	// ---------------------------------------------------------------------------------------------------------------

	/**
	 * @param scope
	 *            the scope
	 * @param agent
	 *            the agent
	 * @return the speed asked for: the argument, else the speed variable
	 */
	private static double askedSpeed(final IScope scope, final IAgent agent) {
		if (scope.hasArg(IKeyword.SPEED)) return scope.getFloatArg(IKeyword.SPEED);
		final Double s = getSpeed(agent);
		return s == null ? 0 : s;
	}

	/**
	 * Steers towards a point.
	 *
	 * @param r
	 *            the robot
	 * @param aim
	 *            the point
	 * @param speed
	 *            the speed asked for
	 * @param remaining
	 *            the distance left to the goal, to slow down near it; null for no slowing down
	 */
	private static void steerToPoint(final RosRobot r, final IPoint aim, final double speed, final Double remaining) {
		final IPoint loc = r.agent().getLocation();
		final double dx = aim.getX() - loc.getX(), dy = aim.getY() - loc.getY();
		if (dx * dx + dy * dy < 1e-12) {
			r.command(0, 0);
			return;
		}
		// y points down in GAMA, so atan2 gives a heading in GAMA's own clockwise convention
		final double bearing = Math.toDegrees(Math.atan2(dy, dx));
		double v = speed;
		if (remaining != null) { v = Math.min(v, remaining * r.num("slowdown_gain")); }
		steer(r, bearing, v);
	}

	/**
	 * The controller: proportional on the heading error, turning first when badly aimed.
	 *
	 * @param r
	 *            the robot
	 * @param heading
	 *            the heading wanted, degrees
	 * @param speed
	 *            the speed wanted, m/s
	 */
	private static void steer(final RosRobot r, final double heading, final double speed) {
		final double error = ((heading - r.currentHeading()) % 360 + 540) % 360 - 180;
		final double turn = error * r.num("turn_gain");
		final double v = Math.abs(error) > r.num("turn_in_place_angle") ? speed * r.num("turn_in_place_speed") : speed;
		r.command(v, turn);
	}

	/**
	 * Lets the moving skill move the agent as it would, notes where it ended up, and puts everything back: the
	 * robot will be steered there instead.
	 *
	 * @param scope
	 *            the scope
	 * @param agent
	 *            the agent
	 * @param move
	 *            the action of the moving skill
	 * @return where the moving skill would have taken the agent
	 */
	private static IPoint carrot(final IScope scope, final IAgent agent, final java.util.function.Supplier<?> move) {
		final IPoint location = agent.getLocation().copy(scope);
		final Object heading = agent.getAttribute(IKeyword.HEADING);
		final Object realSpeed = agent.getAttribute(IKeyword.REAL_SPEED);
		try {
			move.get();
			return agent.getLocation().copy(scope);
		} finally {
			agent.setLocation(location);
			agent.setAttribute(IKeyword.HEADING, heading);
			agent.setAttribute(IKeyword.REAL_SPEED, realSpeed);
		}
	}

	/**
	 * @param from
	 *            a point
	 * @param heading
	 *            degrees, GAMA's convention
	 * @param distance
	 *            metres
	 * @return the point at that distance in that direction
	 */
	private static IPoint ahead(final IPoint from, final double heading, final double distance) {
		final double a = Math.toRadians(heading);
		return GamaPointFactory.create(from.getX() + Math.cos(a) * distance, from.getY() + Math.sin(a) * distance,
				from.getZ());
	}

}
