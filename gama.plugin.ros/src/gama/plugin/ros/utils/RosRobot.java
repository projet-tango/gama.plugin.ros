/*******************************************************************************************************
 *
 * RosRobot.java, in gama.plugin.ros, is part of the source code of the GAMA modeling and simulation platform.
 *
 * (c) 2007-2026 UMI 209 UMMISCO IRD/SU & Partners (IRIT, MIAT, ESPACE-DEV, CTU)
 *
 * Visit https://github.com/TANGO-projet/gama.plugin.ros for license information and contacts.
 *
 ********************************************************************************************************/
package gama.plugin.ros.utils;

import java.util.Map;

import gama.annotations.constants.IKeyword;
import gama.api.GAMA;
import gama.api.exceptions.GamaRuntimeException;
import gama.api.gaml.types.Types;
import gama.api.kernel.agent.IAgent;
import gama.api.runtime.scope.IScope;
import gama.api.types.color.GamaColorFactory;
import gama.api.types.color.IColor;
import gama.api.types.geometry.GamaPointFactory;
import gama.api.types.geometry.IPoint;
import gama.api.types.map.GamaMapFactory;
import gama.api.types.map.IMap;
import gama.plugin.ros.RosGeometryOperators;
import gama.plugin.ros.RosOperators;
import gama.plugin.ros.types.GamaROSNode;
import gama.plugin.ros.types.GamaROSPublisher;
import gama.plugin.ros.types.GamaROSSubscription;
import gama.plugin.ros.types.GamaROSTopic;

/**
 * What the {@code ros_moving} skill knows about one agent: its settings, its topics, the command it decided and the
 * pose that came back.
 *
 * <h3>Two modes</h3>
 * <ul>
 * <li><b>mirror</b>, when the agent has no {@code ros_name}: GAMA moves it exactly as the moving skill would, and it is
 * only shown in ROS, as a marker;</li>
 * <li><b>embodied</b>, when it has one (the empty string included, for a robot whose topics are not namespaced): a
 * real or simulated robot moves, the actions of the skill only decide its velocity command, and {@code location} and
 * {@code heading} report where the robot actually is.</li>
 * </ul>
 *
 * <h3>Where the pose comes from</h3>
 * <p>
 * From {@code pose_topic} if the settings name one, else from {@code odom_topic}, else it is <b>estimated</b> by
 * integrating the commands, which is all a robot without encoders nor tracking, like the AlphaBot2, allows. An
 * estimated pose can be written by the model: setting {@code location} and {@code heading} tells GAMA where the robot
 * was put down.
 * </p>
 *
 * <h3>Threads</h3>
 * <p>
 * Everything runs on the simulation thread, except the watchdog of {@link RosRobots}, which only ever sends a stop.
 * The command state is guarded by the instance lock for that reason.
 * </p>
 */
public final class RosRobot {

	/** The attribute under which an agent keeps its robot. */
	private static final String KEY = "$ros_robot";

	/** The GAML names of the variables of the skill that configure the robot. */
	public static final String NAME = "ros_name", PROFILE = "ros_profile", SETTINGS = "ros_settings", NODE = "ros_node",
			MARKER = "ros_marker", COLOR = "ros_color";

	/** The colour of markers whose agent sets none: steelblue, as in the models. */
	private static final IColor DEFAULT_COLOR = GamaColorFactory.get(70, 130, 180);

	/** The agent. */
	private final IAgent agent;

	/** Set when a variable that configures the robot changed: the topics are rebuilt before the next use. */
	private volatile boolean dirty = true;

	/** The resolved settings. */
	private IMap<String, Object> settings;

	/** Whether a robot is bound, i.e. whether the agent has a ros_name. */
	private boolean embodied;

	/** The node, the agent's own or the shared one. */
	private GamaROSNode node;

	/** The command publisher, null in mirror mode. */
	private GamaROSPublisher cmd;

	/** Whether the command is a TwistStamped rather than a Twist. */
	private boolean stamped;

	/** The subscriptions giving the pose and the measured speed; either or both may be null. */
	private GamaROSSubscription pose, odom;

	// --- the command, guarded by this ---------------------------------------------------------------------------

	/** What the actions decided during the current step. */
	private double wantedSpeed, wantedTurn;

	/** Whether an action decided anything during the current step. */
	private boolean commanded;

	/** What was last sent, i.e. what the robot is doing now. */
	private double activeSpeed, activeTurn;

	/** Whether what was last sent is a stop (or nothing was ever sent). */
	private boolean stopped = true;

	/** When the last command was sent, System.nanoTime. */
	private long lastSent;

	// --- the state ------------------------------------------------------------------------------------------------

	/** When a pose last arrived, -1 if never. */
	private long lastPose = -1;

	/** When the estimated pose was last integrated, -1 if never. */
	private long lastIntegration = -1;

	/** When the robot was bound, for the hint. */
	private long boundAt;

	/** Whether the hint has been printed. */
	private boolean hinted;

	/** The heading wander steers towards: a random walk of its own, since the measured heading is overwritten. */
	private Double wanderHeading;

	/** The previous measured position and its date, to derive a speed when there is no odometry. */
	private IPoint previousPosition;

	/** The date of the previous measured position. */
	private long previousPositionAt;

	/**
	 * @param agent
	 *            the agent
	 */
	private RosRobot(final IAgent agent) {
		this.agent = agent;
	}

	/**
	 * Returns the robot of an agent, creating it and registering it with its simulation on first use.
	 *
	 * @param agent
	 *            an agent with the ros_moving skill
	 * @return its robot
	 */
	public static RosRobot of(final IAgent agent) {
		if (agent.getAttribute(KEY) instanceof final RosRobot r) return r;
		final RosRobot r = new RosRobot(agent);
		agent.setAttribute(KEY, r);
		RosRobots.of(agent.getScope()).add(r);
		return r;
	}

	/**
	 * Says that the configuration changed.
	 */
	public void markDirty() {
		dirty = true;
	}

	/**
	 * Resolves the settings and (re)creates the topics if the configuration changed since the last call.
	 *
	 * @param scope
	 *            the scope
	 */
	public synchronized void ensureReady(final IScope scope) {
		if (!dirty) return;
		// a robot reconfigured while moving is stopped on its old topic before that topic goes away
		if (embodied && !stopped) { send(scope, 0, 0); }
		closeTopics();
		final Object name = agent.getAttribute(NAME);
		final Object profile = agent.getAttribute(PROFILE);
		final Object overrides = agent.getAttribute(SETTINGS);
		// resolved first: a misspelt setting raises here, and the robot stays dirty, so it raises again next time
		// rather than running on what was there before
		final IMap<String, Object> resolved = RosProfiles.resolve(scope, profile == null ? null : profile.toString(),
				overrides instanceof final Map<?, ?> m ? m : null, name == null ? null : name.toString());
		dirty = false;
		embodied = name != null;
		settings = resolved;
		node = agent.getAttribute(NODE) instanceof final GamaROSNode n ? n
				: RosNodes.get(scope, str("node"), (int) num("domain"));
		boundAt = System.nanoTime();
		hinted = false;
		lastPose = -1;
		lastIntegration = -1;
		wanderHeading = null;
		if (!embodied) return;
		final String cmdType = str("cmd_type");
		stamped = cmdType.endsWith("TwistStamped");
		cmd = new GamaROSPublisher(scope, node, topic(scope, "cmd_topic", cmdType, "cmd_qos"));
		if (!str("pose_topic").isEmpty()) {
			pose = new GamaROSSubscription(scope, node, topic(scope, "pose_topic", str("pose_type"), "pose_qos"), 1);
		}
		if (!str("odom_topic").isEmpty()) {
			odom = new GamaROSSubscription(scope, node,
					topic(scope, "odom_topic", "nav_msgs/msg/Odometry", "odom_qos"), 1);
		}
	}

	/**
	 * @param scope
	 *            the scope
	 * @param topicKey
	 *            the setting holding the name
	 * @param type
	 *            the message type
	 * @param qosKey
	 *            the setting holding the QoS
	 * @return the topic
	 */
	private GamaROSTopic topic(final IScope scope, final String topicKey, final String type, final String qosKey) {
		return new GamaROSTopic(scope, str(topicKey), type, RosOperators.qosNamed(scope, str(qosKey)));
	}

	// ---------------------------------------------------------------------------------------------------------------
	// Read by the skill
	// ---------------------------------------------------------------------------------------------------------------

	/** @return the agent */
	public IAgent agent() { return agent; }

	/** @return whether a robot is bound */
	public boolean isEmbodied() { return embodied; }

	/** @return whether the pose is estimated from the commands */
	public boolean isEstimated() { return embodied && pose == null && odom == null; }

	/** @return the resolved settings, read-only by convention */
	public IMap<String, Object> settings() { return settings; }

	/** @return the node */
	public GamaROSNode node() { return node; }

	/** @return the command publisher, or null */
	public GamaROSPublisher commandPublisher() { return cmd; }

	/** @return the pose subscription, or null */
	public GamaROSSubscription poseSubscription() { return pose; }

	/** @return the odometry subscription, or null */
	public GamaROSSubscription odomSubscription() { return odom; }

	/** @return the speed last sent, m/s */
	public synchronized double commandedSpeed() { return activeSpeed; }

	/** @return the rate of turn last sent, degrees/s in GAMA's convention */
	public synchronized double commandedTurnRate() { return activeTurn; }

	/**
	 * @param key
	 *            a setting
	 * @return its value
	 */
	public double num(final String key) {
		return RosProfiles.num(settings, key);
	}

	/**
	 * @param key
	 *            a setting
	 * @return its value
	 */
	public String str(final String key) {
		return RosProfiles.str(settings, key);
	}

	/**
	 * Whether the other side is there: for an embodied robot, whether a pose arrived recently, or, when the pose is
	 * estimated, whether anything listens to the commands. Mirror agents are answered by {@link RosRobots}.
	 *
	 * @return whether the robot answers
	 */
	public boolean isConnected() {
		if (!embodied) return false;
		if (isEstimated()) return cmd != null && cmd.getSubscriberCount() > 0;
		return lastPose >= 0 && (System.nanoTime() - lastPose) / 1e9 < num("timeout");
	}

	/**
	 * @return the heading wander steers towards, starting from the measured one
	 */
	public double wanderHeading() {
		if (wanderHeading == null) { wanderHeading = currentHeading(); }
		return wanderHeading;
	}

	/**
	 * @param h
	 *            the new heading wander steers towards
	 */
	public void setWanderHeading(final double h) {
		wanderHeading = h;
	}

	/**
	 * @return the heading of the agent, 0 if it has none yet
	 */
	public double currentHeading() {
		return agent.getAttribute(IKeyword.HEADING) instanceof final Number n ? n.doubleValue() : 0;
	}

	// ---------------------------------------------------------------------------------------------------------------
	// The command
	// ---------------------------------------------------------------------------------------------------------------

	/**
	 * Records the command decided by an action. It is sent at the end of the step, the last decision winning.
	 *
	 * @param speed
	 *            m/s, clamped to [-max_speed, max_speed]
	 * @param turnRate
	 *            degrees/s in GAMA's convention, clamped to [-max_turn_rate, max_turn_rate]
	 */
	public synchronized void command(final double speed, final double turnRate) {
		final double maxSpeed = num("max_speed"), maxTurn = num("max_turn_rate");
		wantedSpeed = Math.max(-maxSpeed, Math.min(maxSpeed, speed));
		wantedTurn = Math.max(-maxTurn, Math.min(maxTurn, turnRate));
		commanded = true;
	}

	/**
	 * Sends a stop now, without waiting for the end of the step.
	 *
	 * @param scope
	 *            the scope, may be null from the watchdog
	 */
	public synchronized void stopNow(final IScope scope) {
		wantedSpeed = 0;
		wantedTurn = 0;
		commanded = false;
		send(scope, 0, 0);
	}

	/**
	 * Called by the watchdog thread: sends a stop if nothing was sent for longer than the watchdog delay.
	 *
	 * @param now
	 *            System.nanoTime
	 */
	synchronized void watch(final long now) {
		final double delay = settings == null ? 0 : num("watchdog");
		if (!embodied || stopped || delay <= 0 || (now - lastSent) / 1e9 < delay) return;
		send(null, 0, 0);
	}

	/**
	 * Publishes a command.
	 *
	 * @param scope
	 *            the scope, may be null
	 * @param speed
	 *            m/s
	 * @param turnRate
	 *            degrees/s, GAMA's convention
	 */
	private void send(final IScope scope, final double speed, final double turnRate) {
		if (cmd == null || cmd.isClosed()) return;
		IMap<String, Object> msg = RosGeometryOperators.rosTwist(scope, speed, turnRate);
		if (stamped) {
			final IMap<String, Object> wrapped = GamaMapFactory.create(Types.STRING, Types.NO_TYPE);
			wrapped.put("twist", msg);   // the header is filled in by the converter
			msg = wrapped;
		}
		try {
			cmd.publish(scope, msg);
		} catch (final RuntimeException e) {
			// a node closed under our feet when the experiment ends: nothing left to stop
			return;
		}
		activeSpeed = speed;
		activeTurn = turnRate;
		stopped = speed == 0 && turnRate == 0;
		lastSent = System.nanoTime();
	}

	// ---------------------------------------------------------------------------------------------------------------
	// The end of a step
	// ---------------------------------------------------------------------------------------------------------------

	/**
	 * Reads the state that came back, then sends the command of the step: decided by an action, or a stop if no
	 * action was called, just as an agent of the moving skill stays put when it does not move.
	 *
	 * @param scope
	 *            the scope of the simulation
	 */
	public synchronized void endOfStep(final IScope scope) {
		if (!embodied) return;
		readState(scope);
		if (commanded) {
			send(scope, wantedSpeed, wantedTurn);
		} else if (!stopped) { send(scope, 0, 0); }
		commanded = false;
		hintIfSilent(scope);
	}

	/**
	 * Writes the measured, or estimated, pose into the agent.
	 *
	 * @param scope
	 *            the scope
	 */
	private void readState(final IScope scope) {
		final long now = System.nanoTime();
		if (isEstimated()) {
			// integrate what the robot was doing since the last step; never longer than the watchdog lets it
			if (lastIntegration >= 0) {
				double dt = (now - lastIntegration) / 1e9;
				final double delay = num("watchdog");
				if (delay > 0) { dt = Math.min(dt, delay); }
				final double h0 = currentHeading();
				final double mid = Math.toRadians(h0 + activeTurn * dt / 2);
				final IPoint p = agent.getLocation();
				agent.setLocation(GamaPointFactory.create(p.getX() + Math.cos(mid) * activeSpeed * dt,
						p.getY() + Math.sin(mid) * activeSpeed * dt, p.getZ()));
				final double h = (h0 + activeTurn * dt) % 360;
				agent.setAttribute(IKeyword.HEADING, h < 0 ? h + 360 : h);
				agent.setAttribute(IKeyword.REAL_SPEED, Math.abs(activeSpeed));
			}
			lastIntegration = now;
			return;
		}
		final IMap<String, Object> p = pose == null ? null : pose.pollLatest();
		final IMap<String, Object> o = odom == null ? null : odom.pollLatest();
		final IMap<String, Object> source = pose != null ? p : o;
		if (source != null) {
			final IPoint position = RosGeometryOperators.rosLocation(scope, source);
			agent.setLocation(position);
			agent.setAttribute(IKeyword.HEADING, RosGeometryOperators.rosHeading(scope, source));
			lastPose = now;
			if (odom == null && previousPosition != null) {
				final double dt = (now - previousPositionAt) / 1e9;
				if (dt > 0) {
					agent.setAttribute(IKeyword.REAL_SPEED, position.euclidianDistanceTo(previousPosition) / dt);
				}
			}
			previousPosition = position;
			previousPositionAt = now;
		}
		if (o != null) { agent.setAttribute(IKeyword.REAL_SPEED, Math.abs(RosGeometryOperators.rosSpeed(scope, o))); }
	}

	/**
	 * Says once, after a few seconds of silence, what is most likely missing.
	 *
	 * @param scope
	 *            the scope
	 */
	private void hintIfSilent(final IScope scope) {
		if (hinted || isConnected() || (System.nanoTime() - boundAt) / 1e9 < num("hint_after")) return;
		hinted = true;
		final String what = isEstimated() ? "nothing subscribes to its commands on " + str("cmd_topic")
				: "no pose arrived on " + (pose != null ? str("pose_topic") : str("odom_topic"));
		final String msg = "ros_moving: " + agent.getName() + " (ros_name '" + agent.getAttribute(NAME)
				+ "', profile '" + agent.getAttribute(PROFILE) + "'): " + what + " after " + (int) num("hint_after")
				+ " s. " + str("hint");
		GAMA.reportError(scope, GamaRuntimeException.warning(msg, scope), false);
	}

	// ---------------------------------------------------------------------------------------------------------------
	// RViz
	// ---------------------------------------------------------------------------------------------------------------

	/**
	 * Builds the marker of the agent, or null if it asks for none.
	 *
	 * @param scope
	 *            the scope
	 * @return a visualization_msgs/Marker map, header left to the converter
	 */
	public IMap<String, Object> marker(final IScope scope) {
		final int type = markerType(agent.getAttribute(MARKER));
		if (type < 0) return null;
		final double[] scale = RosProfiles.triple(settings, "marker_scale");
		final IPoint p = agent.getLocation();
		final IMap<String, Object> m = GamaMapFactory.create(Types.STRING, Types.NO_TYPE);
		m.put("ns", agent.getSpecies().getName());
		m.put("id", agent.getIndex());
		m.put("type", type);
		m.put("action", 0);
		m.put("pose", RosGeometryOperators.rosPose(scope, GamaPointFactory.create(p.getX(), p.getY(), p.getZ() + scale[2] / 2),
				currentHeading()));
		m.put("scale", RosFrame.xyz(scale[0], scale[1], scale[2]));
		m.put("color", agent.getAttribute(COLOR) instanceof final IColor c ? c : DEFAULT_COLOR);
		final double life = num("marker_lifetime");
		final IMap<String, Object> lifetime = GamaMapFactory.create(Types.STRING, Types.INT);
		lifetime.put("sec", (int) Math.floor(life));
		lifetime.put("nanosec", (int) ((life - Math.floor(life)) * 1e9));
		m.put("lifetime", lifetime);
		return m;
	}

	/**
	 * @param name
	 *            the value of ros_marker
	 * @return the visualization_msgs/Marker type, -1 for none
	 */
	private static int markerType(final Object name) {
		if (name == null) return -1;
		return switch (name.toString().trim().toLowerCase()) {
			case "arrow" -> 0;
			case "cube" -> 1;
			case "sphere" -> 2;
			case "cylinder" -> 3;
			default -> -1;
		};
	}

	// ---------------------------------------------------------------------------------------------------------------
	// End of life
	// ---------------------------------------------------------------------------------------------------------------

	/**
	 * Stops the robot and closes its topics: the agent died, or the simulation ends.
	 *
	 * @param scope
	 *            the scope, may be null
	 */
	public synchronized void dispose(final IScope scope) {
		if (embodied && !stopped) { send(scope, 0, 0); }
		closeTopics();
		embodied = false;
	}

	/**
	 * Closes the topics of the robot, if any.
	 */
	private void closeTopics() {
		if (cmd != null) { cmd.close(); }
		if (pose != null) { pose.close(); }
		if (odom != null) { odom.close(); }
		cmd = null;
		pose = null;
		odom = null;
		stopped = true;
	}

}
