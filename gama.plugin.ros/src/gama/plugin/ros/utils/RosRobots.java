/*******************************************************************************************************
 *
 * RosRobots.java, in gama.plugin.ros, is part of the source code of the GAMA modeling and simulation platform.
 *
 * (c) 2007-2026 UMI 209 UMMISCO IRD/SU & Partners (IRIT, MIAT, ESPACE-DEV, CTU)
 *
 * Visit https://github.com/TANGO-projet/gama.plugin.ros for license information and contacts.
 *
 ********************************************************************************************************/
package gama.plugin.ros.utils;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import gama.api.exceptions.GamaRuntimeException;
import gama.api.gaml.types.Types;
import gama.api.kernel.agent.IAgent;
import gama.api.kernel.simulation.ISimulationAgent;
import gama.api.runtime.scope.IScope;
import gama.api.types.list.GamaListFactory;
import gama.api.types.list.IList;
import gama.api.types.map.GamaMapFactory;
import gama.api.types.map.IMap;
import gama.plugin.ros.RosGeometryOperators;
import gama.plugin.ros.RosOperators;
import gama.plugin.ros.types.GamaROSNode;
import gama.plugin.ros.types.GamaROSPublisher;
import gama.plugin.ros.types.GamaROSTopic;

/**
 * The robots of one simulation, and the work done for all of them at once.
 *
 * <ul>
 * <li>At the <b>end of every step</b>: each embodied robot reads the pose that came back and sends the command its
 * actions decided; then every agent that wants a marker is put in one MarkerArray per marker topic, however many
 * agents there are.</li>
 * <li>A <b>watchdog</b> thread stops any robot that has not been sent a command for longer than its {@code watchdog}
 * setting. cmd_vel is a setpoint that a base keeps applying, so without it a paused, crashed or closed GAMA leaves the
 * robot driving.</li>
 * <li>When the simulation is <b>disposed of</b>, every robot is stopped.</li>
 * </ul>
 */
public final class RosRobots {

	/** The key under which the instance is stored on the simulation. */
	private static final String KEY = "$ros2_robots";

	/** The robots, iterated by the simulation thread and by the watchdog. */
	private final List<RosRobot> robots = new CopyOnWriteArrayList<>();

	/** One MarkerArray publisher per topic, and whether its first message has cleared what a previous run left. */
	private final Map<String, GamaROSPublisher> markerChannels = new LinkedHashMap<>();

	/** The marker topics whose first message has been sent. */
	private final List<String> cleared = new ArrayList<>();

	/** The markers of agents that died, deleted with the next batch of their topic. */
	private final Map<String, List<IMap<String, Object>>> pendingDeletes = new LinkedHashMap<>();

	/** The static transform publisher, kept so that the latched transform keeps being served. */
	private GamaROSPublisher tfStatic;

	/** The watchdog. */
	private final ScheduledExecutorService watchdog = Executors.newSingleThreadScheduledExecutor(r -> {
		final Thread t = new Thread(r, "ros_moving watchdog");
		t.setDaemon(true);
		return t;
	});

	/**
	 * @param sim
	 *            the simulation
	 */
	private RosRobots(final ISimulationAgent sim) {
		sim.postEndAction(scope -> {
			step(scope);
			return null;
		});
		sim.postDisposeAction(scope -> {
			dispose(scope);
			return null;
		});
		watchdog.scheduleAtFixedRate(() -> {
			final long now = System.nanoTime();
			for (final RosRobot r : robots) {
				try {
					r.watch(now);
				} catch (final RuntimeException e) {
					// the watchdog must survive anything: it is what stops the robots
				}
			}
		}, 100, 100, TimeUnit.MILLISECONDS);
	}

	/**
	 * Returns the instance of the simulation of the scope, created on first use.
	 *
	 * @param scope
	 *            the scope
	 * @return the instance
	 */
	public static RosRobots of(final IScope scope) {
		final ISimulationAgent sim = scope == null ? null : scope.getSimulation();
		if (sim == null)
			throw GamaRuntimeException.error("The ros_moving skill can only be used by agents of a simulation", scope);
		synchronized (sim) {
			if (sim.getAttribute(KEY) instanceof final RosRobots r) return r;
			final RosRobots r = new RosRobots(sim);
			sim.setAttribute(KEY, r);
			return r;
		}
	}

	/**
	 * @param robot
	 *            a new robot
	 */
	void add(final RosRobot robot) {
		robots.add(robot);
	}

	/**
	 * Tells whether anything listens to the markers of a robot: what ros_connected means for a mirror agent.
	 *
	 * @param robot
	 *            the robot
	 * @return whether its marker topic has subscribers
	 */
	public boolean markersListened(final RosRobot robot) {
		final GamaROSPublisher p = markerChannels.get(robot.str("marker_topic"));
		return p != null && !p.isClosed() && p.getSubscriberCount() > 0;
	}

	// ---------------------------------------------------------------------------------------------------------------

	/**
	 * The end of a step of the simulation.
	 *
	 * @param scope
	 *            the scope of the simulation
	 */
	private void step(final IScope scope) {
		final Map<String, IList<IMap<String, Object>>> batches = new LinkedHashMap<>();
		GamaROSNode node = null;
		String tfFrame = null;
		for (final RosRobot r : robots) {
			final IAgent agent = r.agent();
			if (agent.dead()) {
				if (r.settings() != null) { queueDelete(r); }
				r.dispose(scope);
				robots.remove(r);
				continue;
			}
			r.ensureReady(scope);
			r.endOfStep(scope);
			final IMap<String, Object> marker = r.marker(scope);
			if (marker != null) {
				batches.computeIfAbsent(r.str("marker_topic"), k -> GamaListFactory.create(Types.MAP)).add(marker);
				if (node == null) {
					node = r.node();
					tfFrame = r.str("tf_frame");
				}
			}
		}
		pendingDeletes.forEach((topic, deletes) -> batches.computeIfAbsent(topic,
				k -> GamaListFactory.create(Types.MAP)).addAll(0, deletes));
		pendingDeletes.clear();
		if (batches.isEmpty() || node == null) return;
		if (tfStatic == null && tfFrame != null && !tfFrame.isEmpty()) { publishWorldFrame(scope, node, tfFrame); }
		for (final Map.Entry<String, IList<IMap<String, Object>>> e : batches.entrySet()) {
			publishMarkers(scope, node, e.getKey(), e.getValue());
		}
	}

	/**
	 * Sends a batch of markers, preceded on the first message of the topic by a DELETEALL that clears whatever a
	 * previous run left behind with ids this one will not reuse.
	 *
	 * @param scope
	 *            the scope
	 * @param node
	 *            the node to publish from
	 * @param topic
	 *            the marker topic
	 * @param markers
	 *            the markers
	 */
	private void publishMarkers(final IScope scope, final GamaROSNode node, final String topic,
			final IList<IMap<String, Object>> markers) {
		final GamaROSPublisher pub = markerChannels.computeIfAbsent(topic, t -> new GamaROSPublisher(scope, node,
				new GamaROSTopic(scope, t, "visualization_msgs/msg/MarkerArray", RosOperators.qosNamed(scope, "default"))));
		if (!cleared.contains(topic)) {
			final IMap<String, Object> clear = GamaMapFactory.create(Types.STRING, Types.NO_TYPE);
			clear.put("action", 3);
			markers.add(0, clear);
			cleared.add(topic);
		}
		final IMap<String, Object> msg = GamaMapFactory.create(Types.STRING, Types.NO_TYPE);
		msg.put("markers", markers);
		pub.publish(scope, msg);
	}

	/**
	 * Makes the frame of the world exist in TF, once and for good, as model 05 does by hand.
	 *
	 * @param scope
	 *            the scope
	 * @param node
	 *            the node
	 * @param child
	 *            the frame placed at the centre of the GAMA world
	 */
	private void publishWorldFrame(final IScope scope, final GamaROSNode node, final String child) {
		tfStatic = new GamaROSPublisher(scope, node,
				new GamaROSTopic(scope, "/tf_static", "tf2_msgs/msg/TFMessage", RosOperators.qosNamed(scope, "latched")));
		tfStatic.publish(scope, RosGeometryOperators.rosWorldTf(scope, child));
	}

	/**
	 * @param r
	 *            a robot whose agent died
	 */
	private void queueDelete(final RosRobot r) {
		final IMap<String, Object> delete = GamaMapFactory.create(Types.STRING, Types.NO_TYPE);
		delete.put("ns", r.agent().getSpecies().getName());
		delete.put("id", r.agent().getIndex());
		delete.put("action", 2);
		pendingDeletes.computeIfAbsent(r.str("marker_topic"), k -> new ArrayList<>()).add(delete);
	}

	/**
	 * Stops every robot and closes what this instance opened.
	 *
	 * @param scope
	 *            the scope
	 */
	private void dispose(final IScope scope) {
		watchdog.shutdownNow();
		for (final RosRobot r : robots) { r.dispose(scope); }
		robots.clear();
		markerChannels.values().forEach(GamaROSPublisher::close);
		markerChannels.clear();
		if (tfStatic != null) { tfStatic.close(); }
	}

}
