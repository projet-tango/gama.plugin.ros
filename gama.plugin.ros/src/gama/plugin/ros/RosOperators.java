/*******************************************************************************************************
 *
 * RosOperators.java, in gama.plugin.ros, is part of the source code of the GAMA modeling and simulation platform.
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
import gama.api.exceptions.GamaRuntimeException;
import gama.api.runtime.scope.IScope;
import gama.api.types.list.IList;
import gama.api.types.map.IMap;
import gama.plugin.ros.types.GamaROSNode;
import gama.plugin.ros.types.GamaROSPublisher;
import gama.plugin.ros.types.GamaROSSubscription;
import gama.plugin.ros.types.GamaROSTopic;
import gama.plugin.ros.utils.RosNodes;
import us.ihmc.jros2.ROS2QoSProfile;

/**
 * The GAML operators of the ROS2 plugin: joining a domain, describing a topic, and moving messages in and out of the
 * simulation.
 *
 * <h3>The shape of a model</h3>
 *
 * <pre>
 * global {
 *     ros_node        bridge  &lt;- ros_node("gama");
 *     ros_topic       cmd_vel &lt;- ros_topic("/cmd_vel", "geometry_msgs/msg/Twist");
 *     ros_publisher   command &lt;- ros_publisher(bridge, cmd_vel);
 *     ros_subscription odometry &lt;- ros_subscription(bridge, ros_topic("/odom", "nav_msgs/msg/Odometry"));
 *
 *     reflex drive {
 *         bool sent &lt;- ros_publish(command, ["linear"::["x"::1.0], "angular"::["z"::0.5]]);
 *         map&lt;string, unknown&gt; state &lt;- ros_read_latest(odometry);
 *         if state != nil { write state["pose"]; }
 *     }
 * }
 * </pre>
 *
 * <h3>Messages are maps</h3>
 *
 * <p>
 * A ROS message is read as a map whose keys are the field names of the type, nested messages being nested maps and
 * sequences being lists. The same shape is used to publish, and a field left out keeps its ROS default, so a
 * {@code Twist} that only sets a forward speed is written {@code ["linear"::["x"::1.0]]}. A key that is not a field of
 * the type raises an error listing the fields that do exist, which {@code ros_fields} also gives.
 * </p>
 *
 * <h3>Operators, not statements</h3>
 *
 * <p>
 * Everything here is an operator, so the ones that act on the network are used in an assignment rather than with
 * {@code do}: {@code bool sent &lt;- ros_publish(...)}, {@code int lost &lt;- ros_clear(...)}. What they return is
 * documented on each of them.
 * </p>
 */
public class RosOperators {

	private RosOperators() {}

	// ---------------------------------------------------------------------------------------------------------------
	// Nodes
	// ---------------------------------------------------------------------------------------------------------------

	/**
	 * Returns the node of the given name on domain 0.
	 *
	 * @param scope
	 *            the scope
	 * @param name
	 *            the name of the node
	 * @return the node
	 */
	@operator (
			value = "ros_node",
			type = RosConstants.NODE_ID,
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Returns the ROS2 node of the given name on domain 0, joining the network on first use. The node belongs to the current experiment: naming it again gives back the same node, and it is closed when the experiment is disposed.",
			examples = { @example (
					value = "ros_node bridge <- ros_node(\"gama\");",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static GamaROSNode rosNode(final IScope scope, final String name) {
		return RosNodes.get(scope, name, 0);
	}

	/**
	 * Returns the node of the given name on the given domain.
	 *
	 * @param scope
	 *            the scope
	 * @param name
	 *            the name of the node
	 * @param domainId
	 *            the ROS domain id, between 0 and 232
	 * @return the node
	 */
	@operator (
			value = "ros_node",
			type = RosConstants.NODE_ID,
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Returns the ROS2 node of the given name on the given domain, joining the network on first use. Two nodes with the same name on different domains are two different nodes, and only participants of the same domain see each other.",
			examples = { @example (
					value = "ros_node bridge <- ros_node(\"gama\", 42);",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static GamaROSNode rosNode(final IScope scope, final String name, final Integer domainId) {
		return RosNodes.get(scope, name, domainId == null ? 0 : domainId);
	}

	// ---------------------------------------------------------------------------------------------------------------
	// Topics
	// ---------------------------------------------------------------------------------------------------------------

	/**
	 * Describes a topic with the default quality of service.
	 *
	 * @param scope
	 *            the scope
	 * @param topicName
	 *            the name of the topic
	 * @param messageType
	 *            the message type
	 * @return the topic
	 */
	@operator (
			value = "ros_topic",
			type = RosConstants.TOPIC_ID,
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Describes a topic by its name and the message type carried on it. The type is written either as in ROS ('geometry_msgs/msg/Twist') or as a Java class name ('geometry_msgs.Twist'), and is checked immediately: an unknown type raises an error here rather than at the first publication.",
			examples = { @example (
					value = "ros_topic cmd_vel <- ros_topic(\"/cmd_vel\", \"geometry_msgs/msg/Twist\");",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static GamaROSTopic rosTopic(final IScope scope, final String topicName, final String messageType) {
		return new GamaROSTopic(scope, topicName, messageType);
	}

	/**
	 * Describes a topic with an explicit quality of service.
	 *
	 * @param scope
	 *            the scope
	 * @param topicName
	 *            the name of the topic
	 * @param messageType
	 *            the message type
	 * @param qos
	 *            "default", "reliable", "best_effort" or "latched"
	 * @return the topic
	 */
	@operator (
			value = "ros_topic",
			type = RosConstants.TOPIC_ID,
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Describes a topic with an explicit quality of service, among 'default', 'reliable', 'best_effort' and 'latched'. Reliable retransmits until the message is acknowledged, which is what commands need; best effort does not, which is what a high-rate sensor needs, and is what most ROS drivers publish scans and images with. A subscription only receives from publishers whose QoS is compatible with its own, so a topic read as reliable from a best-effort driver stays silent. Latched is reliable and transient-local, keeping the last message for subscribers that join later: it is what /tf_static and maps are published with, so something sent once in init still reaches an RViz started afterwards.",
			examples = { @example (
					value = "ros_topic scan <- ros_topic(\"/scan\", \"sensor_msgs/msg/LaserScan\", \"best_effort\");",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static GamaROSTopic rosTopic(final IScope scope, final String topicName, final String messageType,
			final String qos) {
		return new GamaROSTopic(scope, topicName, messageType, qosNamed(scope, qos));
	}

	/**
	 * Builds the latched profile, a fresh one each time since jros2 profiles are mutable.
	 *
	 * @return the profile
	 */
	private static ROS2QoSProfile latched() {
		// what tf2's StaticTransformBroadcaster uses: reliable, transient local, the last message only
		final ROS2QoSProfile qos = new ROS2QoSProfile();
		qos.reliability(ROS2QoSProfile.Reliability.RELIABLE);
		qos.durability(ROS2QoSProfile.Durability.TRANSIENT_LOCAL);
		qos.history(ROS2QoSProfile.History.KEEP_LAST);
		qos.depth(1);
		return qos;
	}

	/**
	 * Translates a GAML quality of service name to its jros2 profile.
	 *
	 * @param scope
	 *            the scope
	 * @param name
	 *            the name written in the model
	 * @return the profile
	 */
	public static ROS2QoSProfile qosNamed(final IScope scope, final String name) {
		if (name == null) return ROS2QoSProfile.DEFAULT;
		return switch (name.toLowerCase().trim()) {
			case "default" -> ROS2QoSProfile.DEFAULT;
			case "reliable" -> ROS2QoSProfile.RELIABLE;
			case "best_effort" -> ROS2QoSProfile.BEST_EFFORT;
			case "latched", "transient_local" -> latched();
			default -> throw GamaRuntimeException.error("'" + name
					+ "' is not a known quality of service. Use 'default', 'reliable', 'best_effort' or 'latched'.", scope);
		};
	}

	/**
	 * Lists the fields of the message type of a topic.
	 *
	 * @param scope
	 *            the scope
	 * @param topic
	 *            the topic
	 * @return the field names, sorted
	 */
	@operator (
			value = "ros_fields",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Returns the names of the fields of the message type of a topic, which are the keys of the maps read from it and published on it. Only the first level is listed: a nested message appears as one key, and its own fields are given by the map that reading the topic returns.",
			examples = { @example (
					value = "write ros_fields(ros_topic(\"/cmd_vel\", \"geometry_msgs/msg/Twist\")); // ['angular','linear']",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static IList<String> rosFields(final IScope scope, final GamaROSTopic topic) {
		if (topic == null) throw GamaRuntimeException.error("ros_fields expects a topic, not nil", scope);
		return topic.getFields();
	}

	// ---------------------------------------------------------------------------------------------------------------
	// Publishing
	// ---------------------------------------------------------------------------------------------------------------

	/**
	 * Creates a publisher.
	 *
	 * @param scope
	 *            the scope
	 * @param node
	 *            the node
	 * @param topic
	 *            the topic
	 * @return the publisher
	 */
	@operator (
			value = "ros_publisher",
			type = RosConstants.PUBLISHER_ID,
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Creates a publisher on the given node and topic, using the quality of service of the topic. Create it once, in the init of the model, and keep it: a publisher is a network resource, and creating one per publication would drop most of the messages, since a freshly created publisher has not discovered its subscribers yet.",
			examples = { @example (
					value = "ros_publisher command <- ros_publisher(bridge, cmd_vel);",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static GamaROSPublisher rosPublisher(final IScope scope, final GamaROSNode node, final GamaROSTopic topic) {
		return new GamaROSPublisher(scope, node, topic);
	}

	/**
	 * Publishes a message.
	 *
	 * @param scope
	 *            the scope
	 * @param publisher
	 *            the publisher
	 * @param message
	 *            the fields of the message, keyed by field name
	 * @return true, so that the operator can be used in an assignment
	 */
	@operator (
			value = "ros_publish",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Publishes a message on a publisher and returns true. The message is a map whose keys are the fields of the message type, a nested message being a nested map and a sequence a list; a field left out of the map keeps its ROS default. A key that is not a field of the type, or a value that does not fit it, raises an error naming the field.",
			examples = { @example (
					value = "bool sent <- ros_publish(command, [\"linear\"::[\"x\"::1.0], \"angular\"::[\"z\"::0.5]]);",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static Boolean rosPublish(final IScope scope, final GamaROSPublisher publisher,
			final IMap<String, Object> message) {
		if (publisher == null) throw GamaRuntimeException.error("ros_publish expects a publisher, not nil", scope);
		publisher.publish(scope, message);
		return Boolean.TRUE;
	}

	/**
	 * Waits for a subscriber on the topic of a publisher.
	 *
	 * @param scope
	 *            the scope
	 * @param publisher
	 *            the publisher
	 * @param milliseconds
	 *            how long to wait at most
	 * @return true if a subscriber was matched within the delay
	 */
	@operator (
			value = "ros_wait_for_subscriber",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Blocks until a subscriber is discovered on the topic of the publisher, or the delay in milliseconds runs out, and returns whether one was found. ROS2 discovery takes a moment, and messages published before a subscriber is matched are dropped, so a model that only publishes a few messages should wait here first. A model publishing continuously does not need to.",
			examples = { @example (
					value = "if !ros_wait_for_subscriber(command, 2000) { write \"nobody is listening to /cmd_vel\"; }",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static Boolean rosWaitForSubscriber(final IScope scope, final GamaROSPublisher publisher,
			final Integer milliseconds) {
		if (publisher == null)
			throw GamaRuntimeException.error("ros_wait_for_subscriber expects a publisher, not nil", scope);
		publisher.checkOpen(scope);
		return publisher.waitForSubscriber(milliseconds == null ? 0 : milliseconds);
	}

	/**
	 * Counts the subscribers matched on the topic of a publisher.
	 *
	 * @param scope
	 *            the scope
	 * @param publisher
	 *            the publisher
	 * @return the number of subscribers
	 */
	@operator (
			value = "ros_subscriber_count",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Returns how many subscribers are currently matched on the topic of the publisher. Zero means that what is published goes nowhere, which is the usual explanation for a robot that does not move.",
			examples = { @example (
					value = "write \"listeners: \" + ros_subscriber_count(command);",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static Integer rosSubscriberCount(final IScope scope, final GamaROSPublisher publisher) {
		return publisher == null || publisher.isClosed() ? 0 : publisher.getSubscriberCount();
	}

	// ---------------------------------------------------------------------------------------------------------------
	// Subscribing
	// ---------------------------------------------------------------------------------------------------------------

	/**
	 * Creates a subscription with the default queue size.
	 *
	 * @param scope
	 *            the scope
	 * @param node
	 *            the node
	 * @param topic
	 *            the topic
	 * @return the subscription
	 */
	@operator (
			value = "ros_subscription",
			type = RosConstants.SUBSCRIPTION_ID,
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Creates a subscription on the given node and topic, using the quality of service of the topic. Messages start arriving immediately, on a thread of their own, and wait in a queue of "
					+ RosConstants.DEFAULT_QUEUE_SIZE
					+ " messages until the model reads them with ros_read, ros_read_latest or ros_read_all. Create it once, in the init of the model: a subscription is a network resource.",
			examples = { @example (
					value = "ros_subscription odometry <- ros_subscription(bridge, ros_topic(\"/odom\", \"nav_msgs/msg/Odometry\"));",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static GamaROSSubscription rosSubscription(final IScope scope, final GamaROSNode node,
			final GamaROSTopic topic) {
		return new GamaROSSubscription(scope, node, topic, RosConstants.DEFAULT_QUEUE_SIZE);
	}

	/**
	 * Creates a subscription with an explicit queue size.
	 *
	 * @param scope
	 *            the scope
	 * @param node
	 *            the node
	 * @param topic
	 *            the topic
	 * @param queueSize
	 *            how many messages to keep before dropping the oldest
	 * @return the subscription
	 */
	@operator (
			value = "ros_subscription",
			type = RosConstants.SUBSCRIPTION_ID,
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Creates a subscription whose queue holds the given number of messages. When the queue is full the oldest message is dropped, as a ROS subscription of the same depth would do, and ros_dropped reports how many were lost. A model that only wants the current state of a sensor can use a queue of 1 with ros_read_latest; a model that must not miss an event needs a queue larger than the number of messages that can arrive between two cycles.",
			examples = { @example (
					value = "ros_subscription scan <- ros_subscription(bridge, scan_topic, 1);",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static GamaROSSubscription rosSubscription(final IScope scope, final GamaROSNode node,
			final GamaROSTopic topic, final Integer queueSize) {
		return new GamaROSSubscription(scope, node, topic, queueSize == null ? RosConstants.DEFAULT_QUEUE_SIZE
				: queueSize);
	}

	/**
	 * Tells whether a message is waiting.
	 *
	 * @param scope
	 *            the scope
	 * @param sub
	 *            the subscription
	 * @return true if at least one message is waiting
	 */
	@operator (
			value = "ros_has_data",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Returns whether at least one message is waiting in the queue of the subscription. A closed or nil subscription simply has no data, so this operator never fails and can guard a read.",
			examples = { @example (
					value = "if ros_has_data(odometry) { write ros_read(odometry); }",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static Boolean rosHasData(final IScope scope, final GamaROSSubscription sub) {
		return sub != null && !sub.isClosed() && sub.hasData();
	}

	/**
	 * Reads the oldest waiting message.
	 *
	 * @param scope
	 *            the scope
	 * @param sub
	 *            the subscription
	 * @return the message, or nil
	 */
	@operator (
			value = "ros_read",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Takes the oldest message out of the queue of the subscription and returns it as a map, or nil if nothing is waiting. Reading consumes the message: reading twice returns two different ones.",
			examples = { @example (
					value = "map<string, unknown> msg <- ros_read(odometry);",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static IMap<String, Object> rosRead(final IScope scope, final GamaROSSubscription sub) {
		if (sub == null) return null;
		sub.checkOpen(scope);
		return sub.poll();
	}

	/**
	 * Reads the newest waiting message and drops the others.
	 *
	 * @param scope
	 *            the scope
	 * @param sub
	 *            the subscription
	 * @return the message, or nil
	 */
	@operator (
			value = "ros_read_latest",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Empties the queue of the subscription and returns its most recent message, or nil if nothing is waiting. This is what a model wants of a sensor topic, where only the current state matters and the messages that piled up during the cycle are already out of date.",
			examples = { @example (
					value = "map<string, unknown> state <- ros_read_latest(odometry);",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static IMap<String, Object> rosReadLatest(final IScope scope, final GamaROSSubscription sub) {
		if (sub == null) return null;
		sub.checkOpen(scope);
		return sub.pollLatest();
	}

	/**
	 * Reads every waiting message.
	 *
	 * @param scope
	 *            the scope
	 * @param sub
	 *            the subscription
	 * @return the messages, oldest first
	 */
	@operator (
			value = "ros_read_all",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Empties the queue of the subscription and returns every message it held, oldest first, or an empty list if nothing was waiting. This is what a model wants of an event topic, where each message counts.",
			examples = { @example (
					value = "loop msg over: ros_read_all(events) { write msg; }",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static IList<IMap<String, Object>> rosReadAll(final IScope scope, final GamaROSSubscription sub) {
		if (sub == null) return null;
		sub.checkOpen(scope);
		return sub.pollAll();
	}

	/**
	 * Counts the waiting messages.
	 *
	 * @param scope
	 *            the scope
	 * @param sub
	 *            the subscription
	 * @return how many messages are waiting
	 */
	@operator (
			value = "ros_queue_size",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Returns how many messages are waiting in the queue of the subscription.",
			examples = { @example (
					value = "write \"backlog: \" + ros_queue_size(scan);",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static Integer rosQueueSize(final IScope scope, final GamaROSSubscription sub) {
		return sub == null ? 0 : sub.queueSize();
	}

	/**
	 * Counts the messages lost to a full queue.
	 *
	 * @param scope
	 *            the scope
	 * @param sub
	 *            the subscription
	 * @return how many messages were dropped
	 */
	@operator (
			value = "ros_dropped",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Returns how many messages the subscription had to drop since it was created because its queue was full when they arrived. A number that keeps growing means the model reads more slowly than the topic publishes, which is fine with ros_read_latest and a loss of information with ros_read.",
			examples = { @example (
					value = "if ros_dropped(scan) > 0 { write \"the model cannot keep up with /scan\"; }",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static Integer rosDropped(final IScope scope, final GamaROSSubscription sub) {
		return sub == null ? 0 : (int) Math.min(Integer.MAX_VALUE, sub.getDroppedCount());
	}

	/**
	 * Counts the messages received.
	 *
	 * @param scope
	 *            the scope
	 * @param sub
	 *            the subscription
	 * @return how many messages have arrived
	 */
	@operator (
			value = "ros_received",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Returns how many messages have arrived on the subscription since it was created, dropped ones included. Zero after a few cycles means that no publisher was matched, most often because of a topic name typed differently or a quality of service that does not match.",
			examples = { @example (
					value = "write \"received so far: \" + ros_received(odometry);",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static Integer rosReceived(final IScope scope, final GamaROSSubscription sub) {
		return sub == null ? 0 : (int) Math.min(Integer.MAX_VALUE, sub.getReceivedCount());
	}

	/**
	 * Empties the queue of a subscription.
	 *
	 * @param scope
	 *            the scope
	 * @param sub
	 *            the subscription
	 * @return how many messages were discarded
	 */
	@operator (
			value = "ros_clear",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Discards every message waiting in the queue of the subscription and returns how many were discarded. Useful after a pause, when the backlog describes a world that no longer exists.",
			examples = { @example (
					value = "int stale <- ros_clear(odometry);",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static Integer rosClear(final IScope scope, final GamaROSSubscription sub) {
		return sub == null ? 0 : sub.clearQueue();
	}

	/**
	 * Waits for a publisher on the topic of a subscription.
	 *
	 * @param scope
	 *            the scope
	 * @param sub
	 *            the subscription
	 * @param milliseconds
	 *            how long to wait at most
	 * @return true if a publisher was matched within the delay
	 */
	@operator (
			value = "ros_wait_for_publisher",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Blocks until a publisher is discovered on the topic of the subscription, or the delay in milliseconds runs out, and returns whether one was found. Waiting in the init of a model turns a silent run into an explicit failure.",
			examples = { @example (
					value = "if !ros_wait_for_publisher(odometry, 5000) { write \"nothing publishes /odom\"; }",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static Boolean rosWaitForPublisher(final IScope scope, final GamaROSSubscription sub,
			final Integer milliseconds) {
		if (sub == null)
			throw GamaRuntimeException.error("ros_wait_for_publisher expects a subscription, not nil", scope);
		sub.checkOpen(scope);
		return sub.waitForPublisher(milliseconds == null ? 0 : milliseconds);
	}

	// ---------------------------------------------------------------------------------------------------------------
	// Closing
	// ---------------------------------------------------------------------------------------------------------------

	/**
	 * Closes a node and everything created from it.
	 *
	 * @param scope
	 *            the scope
	 * @param node
	 *            the node
	 * @return true if the node was open
	 */
	@operator (
			value = "ros_close",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Closes a node, with every publisher and subscription created from it, and returns whether it was still open. Nodes are closed automatically when the experiment is disposed, so this is only needed to leave the domain early, or to give a node up so that a later ros_node with the same name creates a fresh one.",
			examples = { @example (
					value = "bool was_open <- ros_close(bridge);",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static Boolean rosClose(final IScope scope, final GamaROSNode node) {
		return RosNodes.close(scope, node);
	}

	/**
	 * Destroys a publisher.
	 *
	 * @param scope
	 *            the scope
	 * @param publisher
	 *            the publisher
	 * @return true if the publisher was open
	 */
	@operator (
			value = "ros_close",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Destroys a publisher on its node and returns whether it was still open. The node stays open.",
			examples = { @example (
					value = "bool was_open <- ros_close(command);",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static Boolean rosClose(final IScope scope, final GamaROSPublisher publisher) {
		if (publisher == null || publisher.isClosed()) return Boolean.FALSE;
		publisher.close();
		return Boolean.TRUE;
	}

	/**
	 * Destroys a subscription.
	 *
	 * @param scope
	 *            the scope
	 * @param sub
	 *            the subscription
	 * @return true if the subscription was open
	 */
	@operator (
			value = "ros_close",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Destroys a subscription on its node, dropping what it had buffered, and returns whether it was still open. The node stays open.",
			examples = { @example (
					value = "bool was_open <- ros_close(odometry);",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static Boolean rosClose(final IScope scope, final GamaROSSubscription sub) {
		if (sub == null || sub.isClosed()) return Boolean.FALSE;
		sub.close();
		return Boolean.TRUE;
	}

	/**
	 * Closes every node of the experiment.
	 *
	 * @param scope
	 *            the scope
	 * @param unused
	 *            ignored, present so that the operator can be written with an argument
	 * @return how many nodes were closed
	 */
	@operator (
			value = "ros_close_all",
			category = { RosConstants.CATEGORY },
			concept = { RosConstants.CONCEPT })
	@doc (
			value = "Closes every ROS2 node of the current experiment and returns how many were closed. The experiment does this on its own when it is disposed; calling it only makes the departure from the domain happen earlier. The argument is ignored and is only there because GAML operators take at least one.",
			examples = { @example (
					value = "int closed <- ros_close_all(\"\");",
					isExecutable = false) })
	@no_test (Reason.IMPOSSIBLE_TO_TEST)
	public static Integer rosCloseAll(final IScope scope, final String unused) {
		return RosNodes.closeAll(scope);
	}

}
