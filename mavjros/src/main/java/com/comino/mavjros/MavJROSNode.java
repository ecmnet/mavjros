package com.comino.mavjros;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.StringUtils;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Subscriber;

import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Status;
import com.google.common.base.Preconditions;

import rosgraph_msgs.Clock;

public class MavJROSNode {
	
	private static final String ROS_MASTER_URI = "ROS_MASTER_URI";
    private static final String ROS_IP         = "ROS_IP";
    private static final String NODE_NAME      = "/mavjros/";
    private static final String CLOCK_TOPIC    = "/clock";
	
	private static MavJROSNode instance;
	
	private NodeMainExecutor nodeMainExecutor;
	private NodeConfiguration nodeConfiguration;
	private MavJROSSubscriberNode subscriberNode;
	
	private final DataModel model;
	
	private static long     start_unix_time = 0;
	private static Time     start_ros_time;
	
	
	public static MavJROSNode getInstance(DataModel model) {
		if(instance == null) {
			instance = new MavJROSNode(model);
		}
		return instance;
	}
	
	public MavJROSNode(DataModel model) {
		 this.subscriberNode = new MavJROSSubscriberNode();
		 this.model          = model;
	}
	
	
	public void connect() throws URISyntaxException {
		
		final String rosMasterUriEnv = System.getenv(ROS_MASTER_URI);
		final String rosHostIp = System.getenv(ROS_IP);
		Preconditions.checkState(StringUtils.isNotBlank(rosMasterUriEnv), ROS_MASTER_URI + " environment variable needs to be set.");
		Preconditions.checkState(StringUtils.isNotBlank(rosHostIp), ROS_IP + " environment variable needs to be set.");
		
		final URI rosMasterUri = new URI(rosMasterUriEnv);
        nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
        
        nodeConfiguration = NodeConfiguration.newPublic(rosHostIp);
        nodeConfiguration.setNodeName(NODE_NAME);
        nodeConfiguration.setMasterUri(rosMasterUri);
        
        nodeMainExecutor.execute(subscriberNode, nodeConfiguration);
        
        this.addSubscriber(new MavJROSClockSubscriber());
        
        model.sys.setSensor(Status.MSP_ROS_AVAILABILITY, true);
        
        
	}
	
	public void addSubscriber(MavJROSAbstractSubscriber<?> s) {
		  this.subscriberNode.addSubscriber(s);
	}
	

	public void shutdown() {
		if(nodeMainExecutor!=null)
			nodeMainExecutor.shutdown();
	}
	
	
	private class MavJROSSubscriberNode extends AbstractNodeMain {
		
		private List<MavJROSAbstractSubscriber<?>> subscribers = new ArrayList<MavJROSAbstractSubscriber<?>>();
		
		
		public void addSubscriber(MavJROSAbstractSubscriber<?> s) {
			subscribers.add(s);
		}
	
		@Override
		public final void onStart(final ConnectedNode connectedNode) {
			subscribers.forEach(s -> {
				Subscriber<?> subscriber = connectedNode.newSubscriber(s.getTopicName(),s.getType());
				subscriber.addMessageListener((o) -> {
					s.internal_callback(o);
				});
			});
		}

		@Override
		public GraphName getDefaultNodeName() {
			return nodeConfiguration.getNodeName();
		}
		
	}
	
	private class MavJROSClockSubscriber extends MavJROSAbstractSubscriber<rosgraph_msgs.Clock> {

		public MavJROSClockSubscriber() {
			super("/clock", rosgraph_msgs.Clock._TYPE);
		}

		@Override
		public void callback(Clock message) {
			if(start_unix_time==0) {
				start_unix_time = System.currentTimeMillis();
				start_ros_time = message.getClock();
			}
		}		
	}
	
	public static long getAsUnixTime(Time time) {
		return start_unix_time + time.subtract(start_ros_time).totalNsecs()/ 1_000_000L;
	}
}
