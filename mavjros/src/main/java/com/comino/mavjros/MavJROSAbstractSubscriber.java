package com.comino.mavjros;

public abstract class MavJROSAbstractSubscriber<T> {
	
	final String topicName;
	final String type;
	
	public MavJROSAbstractSubscriber(String topicName, String type) {
		this.topicName = topicName;
		this.type      = type;
	}

	public String getTopicName() {
		return topicName;
	}
	
	public String getType() {
		return type;
	}
	
	public abstract void callback(T o);
	
	@SuppressWarnings("unchecked")
	protected void internal_callback(Object o) {
		callback((T)o);
	}
	

}
