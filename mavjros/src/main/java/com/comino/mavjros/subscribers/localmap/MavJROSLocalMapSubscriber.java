package com.comino.mavjros.subscribers.localmap;

import org.ros.message.Time;

import com.comino.mavcom.model.DataModel;
import com.comino.mavjros.MavJROSAbstractSubscriber;
import com.comino.mavmap.map.map3D.impl.octomap.MAVOctoMap3D;


public class MavJROSLocalMapSubscriber extends MavJROSAbstractSubscriber<glmapping.local2global> {

	private final DataModel                            model;

	private Time old_tms;

	public MavJROSLocalMapSubscriber(DataModel model, MAVOctoMap3D map, String rosTopicName)  {
		super(rosTopicName, glmapping.local2global._TYPE);
		this.model        = model;

	}

	@Override
	public void callback(glmapping.local2global message) {
		System.out.println(message.getPtObsCount());

	}

}
