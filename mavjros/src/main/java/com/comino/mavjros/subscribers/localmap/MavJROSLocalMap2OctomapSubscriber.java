package com.comino.mavjros.subscribers.localmap;

import com.comino.mavcom.model.DataModel;
import com.comino.mavjros.MavJROSAbstractSubscriber;
import com.comino.mavjros.utils.MavJROSUtils;
import com.comino.mavmap.map.map3D.impl.octomap.MAVOctoMap3D;

import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.so.Quaternion_F64;


public class MavJROSLocalMap2OctomapSubscriber extends MavJROSAbstractSubscriber<glmapping.local2global> {

	private final DataModel                            model;
	private final Vector3D_F64                         point;
	private final Vector3D_F64                         point_t;
	private final Quaternion_F64                       rotation;
	private final Se3_F64                              transform;
	
	private long tms;

	private MAVOctoMap3D map;

	public MavJROSLocalMap2OctomapSubscriber(DataModel model, MAVOctoMap3D map, String rosTopicName)  {
		super(rosTopicName, glmapping.local2global._TYPE);
		this.model        = model;
		this.map          = map;

		this.point        = new Vector3D_F64();
		this.point_t      = new Vector3D_F64();
		this.rotation     = new Quaternion_F64();
		this.transform    = new Se3_F64();

		map.enableRemoveOutdated(true);
		
	}

	@Override
	public void callback(glmapping.local2global message) {
		
		if((System.currentTimeMillis()-tms)<100)
			return;

		MavJROSUtils.convert(message.getTWL().getTranslation(), transform.T);
		MavJROSUtils.convert(message.getTWL().getRotation(),rotation);
		ConvertRotation3D_F64.quaternionToMatrix(rotation,transform.R);

		message.getPtsMissL().forEach(missed -> {
			MavJROSUtils.convert(missed, point);
			transform.transform(point, point_t); point_t.plusIP(transform.T);
			if(point_t.z < -0.2f)
			  map.insert(point_t.y+0.1, point_t.x+0.1, -point_t.z+0.1, false);
			 
		});

		message.getPtsObsL().forEach(obs -> {
			MavJROSUtils.convert(obs, point);
			transform.transform(point, point_t);  point_t.plusIP(transform.T);
			if(point_t.z < - 0.2f)
			  map.insert(point_t.y+0.1, point_t.x+0.1, - point_t.z+0.1, true);
		});

	}

}
