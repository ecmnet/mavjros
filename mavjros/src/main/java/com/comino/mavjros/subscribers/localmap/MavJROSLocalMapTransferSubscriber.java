package com.comino.mavjros.subscribers.localmap;

import org.mavlink.messages.lquac.msg_msp_micro_grid;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Status;
import com.comino.mavjros.MavJROSAbstractSubscriber;
import com.comino.mavjros.utils.MavJROSUtils;
import com.comino.mavutils.workqueue.WorkQueue;

import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.GeoTuple3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.so.Quaternion_F64;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.tools.OcTreeKeyConversionTools;


public class MavJROSLocalMapTransferSubscriber extends MavJROSAbstractSubscriber<glmapping.local2global> {

	private final IMAVController                       control;
	private final DataModel                            model;
	private final Vector3D_F64                         point;
	private final Vector3D_F64                         point_t;
	private final Quaternion_F64                       rotation;
	private final Se3_F64                              transform;

	public MavJROSLocalMapTransferSubscriber(IMAVController control, String rosTopicName)  {
		super(rosTopicName, glmapping.local2global._TYPE);
		this.control        = control;
		this.model          = control.getCurrentModel();

		this.point        = new Vector3D_F64();
		this.point_t      = new Vector3D_F64();
		this.rotation     = new Quaternion_F64();
		this.transform    = new Se3_F64();
	}

	@Override
	public void callback(glmapping.local2global message) {

		// Limit rate to the transfer rate to MAVGCL
		if(model.grid.hasTransfers())
			return;

		MavJROSUtils.convert(message.getTWL().getTranslation(), transform.T);
		MavJROSUtils.convert(message.getTWL().getRotation(),rotation);
		ConvertRotation3D_F64.quaternionToMatrix(rotation,transform.R);

		message.getPtsMissL().forEach(missed -> {
			MavJROSUtils.convert(missed, point);
			transform.transform(point, point_t); point_t.plusIP(transform.T);
			if(point_t.z < -0.2f) {
				model.grid.add(encode(point_t,(byte)0));
			}
		});

		message.getPtsObsL().forEach(obs -> {
			MavJROSUtils.convert(obs, point);
			transform.transform(point, point_t);  point_t.plusIP(transform.T);
			if(point_t.z < - 0.2f) {
				model.grid.add(encode(point_t,(byte)1));
			}
		});

	}

	private long encode(GeoTuple3D_F64<?> p, byte value) {
		OcTreeKey key = OcTreeKeyConversionTools.coordinateToKey(p.y+0.1f, p.x+0.1f, - p.z-0.1f, 0.2f, 16);
		return encode(key.getKey(0),key.getKey(1),key.getKey(2),value);
	}

	private long encode(int k0, int k1, int k2, int value_4bit) {
		return (((long)(value_4bit) << 60L )| (long)k0 << 40L) | ((long)k1 << 20L) | (long)k2;
	}


}
