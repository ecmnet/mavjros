package com.comino.mavjros.subscribers.odometry;

import org.mavlink.messages.lquac.msg_msp_vision;
import org.mavlink.messages.lquac.msg_odometry;

import com.comino.mavcom.control.IMAVController;
import com.comino.mavcom.model.DataModel;
import com.comino.mavcom.model.segment.Vision;
import com.comino.mavcom.struct.Attitude3D_F64;
import com.comino.mavjros.MavJROSAbstractSubscriber;

import georegression.struct.point.Vector4D_F64;
import georegression.struct.se.Se3_F64;
import nav_msgs.Odometry;

public class MavJROSOdometrySubscriber extends MavJROSAbstractSubscriber<nav_msgs.Odometry> {

	
   private final DataModel                    model;
   private final IMAVController               control;
   
   private final msg_msp_vision               msg = new msg_msp_vision(2,1);
   private final msg_odometry                 odo = new msg_odometry(1,1);
   
   private float quality = 1;
   private int   error_count;
   private long  tms;
   private long  tms_old;
   private double   dt_sec = 0;
   private double   dt_sec_1 = 0;
   
   private final Attitude3D_F64   att      	   = new Attitude3D_F64();
   private final Se3_F64          ned      	   = new Se3_F64();
   private final Se3_F64          ned_s    	   = new Se3_F64();
   private final Se3_F64          body_a   	   = new Se3_F64();
   
   private final Vector4D_F64     precision_lock    = new Vector4D_F64();
   

	public MavJROSOdometrySubscriber(IMAVController control, String rosTopicName)  {
		super(rosTopicName, nav_msgs.Odometry._TYPE);

		this.control = control;
		this.model = control.getCurrentModel();
		
		model.vision.setStatus(Vision.ENABLED, true);
		model.vision.setStatus(Vision.AVAILABLE, true);

	}

	@Override
	public void callback(Odometry message) {
		
		model.vision.setStatus(Vision.SPEED_VALID, true);
		model.vision.setStatus(Vision.POS_VALID, true);
		model.vision.setStatus(Vision.EXPERIMENTAL, false);
		
		ned.T.x =  message.getPose().getPose().getPosition().getY();
		ned.T.y =  message.getPose().getPose().getPosition().getX();
		ned.T.z = -message.getPose().getPose().getPosition().getZ();
		
		ned_s.T.x =  message.getTwist().getTwist().getLinear().getY();
		ned_s.T.y =  message.getTwist().getTwist().getLinear().getX();
		ned_s.T.z = -message.getTwist().getTwist().getLinear().getZ();
		
		tms = System.currentTimeMillis();
		
		dt_sec   = (tms - tms_old) / 1000f;
		if(dt_sec < 0.03)
			return;
		
		dt_sec_1 = 1.0 / dt_sec;
		model.vision.fps = (float)dt_sec_1;
		tms_old = tms;
		
		publishMSPVision(ned,ned_s,body_a,precision_lock,tms);
		
		
	}
	
	private void publishMSPVision(Se3_F64 pose, Se3_F64 speed, Se3_F64 acc_body, Vector4D_F64 offset,long tms) {


		msg.x =  (float) pose.T.x;
		msg.y =  (float) pose.T.y;
		msg.z =  (float) pose.T.z;

		msg.vx =  (float) speed.T.x;
		msg.vy =  (float) speed.T.y;
		msg.vz =  (float) speed.T.z;

		msg.ax =  (float) acc_body.T.x;
		msg.ay =  (float) acc_body.T.y;
		msg.az =  (float) acc_body.T.z;

		msg.px =  (float)offset.x;
		msg.py =  (float)offset.y;
		msg.pz =  (float)offset.z;
		msg.pw =  (float)offset.w;

		msg.h   = (float)att.getYaw();
		msg.r   = (float)att.getRoll();
		msg.p   = (float)att.getPitch();

		msg.quality = (int)(quality * 100f);
		msg.errors  = error_count;
		msg.tms     = tms * 1000;
		msg.flags   = model.vision.flags;
		msg.fps     = model.vision.fps;

		control.sendMAVLinkMessage(msg);


	}

}
