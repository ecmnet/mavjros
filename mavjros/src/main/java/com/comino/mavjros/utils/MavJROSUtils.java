package com.comino.mavjros.utils;


import geometry_msgs.Quaternion;
import geometry_msgs.Vector3;
import georegression.struct.GeoTuple3D_F64;
import georegression.struct.so.Quaternion_F64;

public class MavJROSUtils {

	public static GeoTuple3D_F64<?> convert(Vector3 in, GeoTuple3D_F64<?> out) {
		out.setTo(in.getX(), in.getY(), -in.getZ());
		return out;
	}
	
	public static Quaternion_F64 convert(Quaternion in, Quaternion_F64 out) {
		out.setTo(in.getW(),in.getX(), in.getY(), in.getZ());
		return out;
	}

}
