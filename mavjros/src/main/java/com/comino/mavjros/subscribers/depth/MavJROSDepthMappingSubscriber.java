package com.comino.mavjros.subscribers.depth;

import static boofcv.factory.distort.LensDistortionFactory.narrow;

import java.nio.ShortBuffer;
import java.util.ArrayList;
import java.util.List;

import org.ddogleg.struct.DogArray_I16;

import com.comino.mavcom.model.DataModel;
import com.comino.mavjros.MavJROSAbstractSubscriber;
import com.comino.mavmap.map.map3D.impl.octomap.MAVOctoMap3D;
import com.comino.mavmap.map.map3D.impl.octomap.tools.MAVOctoMapTools;
import com.comino.mavodometry.video.IVisualStreamHandler;
import com.comino.mavutils.legacy.ExecutorService;

import boofcv.concurrency.BoofConcurrency;
import boofcv.struct.calib.CameraPinholeBrown;
import boofcv.struct.distort.Point2Transform2_F64;
import boofcv.struct.geo.Point2D3D;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.se.Se3_F64;
import sensor_msgs.Image;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.jOctoMap.pointCloud.PointCloud;

public class MavJROSDepthMappingSubscriber extends MavJROSAbstractSubscriber<sensor_msgs.Image> {

	private static final int DEPTH_DELAY_MS = 20;

	private final Planar<GrayU8>                       depth_image;
	private final IVisualStreamHandler<Planar<GrayU8>> stream;
	private final DataModel                            model;
	private final MAVOctoMap3D                         map;
	private final Point2Transform2_F64 		           p2n;

	private final DogArray_I16 worker   = new DogArray_I16();
	private final PointCloud   scan     = new PointCloud();
	private  Se3_F64      to_ned   = new Se3_F64();
	private final Point2D3D    ned_p    = new Point2D3D();
	private final Point2D_F64  norm     = new Point2D_F64();

	private final Point2D3D    tmp_p = new Point2D3D();

	private long tms, tms_old;
	private float dt_sec, dt_sec_1;
	private int count;


	public MavJROSDepthMappingSubscriber(DataModel model,MAVOctoMap3D map, String rosTopicName, int width, int height, IVisualStreamHandler<Planar<GrayU8>> stream)  {
		super(rosTopicName, sensor_msgs.Image._TYPE);

		this.depth_image  = new Planar<GrayU8>(GrayU8.class,width,height,3);
		this.stream       = stream;
		this.model        = model;
		this.map          = map;

		CameraPinholeBrown instrinsics = new CameraPinholeBrown();
		instrinsics.cx = 320f;
		instrinsics.cy = 240f;
		instrinsics.fx = 347.99755859375f;
		instrinsics.fy = 347.99755859375f;

		p2n = (narrow(instrinsics)).undistort_F64(true,false);

	}

	@Override
	public void callback(Image message) {

		to_ned = model.getBodyToNedBuffer().get(System.currentTimeMillis() - DEPTH_DELAY_MS );
		convert(message.getData().toByteBuffer().asShortBuffer(), depth_image.height,depth_image.width, depth_image, worker);
		stream.addToStream("DEPTH", depth_image, model, System.currentTimeMillis());

		if((count++) % 4 == 0) {
			ExecutorService.submit(() -> {
				map.getTree().insertPointCloud(scan, new Point3D(to_ned.T.x, to_ned.T.y, -to_ned.T.z));
			});
			
			tms = message.getHeader().getStamp().totalNsecs()/1000_000L;
			dt_sec   = (tms - tms_old) / 1000f;
			if(dt_sec > 0) {
				dt_sec_1 = 4.0f / dt_sec;
				model.slam.fps = model.slam.fps * 0.95f + 0.05f*(float)dt_sec_1;
			}
			tms_old = tms;
		}

	}

	
	// TODO: Avoid double entries in the scan to reduce load
	private  synchronized void convert(ShortBuffer src , int height , int srcStride ,Planar<GrayU8> dst , DogArray_I16 work ){
		work.resize(dst.width);

		GrayU8 r = dst.getBand(2);
		dst.getBand(0).data = r.data;
		dst.getBand(1).data = r.data;

		scan.clear();
	//	BoofConcurrency.loopFor(0, height, y -> {
		
		for (int y = 0; y < height; y++) {	
			src.position(y*srcStride);
			src.get(work.data,0,work.size);

			int indexDst = dst.startIndex + dst.stride * y;
			for (int i = 0; i < work.size; indexDst++) {
				
				if(y > 120 && y <360  && project(i,y,work.data[i],tmp_p)) {
					GeometryMath_F64.mult(to_ned.R, tmp_p.location, ned_p.location );
					ned_p.location.plusIP(to_ned.T); 
					if(ned_p.location.z < -0.2)
						MAVOctoMapTools.addToPointCloud(scan, ned_p.location);
				}
				
				r.data[indexDst] = (byte)(work.data[i]/40);
				i++;
			}
		}
	}

	private boolean project(int x, int y, int depth, Point2D3D p) {

		p.observation.x = x;
		p.observation.y = y;

		p.location.x = (depth ) * 1e-3;
		if(p.location.x < 0.3 || p.location.x > 5.0)
			return false;

		p2n.compute(p.observation.x,p.observation.y,norm);

		p.location.y =     p.location.x * norm.x;
		p.location.z =     p.location.x * norm.y;

		return true;
	}
}
