package com.comino.mavjros.subscribers.rgb;

import java.nio.ByteBuffer;
import java.nio.ShortBuffer;

import org.ddogleg.struct.DogArray_I16;
import org.ddogleg.struct.DogArray_I8;

import com.comino.mavcom.model.DataModel;
import com.comino.mavjros.MavJROSAbstractSubscriber;
import com.comino.mavodometry.video.IVisualStreamHandler;

import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import sensor_msgs.Image;

public class MavJROSDepthSubscriber extends MavJROSAbstractSubscriber<sensor_msgs.Image> {
	
	private final Planar<GrayU8>                       depth_image;
	private final IVisualStreamHandler<Planar<GrayU8>> stream;
	private final DataModel                            model;

	private final DogArray_I16 worker = new DogArray_I16();

	public MavJROSDepthSubscriber(DataModel model, String rosTopicName, int width, int height, IVisualStreamHandler<Planar<GrayU8>> stream)  {
		super(rosTopicName, sensor_msgs.Image._TYPE);
		
		this.depth_image    = new Planar<GrayU8>(GrayU8.class,width,height,3);
		this.stream       = stream;
		this.model        = model;
		
	}

	@Override
	public void callback(Image message) {
		convert(message.getData().toByteBuffer().asShortBuffer(), message.getHeight(), 640, depth_image, worker);
		stream.addToStream("DEPTH", depth_image, model, System.currentTimeMillis());		
	}
	
	private  void convert(ShortBuffer src , int height , int srcStride ,Planar<GrayU8> dst , DogArray_I16 work ){
		work.resize(dst.width);

		GrayU8 r = dst.getBand(2);
		dst.getBand(0).data = r.data;
		dst.getBand(1).data = r.data;

		int indexSrc = 0;
		for (int y = 0; y < height; y++) {
			src.position(indexSrc);
			src.get(work.data,0,work.size);

			int indexDst = dst.startIndex + dst.stride * y;
			for (int i = 0; i < work.size; indexDst++) {
				r.data[indexDst] = (byte)(work.data[i++]/40);
			}
			indexSrc += srcStride;
		}
	}


}
