package com.comino.mavjros.subscribers.rgb;

import java.nio.ByteBuffer;

import org.ddogleg.struct.DogArray_I8;

import com.comino.mavcom.model.DataModel;
import com.comino.mavjros.MavJROSAbstractSubscriber;
import com.comino.mavodometry.video.IVisualStreamHandler;

import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import sensor_msgs.Image;

public class MavJROSRGBSubscriber extends MavJROSAbstractSubscriber<sensor_msgs.Image> {
	
	private final Planar<GrayU8>                       rgb_image;
	private final IVisualStreamHandler<Planar<GrayU8>> stream;
	private final DataModel                            model;
	
	private final DogArray_I8 worker = new DogArray_I8();

	public MavJROSRGBSubscriber(DataModel model, String rosTopicName, int width, int height, IVisualStreamHandler<Planar<GrayU8>> stream)  {
		super(rosTopicName, sensor_msgs.Image._TYPE);
		
		this.rgb_image    = new Planar<GrayU8>(GrayU8.class,width,height,3);
		this.stream       = stream;
		this.model        = model;
		
	}

	@Override
	public void callback(Image message) {
		convert(message.getData().toByteBuffer(), message.getHeight(), 1920, rgb_image, worker);
		stream.addToStream("RGB", rgb_image, model, System.currentTimeMillis());
		
	}
	
	private  void convert(ByteBuffer src , int height , int srcStride ,Planar<GrayU8> dst , DogArray_I8 work ){
		work.resize(dst.width*3);

		GrayU8 r = dst.getBand(2);
		GrayU8 g = dst.getBand(0);
		GrayU8 b = dst.getBand(1);

		int indexSrc = 0;
		for (int y = 0; y < height; y++) {
			src.position(indexSrc);
			src.get(work.data,0,work.size);

			int indexDst = dst.startIndex + dst.stride * y;
			for (int i = 0; i < work.size; indexDst++) {
				r.data[indexDst] = work.data[i++];
				g.data[indexDst] = work.data[i++];
				b.data[indexDst] = work.data[i++];
			}
			indexSrc += srcStride;
		}
	}

}
