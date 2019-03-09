// VAAPI_H264 sender
// Author: German M. Garcia

#include <ros/ros.h>

#include "YUVConverter.h"
#include "VaapiSender.h"

VaapiSender::VaapiSender(ros::NodeHandle &nh) : nh(nh), it(nh), g_lastImageTime(0)
{
	nh.param("width", width, 640);

	double rate;
	nh.param("rate", rate, 60.0);

	nh.param("crf", g_crf, 30.0);

	g_minTimeBetweenImages = ros::Duration(1.0 / rate);

	g_pub = nh.advertise<sensor_msgs::CompressedImage>("encoded", 1);

	sub = it.subscribe("/camera/rgb/image_rect_color", 5, &VaapiSender::handleImageVaapi, this);
}

int VaapiSender::set_hwframe_ctx(AVCodecContext *ctx, AVBufferRef *hw_device_ctx)
{
	AVBufferRef *hw_frames_ref;
	AVHWFramesContext *frames_ctx = NULL;
	int err = 0;

	if (!(hw_frames_ref = av_hwframe_ctx_alloc(hw_device_ctx)))
	{
		fprintf(stderr, "Failed to create VAAPI frame context.\n");
		return -1;
	}
	frames_ctx = (AVHWFramesContext *)(hw_frames_ref->data);
	frames_ctx->format = AV_PIX_FMT_VAAPI;
	frames_ctx->sw_format = AV_PIX_FMT_NV12;
	frames_ctx->width = width;
	frames_ctx->height = height;
	frames_ctx->initial_pool_size = 20;
	if ((err = av_hwframe_ctx_init(hw_frames_ref)) < 0)
	{
		ROS_ERROR_STREAM("Failed to initialize VAAPI frame context. Error code: " << err);
		av_buffer_unref(&hw_frames_ref);
		return err;
	}
	ctx->hw_frames_ctx = av_buffer_ref(hw_frames_ref);
	if (!ctx->hw_frames_ctx)
		err = AVERROR(ENOMEM);

	av_buffer_unref(&hw_frames_ref);
	return err;
}

void VaapiSender::init_sw_frame()
{
	int err = 0;
	if (!(sw_frame = av_frame_alloc()))
	{
		ROS_ERROR_STREAM("av_frame_alloc err=" << AVERROR(ENOMEM));
		return;
	}
	/* read data into software frame, and transfer them into hw frame */
	sw_frame->width = width;
	sw_frame->height = height;
	sw_frame->format = AV_PIX_FMT_YUV420P; //AV_PIX_FMT_NV12;
	if ((err = av_frame_get_buffer(sw_frame, 32)) < 0)
	{
		ROS_ERROR_STREAM("av_frame_get_buffer err=" << err);
		return;
	}
}

void VaapiSender::init_hw_frame()
{
	int err = 0;
	if (!(hw_frame = av_frame_alloc()))
	{
		ROS_ERROR_STREAM("av_frame_alloc hwframe err=" << err);
		return;
	}
	if ((err = av_hwframe_get_buffer(avctx->hw_frames_ctx, hw_frame, 0)) < 0)
	{
		ROS_ERROR_STREAM("av_frame_get_buffer err=" << err);
		return;
	}
	if (!hw_frame->hw_frames_ctx)
	{
		ROS_ERROR_STREAM("hw_frames_ctx is null");
		return;
	}
}

int VaapiSender::init_vaapi(int w, int h)
{
	int size, err;
	FILE *fin = NULL, *fout = NULL;
	AVCodec *codec = NULL;
	const char *enc_name = "h264_vaapi";

	width = w;
	height = h;
	size = width * height;

	g_inBuf.resize(width * height + width * height / 2);
	converter = new YUVConverter(width, height);

	err = av_hwdevice_ctx_create(&hw_device_ctx, AV_HWDEVICE_TYPE_VAAPI,
								 NULL, NULL, 0);
	if (err < 0)
	{
		ROS_ERROR_STREAM("Failed to create a VAAPI device. Error code: " << err);
		return -1;
	}

	if (!(codec = avcodec_find_encoder_by_name(enc_name)))
	{
		ROS_ERROR("Could not find H264 VAAPI encoder.");
		return -1;
	}

	if (!(avctx = avcodec_alloc_context3(codec)))
	{
		err = AVERROR(ENOMEM);
		return -1;
	}

	avctx->width = width;
	avctx->height = height;
	avctx->time_base = (AVRational){1, 30};
	avctx->framerate = (AVRational){30, 1};
	avctx->sample_aspect_ratio = (AVRational){1, 1};
	avctx->pix_fmt = AV_PIX_FMT_VAAPI;

	/* set hw_frames_ctx for encoder's AVCodecContext */
	if ((err = set_hwframe_ctx(avctx, hw_device_ctx)) < 0)
	{
		ROS_ERROR("Failed to set hwframe context.");
		return -1;
	}

	if ((err = avcodec_open2(avctx, codec, NULL)) < 0)
	{
		ROS_ERROR_STREAM("Cannot open video encoder codec. Error code: " << err);
		return -1;
	}

	init_sw_frame();
	init_hw_frame();
	ROS_INFO("Initialized vaapi encoder");
}

int VaapiSender::encode_write(AVFrame *frame, AVPacket &enc_pkt)
{
	int ret = 0;

	av_init_packet(&enc_pkt);
	enc_pkt.data = NULL;
	enc_pkt.size = 0;

	if ((ret = avcodec_send_frame(avctx, frame)) < 0)
	{
		ROS_ERROR_STREAM("Error code: " << ret);
		return -1;
	}

	ret = avcodec_receive_packet(avctx, &enc_pkt);
	if (ret)
		return -1;

	enc_pkt.stream_index = 0;
}

void VaapiSender::handleImageVaapi(const sensor_msgs::ImageConstPtr &img)
{
	int err;

	ros::Time now = ros::Time::now();
	if (now - g_lastImageTime < g_minTimeBetweenImages)
		return;
	g_lastImageTime = now;

	ros::Time start = ros::Time::now();

	cv_bridge::CvImageConstPtr cvImg = cv_bridge::toCvShare(img, "bgr8");
	// if not itialised
	if (!avctx)
	{
		init_vaapi(cvImg->image.cols, cvImg->image.rows);
		ROS_INFO("vaapi encoder initialised");
	}

	converter->cRGB_to_YUV420(cvImg->image.data, g_inBuf.data(), width, height);

	sw_frame->data[0] = g_inBuf.data();
	sw_frame->data[1] = g_inBuf.data() + width * height;
	sw_frame->data[2] = g_inBuf.data() + width * height + width * height / 4;

	if ((err = av_hwframe_transfer_data(hw_frame, sw_frame, 0)) < 0)
	{
		ROS_ERROR_STREAM("Error while transferring frame data to surface. err=" << err);
		return;
	}

	AVPacket enc_pkt;
	encode_write(hw_frame, enc_pkt);

	/*
	   ROS publishing
	*/
	sensor_msgs::CompressedImagePtr msg(new sensor_msgs::CompressedImage);
	msg->header = img->header;
	msg->format = "h264";
	msg->data.resize(enc_pkt.size);
	memcpy(msg->data.data(), enc_pkt.data, enc_pkt.size);

	g_pub.publish(msg);
	ROS_DEBUG("took %f", (ros::Time::now() - start).toSec());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vaapi_sender");

	ros::NodeHandle nh("~");

	VaapiSender sender(nh);

	ros::spin();

	return 0;
}
