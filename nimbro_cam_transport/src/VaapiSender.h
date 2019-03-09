// VAAPI_H264 sender
// Author: German M. Garcia

#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include <sensor_msgs/CompressedImage.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <x264.h>
#define HAVE_AVHWCTX
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/pixdesc.h>
#include <libavutil/hwcontext.h>
}

#include "YUVConverter.h"

class VaapiSender
{

  public:
	VaapiSender(ros::NodeHandle &);
	void handleImageVaapi(const sensor_msgs::ImageConstPtr &img);

  private:
	int init_vaapi(int w, int h);
	void init_sw_frame();
	void init_hw_frame();
	int set_hwframe_ctx(AVCodecContext *ctx, AVBufferRef *hw_device_ctx);
	int encode_write(AVFrame *frame, AVPacket &enc_pkt);

	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Subscriber sub;

	ros::Time g_lastImageTime;
	ros::Duration g_minTimeBetweenImages;

	std::vector<uint8_t> g_inBuf;

	ros::Publisher g_pub;

	double g_crf;

	int width, height;
	AVBufferRef *hw_device_ctx = NULL;
	AVCodecContext *avctx = NULL;
	AVFrame *sw_frame = NULL, *hw_frame = NULL;
	cv::Mat resized;
	YUVConverter *converter = NULL;
};
