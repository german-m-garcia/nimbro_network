#ifndef YUV_CONVERTER_H
#define YUV_CONVERTER_H


class YUVConverter
{
	public:
	YUVConverter(int width, int height);
	~YUVConverter();
	void cRGB_to_YUV420(const unsigned char * rgb, unsigned char * yuv420, int width, int height);

	private:
	void crgb_to_yuv(unsigned char   b, unsigned char   g, unsigned char   r,
					unsigned char & y, unsigned char & u, unsigned char & v);

	int _width;
	int _height;
	unsigned char * U_tmp;
	unsigned char * V_tmp;
};




#endif
