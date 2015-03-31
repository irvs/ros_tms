#ifndef DEPTH_IMAGE_MAKER_H_
#define DEPTH_IMAGE_MAKER_H_

class DepthImageMaker : public openni::VideoStream::NewFrameListener
{
  public:
    cv::Mat image;
    openni::VideoFrameRef frame;
    void onNewFrame(openni::VideoStream& stream);
  private:
};

#endif
