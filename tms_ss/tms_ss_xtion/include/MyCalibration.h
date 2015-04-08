#ifndef _MYCALIBRATION_H_
#define _MYCALIBRATION_H_

#define CAMERA_RESOLUTION_X 640
#define CAMERA_RESOLUTION_Y 480

#define PATTERN_WIDTH 45
#define CORRESPOND_POINTS 30

void mouseCallback(int event, int x,int y, int flag, void*);

class MyCalibration
{
public:
  cv::Mat image;

  int initialize(
      pcl::visualization::CloudViewer& viewer,
      uint8_t *depth_frame_region,
      uint8_t *color_frame_region);
  int extractPlanePoints();
  int viewPoints();
  int calcurateExtrinsicParameters(
      double convertion_th = 0.01,
      int convertion_method = 2,
      float pattern_width = 45.0);
  int startPickingPoints();
  int pickPointsAutomatically(int pattern_rows, int pattern_cols);
  void getDepthFrameCallback(const sensor_msgs::Image::ConstPtr& frame);
  void getColorFrameCallback(const sensor_msgs::Image::ConstPtr& frame);
private:
  Eigen::Matrix3f correct_mirroring;
  uint8_t *depth_frame;
  uint8_t *color_frame;
  openni::VideoStream *m_video_stream;
  openni::VideoStream *m_color_stream;
  Eigen::Vector3f world_points[CORRESPOND_POINTS];
  Eigen::Vector3f picked_points[CORRESPOND_POINTS];

  std::vector<Eigen::Vector3f> all_points;

  std::vector<Eigen::Vector3f*> points_on_plane;
  std::vector<Eigen::Vector3f*> other_points;
  std::vector<Eigen::Vector3f*> picked_points_around;

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >
    cloud{new pcl::PointCloud<pcl::PointXYZRGB>()};
  pcl::visualization::CloudViewer *viewer;
};

#endif
