//This is the fucntion that is responsible to
//making face detections and tracking decisions for cmt_tracker_node.

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

static const std::string OPENCV_WINDOW = "Image window";
class ImageConverter
{
  int counter;
  geometry_msgs::Point face_points;
  cv::CascadeClassifier face_cascade;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  //List of face values in space.
  ros::Publisher faces_locations;
  std::vector<cv::Rect> faces;
  int time_sec;
  std_msgs::String tracking_method;
  bool setup;
public:
  ImageConverter()
    : it_(nh_)
  {
    counter = 0;

    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
                               &ImageConverter::imageCb, this);
    // image_pub_ = it_.advertise("/image_converter/output_video", 1);
    
    //Later refactor this to load from the xml documents of the file.
    if ( !face_cascade.load("/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml" ) )
    { setup = false;  };
    setup = true;
    cv::namedWindow(OPENCV_WINDOW);
    //Get time it takes to set the tracking values.
    nh_.getParam("tracker_set_time", time_sec);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    if (setup)
    {
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        // cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      // Draw an example circle on the video stream
      // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

      cv::Mat frame_gray;
      cv::cvtColor(cv_ptr->image, frame_gray, cv::COLOR_BGR2GRAY);
      cv::equalizeHist( frame_gray, frame_gray );

      face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30) );

      //TODO: namespace mapping to the system.
      std::string tracking;
      nh_.getParam("tracking_method", tracking);
      //Process tracking_method.data
      if (tracking.compare("handtracking") != 0)
      {
        // ROS_DEBUG("Using Methods of Hand Tracking Methods");



        for (size_t i = 0; i < faces.size(); i++)
        {
          cv::Point left( faces[i].x, faces[i].y);
          cv::Point oLeft(faces[i].x + faces[i].width , faces[i].y + faces[i].height);
          cv::Point center(faces[i].x + faces[i].width / 2, faces[i].y + faces[i].height / 2 );
          cv::rectangle(frame_gray, left, oLeft, 0);

        }


      }
      else
      {
        // ROS_DEBUG("Using Methods of Optical Flow Method");

      }
      // Update GUI Window
      cv::imshow(OPENCV_WINDOW, frame_gray);
      cv::waitKey(3);

      // Output modified video stream
      // image_pub_.publish(cv_ptr->toImageMsg());
    }
  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "face_locator");
  ImageConverter ic;
  ros::spin();
  return 0;
}