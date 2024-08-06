#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

class ExampleImgNode
{
public:
    ExampleImgNode()
        : it_(nh_)
    {
        image_sub_ = it_.subscribe("/oak/rgb/image_raw", 1, &ExampleImgNode::imageCallback, this);
        image_pub_ = it_.advertise("/camera/cross_image", 1);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Draw a cross in the middle of the image
        int img_center_x = cv_ptr->image.cols / 2;
        int img_center_y = cv_ptr->image.rows / 2;
        cv::drawMarker(cv_ptr->image, cv::Point(img_center_x, img_center_y), CV_RGB(255,0,0), cv::MARKER_CROSS, 40, 2);

        // Show the image with the cross
        cv::imshow("Cross Image", cv_ptr->image);
        cv::waitKey(30);

        // Publish the image with the cross
        image_pub_.publish(cv_ptr->toImageMsg());
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "example_img_node");
    ExampleImgNode node;
    ros::spin();
    return 0;
}
