
#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <bobi_vision/camera_config.hpp>

#include <bobi_msgs/ConvertCoordinates.h>

using namespace bobi;

class CoordinateMapper {
public:
    CoordinateMapper(
        std::shared_ptr<ros::NodeHandle> nh,
        const std::string& service_name,
        const std::vector<cv::Point2d>& world_points,
        const std::vector<cv::Point2d>& image_points,
        const std::vector<double> camera_matrix,
        const std::vector<double> distortion_coeffs,
        double pix2m,
        int camera_px_width,
        int camera_px_height) : _nh(nh), _service_name(service_name)
    {
        // compute opencv vars
        _distortion_coeffs = cv::Mat(1, 5, CV_64F);
        memcpy(_distortion_coeffs.data, distortion_coeffs.data(), distortion_coeffs.size() * sizeof(double));
        _distortion_coeffs *= pix2m;

        _opt_camera_matrix = cv::Mat(3, 3, CV_64F);
        memcpy(_opt_camera_matrix.data, camera_matrix.data(), camera_matrix.size() * sizeof(double));
        _opt_camera_matrix *= pix2m;

        std::vector<cv::Point3d> world_points3d;
        for (const cv::Point2d& p : world_points) {
            world_points3d.push_back(cv::Point3d(p.x, p.y, 0.));
        }

        _rvec = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
        _tvec = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
        cv::solvePnP(world_points3d, image_points, _opt_camera_matrix, _distortion_coeffs, _rvec, _tvec, true, cv::SOLVEPNP_EPNP);

        // init ROS vars
        _converter_srv = _nh->advertiseService(_service_name, &CoordinateMapper::_converter_srv_cb, this);
    }

    cv::Point2d convert(cv::Point3d wpoint) const
    {
        std::vector<cv::Point3d> wpoints = {wpoint};
        std::vector<cv::Point2d> projected;
        cv::projectPoints(wpoints, _rvec, _tvec, _opt_camera_matrix, _distortion_coeffs, projected);
        return projected[0];
    }

protected:
    bool _converter_srv_cb(
        bobi_msgs::ConvertCoordinates::Request& req,
        bobi_msgs::ConvertCoordinates::Response& res)
    {
        cv::Point3d to_convert;
        to_convert.x = req.p.x;
        to_convert.y = req.p.y;
        to_convert.z = req.p.z;
        cv::Point2d converted = convert(to_convert);
        res.converted_p.x = converted.x;
        res.converted_p.y = converted.y;
        return true;
    }

    cv::Mat _opt_camera_matrix;
    cv::Mat _distortion_coeffs;
    cv::Mat _rvec;
    cv::Mat _tvec;
    std::shared_ptr<ros::NodeHandle> _nh;
    std::string _service_name;

    ros::ServiceServer _converter_srv;
};

template <typename T>
std::vector<T> load_point_mat(std::shared_ptr<ros::NodeHandle> nh, const std::string& param_name)
{
    std::vector<T> points;

    XmlRpc::XmlRpcValue xmlrpc_points;
    nh->getParam(param_name, xmlrpc_points);
    if (xmlrpc_points.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (size_t i = 0; i < xmlrpc_points.size(); ++i) {
            T point;
            point.x = xmlrpc_points[i][0];
            point.y = xmlrpc_points[i][1];
            points.push_back(point);
        }
    }

    return points;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "coordinate_converter_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    std::vector<cv::Point2d> points_bottom = load_point_mat<cv::Point2d>(nh, "points_bottom");
    std::vector<cv::Point2d> points_top = load_point_mat<cv::Point2d>(nh, "points_top");

    top::CameraConfig top_cfg = top::get_camera_config(*nh);
    bottom::CameraConfig bottom_cfg = bottom::get_camera_config(*nh);

    CoordinateMapper top2bottom(nh, "convert_top2bottom", points_bottom, points_top, top_cfg.camera_matrix, top_cfg.distortion_coeffs, top_cfg.pix2m, top_cfg.camera_px_width, top_cfg.camera_px_height);
    CoordinateMapper bottom2top(nh, "convert_bottom2top", points_top, points_bottom, bottom_cfg.camera_matrix, bottom_cfg.distortion_coeffs, bottom_cfg.pix2m, bottom_cfg.camera_px_width, bottom_cfg.camera_px_height);

    ros::spin();

    return 0;
}