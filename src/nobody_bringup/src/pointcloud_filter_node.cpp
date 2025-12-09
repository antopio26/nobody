#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>

class PointCloudFilterNode : public rclcpp::Node
{
public:
    PointCloudFilterNode() : Node("pointcloud_filter_node")
    {
        // Declare parameters
        this->declare_parameter("input_topic", "/pointcloud");
        this->declare_parameter("output_topic", "/filtered_pointcloud");
        this->declare_parameter("bounding_box_min", std::vector<double>{-1.0, -1.0, -1.0});
        this->declare_parameter("bounding_box_max", std::vector<double>{1.0, 1.0, 1.0});
        this->declare_parameter("enable_downsampling", false);
        this->declare_parameter("voxel_size", 0.05);
        this->declare_parameter("enable_height_filter", false);
        this->declare_parameter("min_height", -std::numeric_limits<double>::infinity());
        this->declare_parameter("max_height", std::numeric_limits<double>::infinity());
        this->declare_parameter("enable_radius_filter", false);
        this->declare_parameter("max_radius", std::numeric_limits<double>::infinity());

        // Get parameters
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        
        std::vector<double> bbox_min_vec = this->get_parameter("bounding_box_min").as_double_array();
        std::vector<double> bbox_max_vec = this->get_parameter("bounding_box_max").as_double_array();
        
        if (bbox_min_vec.size() == 3) {
            bbox_min_ = Eigen::Vector4f(bbox_min_vec[0], bbox_min_vec[1], bbox_min_vec[2], 1.0);
        } else {
            RCLCPP_ERROR(this->get_logger(), "bounding_box_min must have 3 elements");
        }

        if (bbox_max_vec.size() == 3) {
            bbox_max_ = Eigen::Vector4f(bbox_max_vec[0], bbox_max_vec[1], bbox_max_vec[2], 1.0);
        } else {
            RCLCPP_ERROR(this->get_logger(), "bounding_box_max must have 3 elements");
        }

        enable_downsampling_ = this->get_parameter("enable_downsampling").as_bool();
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        enable_height_filter_ = this->get_parameter("enable_height_filter").as_bool();
        min_height_ = this->get_parameter("min_height").as_double();
        max_height_ = this->get_parameter("max_height").as_double();
        enable_radius_filter_ = this->get_parameter("enable_radius_filter").as_bool();
        max_radius_ = this->get_parameter("max_radius").as_double();

        // Create subscriber and publisher
        // Best effort QoS for sensor data
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.best_effort();
        
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, qos, std::bind(&PointCloudFilterNode::pc_callback, this, std::placeholders::_1));
        
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

        RCLCPP_INFO(this->get_logger(), "Filtering pointcloud from %s to %s", input_topic_.c_str(), output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Bounding box: min=[%f, %f, %f], max=[%f, %f, %f]", 
            bbox_min_[0], bbox_min_[1], bbox_min_[2], bbox_max_[0], bbox_max_[1], bbox_max_[2]);
    }

private:
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received pointcloud with %d points", msg->width * msg->height);
        // 1. Convert to PCL PointCloud<PointXYZ>
        // This automatically strips all fields except x, y, z
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty pointcloud");
            return;
        }

        // 2. Apply Filters

        // Box Filter (Remove points INSIDE the box)
        // The python code removed points INSIDE the box. 
        // "Filter points *outside* the bounding box (remove points inside the box)" -> logic in python was:
        // bbox_mask = ~((xyz[:,0] >= self.bbox_min[0]) & ... )
        // So we want to KEEP points that are NOT in the box.
        
        pcl::CropBox<pcl::PointXYZ> box_filter;
        box_filter.setInputCloud(cloud);
        box_filter.setMin(bbox_min_);
        box_filter.setMax(bbox_max_);
        box_filter.setNegative(true); // True means remove points inside the box
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        box_filter.filter(*cloud_filtered);

        // Height Filter
        if (enable_height_filter_) {
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud_filtered);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(min_height_, max_height_);
            // pass.setFilterLimitsNegative(false); // Default is false, keep points inside
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_height_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pass.filter(*cloud_height_filtered);
            cloud_filtered = cloud_height_filtered;
        }

        // Radius Filter
        if (enable_radius_filter_) {
            // PCL doesn't have a direct "Cylinder" filter in standard filters easily accessible like PassThrough for radius.
            // But we can iterate efficiently or use a condition.
            // Since we want high performance, a manual check might be fastest if we don't want to use ConditionEuclideanClustering or similar complex ones.
            // Let's use a simple manual iteration with `remove_if` style or just building a new cloud.
            // Actually, for "real-time", iterating once is fine.
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_radius_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            cloud_radius_filtered->header = cloud_filtered->header;
            cloud_radius_filtered->points.reserve(cloud_filtered->points.size());

            float max_radius_sq = max_radius_ * max_radius_;
            for (const auto& point : cloud_filtered->points) {
                if ((point.x * point.x + point.y * point.y) <= max_radius_sq) {
                    cloud_radius_filtered->points.push_back(point);
                }
            }
            cloud_radius_filtered->width = cloud_radius_filtered->points.size();
            cloud_radius_filtered->height = 1;
            cloud_radius_filtered->is_dense = true; // Assuming no NaNs/Inf after processing or if input was clean
            
            cloud_filtered = cloud_radius_filtered;
        }

        // Downsampling
        if (enable_downsampling_ && !cloud_filtered->empty()) {
            pcl::VoxelGrid<pcl::PointXYZ> sor;
            sor.setInputCloud(cloud_filtered);
            sor.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
            sor.filter(*cloud_downsampled);
            cloud_filtered = cloud_downsampled;
        }

        // 3. Publish
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_filtered, output_msg);
        output_msg.header = msg->header; // Preserve header (frame_id, stamp)
        
        pub_->publish(output_msg);

        RCLCPP_INFO(this->get_logger(), "Published filtered pointcloud with %lu points", cloud_filtered->points.size());
    }

    std::string input_topic_;
    std::string output_topic_;
    Eigen::Vector4f bbox_min_;
    Eigen::Vector4f bbox_max_;
    bool enable_downsampling_;
    double voxel_size_;
    bool enable_height_filter_;
    double min_height_;
    double max_height_;
    bool enable_radius_filter_;
    double max_radius_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
