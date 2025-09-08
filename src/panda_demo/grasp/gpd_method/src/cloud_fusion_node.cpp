// cloud_fusion_node.cpp
// 将三路 PointCloud2（通常为 /cam1|2|3/depth/points）变换到同一坐标系并合并，
// 发布到单一路径（默认 /fusion/points），供 GPD 的 cloud_topic 直接订阅。
// Build: catkin
// Run: rosrun <your_pkg> cloud_fusion_node _cloud1_topic:=/cam1/depth/points _cloud2_topic:=/cam2/depth/points _cloud3_topic:=/cam3/depth/points _target_frame:=base_link _out_topic:=/fusion/points

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/io.h>

class CloudFusionNode {
public:
  CloudFusionNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh), tf_buffer_(), tf_listener_(tf_buffer_) {
    // params
    pnh_.param<std::string>("cloud1_topic", cloud1_topic_, std::string("/cam1/depth/points"));
    pnh_.param<std::string>("cloud2_topic", cloud2_topic_, std::string("/cam2/depth/points"));
    pnh_.param<std::string>("cloud3_topic", cloud3_topic_, std::string("/cam3/depth/points"));
    pnh_.param<std::string>("target_frame", target_frame_, std::string("panda_link0"));
    pnh_.param<std::string>("out_topic", out_topic_, std::string("/fusion/points"));
    pnh_.param<int>("queue_size", queue_size_, 5);
    pnh_.param<bool>("approximate_sync", approximate_sync_, true);
    pnh_.param<double>("voxel_leaf", voxel_leaf_, 0.0); // 0 表示不下采样

    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(out_topic_, 1, false);

    s1_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, cloud1_topic_, queue_size_));
    s2_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, cloud2_topic_, queue_size_));
    s3_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, cloud3_topic_, queue_size_));

    if (approximate_sync_) {
      using Policy = message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>;
      sync_approx_.reset(new message_filters::Synchronizer<Policy>(Policy(queue_size_), *s1_, *s2_, *s3_));
      sync_approx_->registerCallback(boost::bind(&CloudFusionNode::cb, this, _1, _2, _3));
    } else {
      using Policy = message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>;
      sync_exact_.reset(new message_filters::Synchronizer<Policy>(Policy(queue_size_), *s1_, *s2_, *s3_));
      sync_exact_->registerCallback(boost::bind(&CloudFusionNode::cb, this, _1, _2, _3));
    }

    ROS_INFO_STREAM("cloud_fusion_node ready: target_frame='" << target_frame_ << "', out_topic='" << out_topic_ << "'.");
  }

private:
  void cb(const sensor_msgs::PointCloud2::ConstPtr& c1,
          const sensor_msgs::PointCloud2::ConstPtr& c2,
          const sensor_msgs::PointCloud2::ConstPtr& c3) {
    try {
      sensor_msgs::PointCloud2 t1, t2, t3;
      // 将三路点云变换到 target_frame_
      transformToTarget(*c1, t1);
      transformToTarget(*c2, t2);
      transformToTarget(*c3, t3);

      // 合并：使用 PCL 的 PCLPointCloud2 + concatenatePointCloud
      pcl::PCLPointCloud2 pc1, pc2, pc3, pc12, pc123;
      pcl_conversions::toPCL(t1, pc1);
      pcl_conversions::toPCL(t2, pc2);
      pcl_conversions::toPCL(t3, pc3);

      pcl::concatenatePointCloud(pc1, pc2, pc12);
      pcl::concatenatePointCloud(pc12, pc3, pc123);

      // 可选体素下采样，减少点数
      if (voxel_leaf_ > 0.0) {
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(boost::make_shared<pcl::PCLPointCloud2>(pc123));
        sor.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
        pcl::PCLPointCloud2 filtered;
        sor.filter(filtered);
        pc123 = filtered;

      }

      sensor_msgs::PointCloud2 out;
      pcl_conversions::fromPCL(pc123, out);
      out.header.frame_id = target_frame_;
      out.header.stamp = std::max(c1->header.stamp, std::max(c2->header.stamp, c3->header.stamp));
      pub_.publish(out);
    }
    catch (const tf2::TransformException& ex) {
      ROS_WARN_STREAM_THROTTLE(1.0, "TF transform failed: " << ex.what());
    }
    catch (const std::exception& e) {
      ROS_ERROR_STREAM("cloud_fusion_node error: " << e.what());
    }
  }

  void transformToTarget(const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out) {
    if (in.header.frame_id == target_frame_) {
      out = in; // 直接复用
      return;
    }
    geometry_msgs::TransformStamped T = tf_buffer_.lookupTransform(
        target_frame_, in.header.frame_id, in.header.stamp, ros::Duration(0.2));
    tf2::doTransform(in, out, T); // tf2_sensor_msgs 对 PointCloud2 的特化
  }

  ros::NodeHandle nh_, pnh_;
  std::string cloud1_topic_, cloud2_topic_, cloud3_topic_;
  std::string target_frame_, out_topic_;
  int queue_size_;
  bool approximate_sync_;
  double voxel_leaf_;

  ros::Publisher pub_;
  std::unique_ptr< message_filters::Subscriber<sensor_msgs::PointCloud2> > s1_, s2_, s3_;
  std::unique_ptr< message_filters::Synchronizer< message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> > > sync_approx_;
  std::unique_ptr< message_filters::Synchronizer< message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> > > sync_exact_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_fusion_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  CloudFusionNode node(nh, pnh);
  ros::spin();
  return 0;
}
