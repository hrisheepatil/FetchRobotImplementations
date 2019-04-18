#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <limits.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/angles.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <fetch_perc/SegmentedClustersArray.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

static const std::string IMAGE_TOPIC = "/head_camera/depth_registered/points";
static const std::string PUBLISH_TOPIC_PT = "/pcl/points_passthrough";
static const std::string PUBLISH_TOPIC_RANSAC_SURF = "/pcl/points_ransac_surf";
static const std::string PUBLISH_TOPIC_RANSAC_OBJ = "/pcl/points_ransac_obj";
static const std::string PUBLISH_TOPIC_CLUSTERS = "/pcl/points_ransac_clusters";
static const std::string PUBLISH_TOPIC_OBJ1 = "/pcl/points_obj1";
static const std::string PUBLISH_TOPIC_OBJ2 = "/pcl/points_obj2";
static const std::string PUBLISH_TOPIC_POSE = "/pose";
ros::Publisher pub_pt;
ros::Publisher pub_ransac_obj;
ros::Publisher pub_ransac_surf;
ros::Publisher pub_pose;
ros::Publisher pub_clusters;
ros::Publisher pub_obj1;
ros::Publisher pub_obj2;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)  
{
	// Calculating transform from camera to base

	sensor_msgs::PointCloud2ConstPtr cloudcam = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("head_camera/depth_registered/points");
	tf::TransformListener tf_listener;
	tf_listener.waitForTransform("base_link", cloudcam->header.frame_id, ros::Time(0), ros::Duration(5.0)); 
        tf::StampedTransform transform;
	tf_listener.lookupTransform("base_link", cloudcam->header.frame_id, ros::Time(0), transform);
	//sensor_msgs::PointCloud2 cloud_out;
  	//pcl_ros::transformPointCloud("base_link", transform, *cloudcam, cloud_out);
	//pub.publish(cloud_out);
	

	// Converting cloud_msg to cloud

	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);
	pcl_conversions::toPCL(*cloud_msg, *cloud);

	// Voxel Grid Downsampling

	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud (cloudPtr);
        sor.setLeafSize (0.01, 0.01, 0.01);
        sor.filter (*cloudFilteredPtr);

	// Passthrough filtering

	pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
  	pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr (xyz_cloud);
	pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr); 
	
	pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);

	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud (xyzCloudPtr);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.5, 1.1);
	pass.filter (*xyzCloudPtrFiltered);
	sensor_msgs::PointCloud2 output_pt;
	pcl::toROSMsg(*xyzCloudPtrFiltered, output_pt);
	pub_pt.publish (output_pt);

	// Model Coefficients after RANSAC

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZRGB> seg1;
	seg1.setOptimizeCoefficients (true);
	seg1.setModelType (pcl::SACMODEL_PLANE);
	seg1.setMethodType (pcl::SAC_RANSAC);
	seg1.setDistanceThreshold (0.04);
	seg1.setInputCloud (xyzCloudPtrFiltered);
	seg1.segment (*inliers, *coefficients);
	//pcl_msgs::ModelCoefficients ros_coefficients;
	//pcl_conversions::fromPCL(*coefficients, ros_coefficients);		
	//pub.publish (ros_coefficients);


	// Ransac Filtering for segmentation

	pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_ransac_filtered_surf = new pcl::PointCloud<pcl::PointXYZRGB>;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacSurfFiltered (xyz_cloud_ransac_filtered_surf);
	pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_ransac_filtered_obj = new pcl::PointCloud<pcl::PointXYZRGB>;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacObjFiltered (xyz_cloud_ransac_filtered_obj);

	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	pcl::PointIndices::Ptr above_inliers (new pcl::PointIndices);
	pcl::PointIndices::Ptr surface_inliers (new pcl::PointIndices);
	extract.setInputCloud (xyzCloudPtrFiltered);
	extract.setIndices (inliers);

	extract.setNegative (false);
	extract.filter (*xyzCloudPtrRansacSurfFiltered);
	extract.filter(surface_inliers->indices);

	extract.setNegative (true);
	extract.filter (*xyzCloudPtrRansacObjFiltered);
	extract.filter(above_inliers->indices);
	//ROS_INFO("There are %ld points above the table", above_inliers->indices.size());
	
	sensor_msgs::PointCloud2 output_ransac_surf;
	pcl::toROSMsg(*xyzCloudPtrRansacSurfFiltered, output_ransac_surf);
	pub_ransac_surf.publish (output_ransac_surf);

	sensor_msgs::PointCloud2 output_ransac_obj;
	pcl::toROSMsg(*xyzCloudPtrRansacObjFiltered, output_ransac_obj);
	pub_ransac_obj.publish (output_ransac_obj);

	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> euclid;
	std::vector<pcl::PointIndices> object_indices;
	euclid.setInputCloud(xyzCloudPtrRansacObjFiltered);
	//euclid.setIndices(above_inliers);
	euclid.setClusterTolerance(0.02);
	euclid.setMinClusterSize(10);
	euclid.setMaxClusterSize(200);
	euclid.extract(object_indices);

	size_t min_size = std::numeric_limits<size_t>::max();
	size_t max_size = std::numeric_limits<size_t>::min();
	for (size_t i = 0; i < object_indices.size(); ++i) 
	{
		size_t cluster_size = (object_indices)[i].indices.size();
		if (cluster_size < min_size) 
		{
			min_size = cluster_size;
		}
		if (cluster_size > max_size) 
		{
			max_size = cluster_size;
		}
	}

  	ROS_INFO("Found %ld objects, min size: %ld, max size: %ld", object_indices.size(), min_size, max_size);	

	pcl::ExtractIndices<pcl::PointXYZRGB> extract1;
	pcl::PointIndices::Ptr indices1(new pcl::PointIndices);
	*indices1 = object_indices[0];
	extract1.setIndices(indices1);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
	extract1.filter(*object_cloud1);
	sensor_msgs::PointCloud2 output_obj1;
	pcl::toROSMsg(*object_cloud1, output_obj1);
	pub_obj1.publish (output_obj1);

	pcl::ExtractIndices<pcl::PointXYZRGB> extract2;
	pcl::PointIndices::Ptr indices2(new pcl::PointIndices);
	*indices2 = object_indices[1];
	extract2.setIndices(indices2);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
	extract2.filter(*object_cloud2);
	sensor_msgs::PointCloud2 output_obj2;
	pcl::toROSMsg(*object_cloud2, output_obj2);
	pub_obj2.publish (output_obj2);
	

	// Cluster Indices collection after segmentation

	/*pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud (xyzCloudPtrRansacObjFiltered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setInputCloud (xyzCloudPtrRansacObjFiltered);
	ec.setClusterTolerance (0.01); 
	ec.setMinClusterSize (10);
	ec.setMaxClusterSize (200);
	ec.setSearchMethod (tree);
	ec.extract (cluster_indices);
	
	fetch_perc::SegmentedClustersArray CloudClusters;
	sensor_msgs::PointCloud2 output;
  	pcl::PCLPointCloud2 outputPCL;	
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  	{
		pcl::PointCloud<pcl::PointXYZRGB> *cluster = new pcl::PointCloud<pcl::PointXYZRGB>;
    		pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr (cluster);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    		{
      			clusterPtr->points.push_back(xyzCloudPtrRansacObjFiltered->points[*pit]);

        	}
		pcl::toPCLPointCloud2( *clusterPtr ,outputPCL);
    		pcl_conversions::fromPCL(outputPCL, output);
		CloudClusters.clusters.push_back(output);
	}
	pub_clusters.publish(CloudClusters);*/


	// Calculating pose of the object

	/*pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_ransac_filtered_out = new pcl::PointCloud<pcl::PointXYZRGB>;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacFiltered_out (xyz_cloud_ransac_filtered_out);
	pcl_ros::transformPointCloud(*xyzCloudPtrRansacFiltered, *xyzCloudPtrRansacFiltered_out, transform);

	geometry_msgs::Pose pose;
	geometry_msgs::Vector3 dimensions;
	Eigen::Vector4f min_pt, max_pt;
	pcl::getMinMax3D(*xyzCloudPtrRansacFiltered_out, min_pt, max_pt);
	pose.position.x = (max_pt.x() + min_pt.x()) / 2;
	pose.position.y = (max_pt.y() + min_pt.y()) / 2;
	pose.position.z = (max_pt.z() + min_pt.z()) / 2;
	pose.orientation.w = 1;
	dimensions.x = max_pt.x() - min_pt.x();
	dimensions.y = max_pt.y() - min_pt.y();
	dimensions.z = max_pt.z() - min_pt.z();	
	pub_pose.publish(pose);*/
	
		
}

int main (int argc, char** argv) 
{
	ros::init(argc, argv, "point_segment");
	ros::NodeHandle nh;

	ROS_INFO_STREAM("Hello from ROS Node: " << ros::this_node::getName());

	ros::Subscriber sub = nh.subscribe(IMAGE_TOPIC, 1, cloud_cb);
	
	pub_pt = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC_PT, 1);	
	pub_ransac_surf = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC_RANSAC_SURF, 1);
	pub_ransac_obj = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC_RANSAC_OBJ, 1);
	pub_clusters = nh.advertise<fetch_perc::SegmentedClustersArray> (PUBLISH_TOPIC_CLUSTERS,1);
	pub_obj1 = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC_OBJ1, 1);
	pub_obj2 = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC_OBJ2, 1);
	//pub = nh.advertise<pcl_msgs::ModelCoefficients> (PUBLISH_TOPIC, 1);
        //pub_pose = nh.advertise<geometry_msgs::Pose>(PUBLISH_TOPIC_POSE, 1);
	//pub = nh.advertise<geometry_msgs::Vector3>(PUBLISH_TOPIC, 1);
	ros::spin();

	return 0;
}
