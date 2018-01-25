#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include "std_msgs/String.h"
#include<iostream>

#include <sstream>


#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/region_growing.h>

#include "/home/mayank/catkin_ws/src/grasp_segmentation/src/clusterAnalysis.h"

//Markers
#include <visualization_msgs/Marker.h>

ros::Publisher pub;
ros::Publisher vis_pub;


void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
  output = *input;

  // Publish the data.
  pub.publish (output);
}

void publish_grip(int clusterNum, pcl::PointCloud<pcl::PointXYZ>::Ptr clusterPtr, Eigen::Vector3f middle_vector, ros::Publisher &vis_pub)
{
		visualization_msgs::Marker gripDir;
		// Set our initial shape type to be a cube
		uint32_t shape = visualization_msgs::Marker::LINE_LIST;
		//visualization_msgs::Marker gripDir;
		// Set the frame ID and timestamp.  See the TF tutorials for information on these.
		gripDir.header.frame_id = "sensor_frame";
		gripDir.header.stamp = ros::Time::now();
		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		gripDir.ns = "rgSimple";
		gripDir.id = clusterNum;
		// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		gripDir.type = shape;
		// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		gripDir.action = visualization_msgs::Marker::ADD;
		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		//    	gripDir.pose.start.x = 0;
		//    	gripDir.pose.start.y = 0;
		//    	gripDir.pose.start.z = 0;
		//    	gripDir.pose.orientation.x = 0.0;
		//    	gripDir.pose.orientation.y = 0.0;
		//    	gripDir.pose.orientation.z = 0.0;
		//    	gripDir.pose.orientation.w = 1.0;
		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		gripDir.scale.x = 0.5;
		//gripDir.scale.y = 0.0001;
		//gripDir.scale.z = 0.0001;

		// Set the color -- be sure to set alpha to something non-zero!
		gripDir.color.r = 0.0f;
		gripDir.color.g = 1.0f;
		gripDir.color.b = 0.0f;
		gripDir.color.a = 0.1;
		gripDir.lifetime = ros::Duration(5);

		geometry_msgs::Point q,p;

		for (unsigned int index = 0; index< clusterPtr->points.size(); index = index + 5)
		{

			p.x =  clusterPtr->points[index].x;
			p.y =  clusterPtr->points[index].y;
			p.z =  clusterPtr->points[index].z;
			q.x =  clusterPtr->points[index].x + middle_vector(0);
			q.y =  clusterPtr->points[index].y + middle_vector(1);
			q.z =  clusterPtr->points[index].z + middle_vector(2);

			gripDir.points.push_back(p);
			gripDir.points.push_back(q);
		}

		vis_pub.publish(gripDir);
		gripDir.points.clear();
}


int
main (int argc, char** argv)
{



	 ros::init(argc, argv, "rgSimple");
	 ros::NodeHandle n;
	 typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;
	 ros::Publisher object_pub = n.advertise<PCLCloud> ("output/object", 1000);
	 ros::Publisher cluster_pub = n.advertise<PCLCloud> ("output/cluster", 1000);
	 ros::Publisher condensedCluster_pub = n.advertise<PCLCloud> ("output/condensedCluster", 1000);


	 ros::Rate loop_rate(10);
	 ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>("output/marker", 1000);


	 int count = 0;
	 while (ros::ok())
	 {

		PCLfinder table;	

		std::ifstream infile("/home/mayank/catkin_ws/src/grasp_segmentation/src/tablewon.xyz");
		double  x, y, z;

		if(infile)
		while (infile >> x >> y>> z)
		{
			table.objectPtr->points.push_back (pcl::PointXYZ(x/100,y/100,z/100));
			//std::cout<<x<<std::endl;
		    // process pair (a,b)
		}
		else
		std::cout<<"error opening file";

		table.objectPtr->width = (int) table.objectPtr->points.size ();
		table.objectPtr->height = 1;
		table.objectPtr->is_dense = true;
		std::cout<<*table.objectPtr;		

		table.findClusters();
		std::cout << "\nClusters after growing: " << table.clusters.size () << std::endl;

		for(int i=0; i< table.clusters.size(); i++)
		{
			table.createCluster(i);
			std::cout << "\n Patch size : " << table.clusterPtr->points.size () << std::endl;
			//std::cout<<table.clusterPtr->points.size();
			
			if(table.clusterPtr->points.size()!=0)
			{

			table.PCA();

			sensor_msgs::PointCloud2 cluster;
			pcl::toROSMsg(*table.clusterPtr,cluster);
			cluster.header.frame_id = "sensor_frame";
		   	cluster_pub.publish(cluster);
		   	publish_grip(i, table.clusterPtr, table.middle_vector, vis_pub);


		   	table.condenseCluster();
		   	std::cout << "Condensed size : " << table.condensed_clusterPtr->points.size () << std::endl;
		   	//cout<<"\nMean Curvature = "<<mean_curvature<<endl;
		   	cout<<"GripL = "<<table.grip(1)<<"\tGripR = "<<table.grip(2)<<std::endl;
		   	cout<<"Grip Width = "<<table.grip_width<<"\n Grip Axis = \n"<<table.middle_vector<<std::endl;

		   	sensor_msgs::PointCloud2 condensedCluster;
		   	pcl::toROSMsg(*table.condensed_clusterPtr,condensedCluster);
		   	condensedCluster.header.frame_id = "sensor_frame";

		   	getchar();

		   	if(table.grip_width>0.30 && table.grip_width<1.0)
		   	{
		   		condensedCluster_pub.publish(condensedCluster);
		   		publish_grip(i, table.condensed_clusterPtr, table.middle_vector, vis_pub);
		   	}

		   	else
		   		std::cout<<"Grip Disqualified"<<std::endl;


		   	ros::spinOnce();
	        loop_rate.sleep();
	        ++count;
			}
			getchar();

		
		}
				
		sensor_msgs::PointCloud2 object;
		pcl::toROSMsg(*table.object_coloredPtr,object);
		object.header.frame_id = "sensor_frame";
	   	object_pub.publish(object);

		ros::spinOnce();

	        loop_rate.sleep();
	        ++count;
	   }

}
