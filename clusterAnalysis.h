/* \Mayank Roy */
//Standard libraries
#pragma once
#include <stdlib.h>   
#include <pcl/common/common_headers.h>
#include <stdio.h>
//#include <tchar.h>
//#include<conio.h>
//#include<io.h>
#include <iostream>
#include <math.h>  
#include<string.h>
//#include<highgui.h>
//#include<cv.h>
#include <vector>
#include <list>
#include <map>
#include <deque>
#include <sstream>
#include <fstream>
//#include <ostream>
#include <istream>
#include <iomanip>
#include <algorithm>
#include <time.h>
 
 
//PCL libraries
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
 

// downSampling
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>



// filtering
#include <pcl/filters/statistical_outlier_removal.h>

//Voxelize
#include <pcl/filters/passthrough.h>
 
//regionGrowing
#include <vector>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
 
//PCE principal curvature estimation
#include <pcl/features/principal_curvatures.h>
 
//Moment of Inertia estimation
#include <pcl/features/moment_of_inertia_estimation.h>
 
 
 
class PCLfinder{    
 
public:
 
    pcl::PointCloud<pcl::PointXYZ>::Ptr objectPtr;
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr object_coloredPtr;
    pcl::PointCloud<pcl::Normal>::Ptr object_normalsPtr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterPtr ;
    pcl::PointCloud<pcl::Normal>::Ptr cluster_normalsPtr;
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cluster_pcPtr ;
    pcl::PointCloud<pcl::PointXYZ>::Ptr condensed_clusterPtr ;
    pcl::PointCloud<pcl::Normal>::Ptr gripAxisPtr;
    std::vector <pcl::PointIndices> clusters;
    pcl::PointCloud<pcl::PointXYZ>::Ptr projected_clusterPtr ;
    pcl::IndicesPtr object_indices;
    pcl::IndicesPtr cluster_indices;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;
    Eigen::Vector3f grip;
    Eigen::Vector3f centroid;
    long double mean_curvature, grip_width;
    Eigen::Vector3f length;
    Eigen::Vector3f breadth;
    Eigen::Vector3f height;
    Eigen::Matrix3f Orientation;
    float X,Y,Z,A,B,C;
 
 
    //pcl::PointXYZ center;
    PCLfinder()
    {                                      
        //Initializing Parameters
        objectPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
        object_coloredPtr = pcl::PointCloud <pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
        object_normalsPtr = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
        clusterPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);  
        cluster_normalsPtr = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
        cluster_pcPtr = pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr (new pcl::PointCloud<pcl::PrincipalCurvatures>);
        condensed_clusterPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);  
        gripAxisPtr = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
        object_indices = pcl::IndicesPtr (new std::vector <int>);
        cluster_indices = pcl::IndicesPtr (new std::vector <int>);
        mean_curvature = 0.0;
        grip_width = 0.0;
        X=0;
        Y=0;
        Z=0;
        A=0;
        B=0;
        C=0;
        
    }

 
	void findClusters()
	{
    
		// Finding highest point
   		// Create the normal estimation class, and pass the input dataset to it
 		

    		//pcl::PointCloud<pcl::PointXYZ>::Ptr object (new pcl::PointCloud<pcl::PointXYZ>);
    		//object = objectPtr;
      		 pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      		//pcl::PointCloud<pcl::Normal>::Ptr object_normals (new pcl::PointCloud<pcl::Normal>);
		// Estimation of normals
		ne.setSearchMethod (tree);
      	ne.setInputCloud (objectPtr);
		// Use all neighbors in a sphere of radius 3cm
  		//ne.setRadiusSearch (0.0010);
  		//ne.setInputCloud (cloud);
  		ne.setKSearch (50);
  		ne.compute (*object_normalsPtr);
      		// Create an empty kdtree representation, and pass it to the normal estimation object.
      		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  		//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  		//ne.setSearchMethod (tree);
  		//cout<<"kd tree defined\n";
  		// Output datasets


  		//cout<<"search radius defined\n";
		cout<<"done computing normals\n";
		cout<<" growing regions \n";
		//Region growing parameters based on - min and max points and curvature info
		pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
		reg.setMinClusterSize (50);
		reg.setMaxClusterSize (20000000);
		reg.setSearchMethod (tree);
		reg.setNumberOfNeighbours (30);
		reg.setInputCloud (objectPtr);
		//  reg.setIndices (indices);
		reg.setInputNormals (object_normalsPtr);
		reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);//11
		reg.setCurvatureThreshold (1.0);//0.03
		// std::vector <pcl::PointIndices> clusters;
		reg.extract (clusters);
		
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_colored (new pcl::PointCloud<pcl::PointXYZRGB>);
		object_coloredPtr = reg.getColoredCloud ();
		//std::cout << "\nClusters after growing: " << clusters.size () << std::endl;
		//std::cout << "Cloud after growing: " << std::endl;
		//std::cout << *object_colored << std::endl;
		int d;
		    
		//cluster Info
		 
		//std::cout <<"done growing regions"<< std::endl;
		 
		//object_normalsPtr = object_normals;
		//object_coloredPtr = object_colored;
		 
		   
	}


	void createCluster(int cluster_num)
	{
		//pcl::IndicesPtr cluster_indices (new std::vector <int>);
		//pcl::PointCloud<pcl::PointXYZ>::Ptr object (new pcl::PointCloud<pcl::PointXYZ>);
		//object = objectPtr;
		//Chose cluster
		//int cluster_num = 0;
		*cluster_indices = clusters[cluster_num].indices;
		cout<<"done transferring indices\n";
		pcl::PointXYZ cluster_insert;  
		pcl::Normal normal_insert;
		//cluster extraction for individual processing
		pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);  
		pcl::PointCloud<pcl::Normal>::Ptr cluster_normals (new pcl::PointCloud<pcl::Normal>);
		int counter = 0;
		while (counter < clusters[cluster_num].indices.size ())
		{
		    cluster_insert.x = objectPtr->points[clusters[cluster_num].indices[counter]].x;
		    cluster_insert.y = objectPtr->points[clusters[cluster_num].indices[counter]].y;  
		    cluster_insert.z = objectPtr->points[clusters[cluster_num].indices[counter]].z;
		    cluster->points.push_back (cluster_insert);
		
		    normal_insert = object_normalsPtr->points[clusters[cluster_num].indices[counter]];
		    normal_insert.curvature = object_normalsPtr->points[clusters[cluster_num].indices[counter]].curvature;
		    cluster_normals->points.push_back (normal_insert);
		 
		    counter++;
		 
		}

		clusterPtr = cluster;
		cluster_normalsPtr = cluster_normals;

		clusterPtr->width = (int) clusterPtr->points.size ();
		clusterPtr->height = 1;
		clusterPtr->is_dense = true;
		//std::cout<<*table.objectPtr;
		//std::cout << "\n Patch size : " << clusterPtr->points.size () << std::endl;
		cout<<"Done defining cluster\n";
	}


	void PCA()
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);  
		cluster = clusterPtr;
		//PCA and Bounding Box calculations
		pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
		feature_extractor.setInputCloud (cluster);
		feature_extractor.compute ();
		 
		std::vector <float> moment_of_inertia;
		std::vector <float> eccentricity;
		pcl::PointXYZ min_point_AABB;
		pcl::PointXYZ max_point_AABB;
		pcl::PointXYZ min_point_OBB;
		pcl::PointXYZ max_point_OBB;
		pcl::PointXYZ position_OBB;
		Eigen::Matrix3f rotational_matrix_OBB;
		float major_value, middle_value, minor_value;
		 
		feature_extractor.getMomentOfInertia (moment_of_inertia);
		feature_extractor.getEccentricity (eccentricity);
		feature_extractor.getAABB (min_point_AABB, max_point_AABB);
		feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
		feature_extractor.getEigenValues (major_value, middle_value, minor_value);
		feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
		feature_extractor.getMassCenter (mass_center);
		
		//std::cout<<mass_center<<"\n\n"<<major_vector<<"\n\n"<<middle_vector<<"\n\n"<<minor_vector<<endl;
		Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
		Eigen::Quaternionf quat (rotational_matrix_OBB);
		/* viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");*/
		 
		pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
		pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
		pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
		pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
		 
		//Eigen::Vector3f p1 (min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
		//Eigen::Vector3f p2 (min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
		//Eigen::Vector3f p3 (max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
		//Eigen::Vector3f p4 (max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
		//Eigen::Vector3f p5 (min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
		//Eigen::Vector3f p6 (min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
		//Eigen::Vector3f p7 (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
		//Eigen::Vector3f p8 (max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
		
		//p1 = rotational_matrix_OBB * p1 + position;
		//p2 = rotational_matrix_OBB * p2 + position;
		//p3 = rotational_matrix_OBB * p3 + position;
		//p4 = rotational_matrix_OBB * p4 + position;
		//p5 = rotational_matrix_OBB * p5 + position;
		//p6 = rotational_matrix_OBB * p6 + position;
		//p7 = rotational_matrix_OBB * p7 + position;
		//p8 = rotational_matrix_OBB * p8 + position;
		
		//pcl::PointXYZ pt1 (p1 (0), p1 (1), p1 (2));
		//pcl::PointXYZ pt2 (p2 (0), p2 (1), p2 (2));
		//pcl::PointXYZ pt3 (p3 (0), p3 (1), p3 (2));
		//pcl::PointXYZ pt4 (p4 (0), p4 (1), p4 (2));
		//pcl::PointXYZ pt5 (p5 (0), p5 (1), p5 (2));
		//pcl::PointXYZ pt6 (p6 (0), p6 (1), p6 (2));
		//pcl::PointXYZ pt7 (p7 (0), p7 (1), p7 (2));
		//pcl::PointXYZ pt8 (p8 (0), p8 (1), p8 (2));
		 
		 //min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z
		length(0) = max_point_OBB.x;
		length(1) = min_point_OBB.x;
		   
		//cout<<"Length :"<<length<<endl;
		breadth(0) = max_point_OBB.y;
		breadth(1) = min_point_OBB.y;
		  
		//cout<<"Breadth :"<<breadth<<endl;
		height(0) = max_point_OBB.z;
		height(1) = min_point_OBB.z;
		   
		   
		 
	}
	
	void analyseCluster()  
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);
		cluster = clusterPtr;
		//pcl::PointCloud<pcl::Normal>::Ptr cluster_normals (new pcl::PointCloud<pcl::Normal>);  
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> pne;
		// Estimation of normals for cluster
		pne.setInputCloud (cluster);
		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZ>::Ptr cluster_tree (new pcl::search::KdTree<pcl::PointXYZ> ());
		pne.setSearchMethod (cluster_tree);
		//cout<<"kd tree defined\n";
		// Use all neighbors in a sphere of radius 3cm
		pne.setRadiusSearch (0.005);
		cout<<"search radius defined\n";
		// Compute the features
		pcl::PointCloud<pcl::Normal>::Ptr cluster_normals (new pcl::PointCloud<pcl::Normal>);
		pne.compute (*cluster_normals);
		cout<<"done computing normals\n";
		
		cout<<"Computing Curvature\n";
		pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pci;
		// Setup the principal curvatures computation
		
		// Provide the original point cloud (without normals)
		pci.setInputCloud (cluster);
		cout<<"Setting input parameters\n";
		// Provide the point cloud with normals
		pci.setInputNormals(cluster_normals);
	
		//Set cluster indices
		//pci.setIndices(cluster_indices);
		 
		//pcl::search::KdTree<pcl::PointXYZ>::Ptr cluster_tree (new pcl::search::KdTree<pcl::PointXYZ> ());
		// Use the same KdTree from the cluster normal estimation
		pci.setSearchMethod (cluster_tree);
		pci.setRadiusSearch(0.007);
		cout<<"setting search radius\n";
		// Actually compute the principal curvatures
		pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cluster_pc (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
		pci.compute (*cluster_pc);
		cluster_pcPtr = cluster_pc;
		std::cout << "Patch size : " << cluster->points.size () << std::endl;
		
		mean_curvature = 0.0;
		
		pcl::PrincipalCurvatures descriptor ;
		int stray =0 ;
		for (size_t i = 0; i < cluster_pc->points.size (); ++i)
		{
		      descriptor = cluster_pc->points[i];
		      if(abs(descriptor.pc1)>3||abs(descriptor.pc1)<0)
		      {
			  cout<<"stary Point"<<endl;stray++;
		      }
		      else
		      {mean_curvature = mean_curvature +  abs(descriptor.pc1);}
		}
		mean_curvature = mean_curvature/(cluster_pc->points.size ()-stray);
		 
		std::cout <<"mean curvature value is"<< mean_curvature << std::endl;
		 
		cluster_normalsPtr = cluster_normals;
		cluster_pcPtr = cluster_pc;
		condensed_clusterPtr = cluster;
	
	}


	void condenseCluster()
	{

		pcl::PointXYZ cluster_insert;
		pcl::Normal normal_insert;
		grip << 0, 0, 0;
		mean_curvature =0;
		grip_width = 0;

		pcl::PointCloud<pcl::PointXYZ>::Ptr condensed_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::Normal>::Ptr gripAxis (new pcl::PointCloud<pcl::Normal>);

		for (unsigned int index = 0; index< clusterPtr->points.size(); index++)
		{

			cluster_insert.x =  clusterPtr->points[index].x;
			cluster_insert.y =  clusterPtr->points[index].y;
			cluster_insert.z =  clusterPtr->points[index].z;
			normal_insert.normal_x =  middle_vector(0);
			normal_insert.normal_y =  middle_vector(1);
			normal_insert.normal_z =  middle_vector(2);

			grip(0) = (cluster_insert.x * normal_insert.normal_x +
								cluster_insert.y * 	normal_insert.normal_y +
									cluster_insert.z *	normal_insert.normal_z) -
											(mass_center(0) * normal_insert.normal_x +
												mass_center(1) * 	normal_insert.normal_y +
													mass_center(2) *	normal_insert.normal_z);
			if(grip(0)<grip(1))
				grip(1) = grip(0);
			else if(grip(0)>grip(2))
				grip(2) = grip(0);

			mean_curvature = mean_curvature + cluster_normalsPtr->points[index].curvature;
//			std::cout<< grip(0)<<"\t"<<cluster_insert<<"\n";

			cluster_insert.x = cluster_insert.x - grip(0)*normal_insert.normal_x;
			cluster_insert.y = cluster_insert.y - grip(0)*normal_insert.normal_y;
			cluster_insert.z = cluster_insert.z - grip(0)*normal_insert.normal_z;
			normal_insert.curvature =  grip(0);


			//cout<< normal_insert<<"\t"<<cluster_insert<<endl;


			condensed_cluster->points.push_back(cluster_insert);
			gripAxis->points.push_back(normal_insert);
		}

		mean_curvature = mean_curvature/(int) condensed_clusterPtr->points.size ();
		grip_width = fabs(double(grip(2) - grip(1)));

		condensed_clusterPtr = condensed_cluster;
		gripAxisPtr = gripAxis;

		//cout<<"\nMean Curvature = "<<mean_curvature<<endl;
		//cout<<"\t Grip Width = "<<grip_width<<"\n Grip Axis = "<<middle_vector<<std::endl;

		condensed_clusterPtr->width = (int) condensed_clusterPtr->points.size ();
		condensed_clusterPtr->height = 1;
		condensed_clusterPtr->is_dense = true;
		//std::cout<<*table.objectPtr;
		//std::cout << "Condensed size : " << condensed_clusterPtr->points.size () << std::endl;
		cout<<"Done condensing cluster\n";

	}

	
		 
};

