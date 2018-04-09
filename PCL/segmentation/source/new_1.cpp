#include <pcl/ModelCoefficients.h>  
#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h>  
#include <pcl/filters/extract_indices.h>  
#include <pcl/filters/voxel_grid.h>  
#include <pcl/features/normal_3d.h>  
#include <pcl/kdtree/kdtree.h>  
#include <pcl/sample_consensus/method_types.h>  
#include <pcl/sample_consensus/model_types.h>  
#include <pcl/segmentation/sac_segmentation.h>  
#include <pcl/segmentation/extract_clusters.h>  
  
#include <pcl/visualization/pcl_visualizer.h>       
#include <pcl/common/time.h>   
#include <time.h>  
  
int   
main (int argc, char** argv)  
{  
    clock_t start, end;  
  // Read in the cloud data读数据  
  pcl::PCDReader reader;  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);  
  reader.read ("table_scene_lms400.pcd", *cloud);  
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*  
  
  // Create the filtering object: downsample the dataset using a leaf size of 1cm体素栅格滤波  
  pcl::VoxelGrid<pcl::PointXYZ> vg;  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);  
  vg.setInputCloud (cloud);  
  vg.setLeafSize (0.01f, 0.01f, 0.01f);  
  vg.filter (*cloud_filtered);  
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*  
  
  // Create the segmentation object for the planar model and set all the parameters随机采样一致性分割  
  pcl::SACSegmentation<pcl::PointXYZ> seg;  
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);  
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());  
  pcl::PCDWriter writer;  
  seg.setOptimizeCoefficients (true);  
  seg.setModelType (pcl::SACMODEL_PLANE);  
  seg.setMethodType (pcl::SAC_RANSAC);  
  seg.setMaxIterations (100);  
  seg.setDistanceThreshold (0.02);  
  
  int i=0, nr_points = (int) cloud_filtered->points.size ();  
  while (cloud_filtered->points.size () > 0.3 * nr_points)  
  {  
    // Segment the largest planar component from the remaining cloud从剩余点云中分割出最大平面  
    seg.setInputCloud (cloud_filtered);  
    seg.segment (*inliers, *coefficients);  
    if (inliers->indices.size () == 0)  
    {  
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;  
      break;  
    }  
  
    // Extract the planar inliers from the input cloud从输入点云（滤波后的点云）中提取平面内点  
    pcl::ExtractIndices<pcl::PointXYZ> extract;  
    extract.setInputCloud (cloud_filtered);  
    extract.setIndices (inliers);  
    extract.setNegative (false);  
  
    // Write the planar inliers to disk存储平面点云  
    extract.filter (*cloud_plane);  
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;  
  
    // Remove the planar inliers, extract the rest移除内点，提取剩下的点云  
    extract.setNegative (true);  
    extract.filter (*cloud_f);  
    cloud_filtered = cloud_f;  
  }  
  
  //////////////////////////////////////////////////////////   
  // Creating the KdTree object for the search method of the extraction为提取算法创建KdTree  
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);  
  tree->setInputCloud (cloud_filtered);  
  //////////////////////////////////////////////////////////   
  
  std::vector<pcl::PointIndices> cluster_indices;  
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;//欧式聚类  
  ec.setClusterTolerance (0.02); // 2cm  
  ec.setMinClusterSize (100);  
  ec.setMaxClusterSize (25000);  
  ec.setSearchMethod (tree);  
  ec.setInputCloud (cloud_filtered);  
  
  start = clock();  
  ec.extract (cluster_indices);  
  end = clock();  
  
  //////////////////////////////////////////////////////////   
  
  int j = 0;  
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)  
  {  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);  
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)  
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*  
    cloud_cluster->width = cloud_cluster->points.size ();  
    cloud_cluster->height = 1;  
    cloud_cluster->is_dense = true;  
  
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;  
    std::stringstream ss;  
    ss << "cloud_cluster_" << j << ".pcd";  
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*  
    j++;  
  }  
  //////////////////////////////////////////////////////////    
  //得到欧式聚类时间    
  std::cout << "欧式聚类运行时间" << std::endl;  
  std::cout << "运行时间:" << (double)(end-start)/*/CLOCKS_PER_SEC*/ << "毫妙" << std::endl;  
    
  //////////////////////////////////////////////////////////    
  pcl::visualization::PCLVisualizer viewer;  
  int v1(0);  
  viewer.createViewPort(0.0, 0.0, 0.25, 1.0, v1);  
  viewer.setBackgroundColor(1.0, 0.5, 1.0, v1);  
  viewer.addText("Remove the planar inliers, extract the rest ", 10, 10, "cloud_f", v1);  
  viewer.addPointCloud(cloud_f, "cloud_f", v1);  
  
  int v2(0);  
  viewer.createViewPort(0.25, 0.0, 0.5, 1.0, v2);  
  viewer.setBackgroundColor(1.0, 0.5, 1.0, v2);  
  viewer.addText("cloud_plane", 10, 10, "cloud_plane", v2);  
  viewer.addPointCloud(cloud_plane, "cloud_plane", v2);  
  
  int v3(0);  
  viewer.createViewPort(0.5, 0.0, 0.75, 1.0, v3);  
  viewer.setBackgroundColor(1.0, 0.5, 1.0, v3);  
  viewer.addText("cloud", 10, 10, "cloud", v3);  
  viewer.addPointCloud(cloud, "cloud", v3);  
  
  int v4(0);  
  viewer.createViewPort(0.75, 0.0, 1.0, 1.0, v4);  
  viewer.setBackgroundColor(1.0, 0.5, 1.0, v4);  
  viewer.addText("cloud_filtered", 10, 10, "cloud_filtered", v4);  
  viewer.addPointCloud(cloud_filtered, "cloud_filtered", v4);  
  //////////////////////////////////////////////////////////  
  viewer.spin();  
  
  //////////////////////////////////////////////////////////    
  return (0);  
}  