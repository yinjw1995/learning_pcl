#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h> 
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h> 

#include <pcl/common/time.h>   
#include <time.h> 
/****************************************************************************************************
**       读取->直通滤波->聚类平面及物体->分割平面->识别(或多个物体再聚类）->物体的重心             **
**   该程序：读取->直通滤波->聚类平面->分割平面->平面重心->取重心处的点云->(识别圆柱)->计算重心    **
*****************************************************************************************************/

typedef pcl::PointXYZ PointT;
int main()
{
	
	clock_t start, end;													//计算程序运行时间，ms
	
	//读取文件
	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);	//所需读取点云指针
	pcl::PCDReader reader;                    							//PCD文件读取对象
	
	reader.read ("table_scene_lms400.pcd", *cloud);
	std::cerr << "source loaded!\n" <<"PointCloud has: " 
				<< cloud->points.size () << "data points." << std::endl;
	start = clock();
	//滤波
	//这里先使用直通滤波，只选取特定x,y,z,上的点云数据
	pcl::PointCloud<PointT>::Ptr cloud_source_filtered(new pcl::PointCloud<PointT>);  
    pcl::PassThrough<PointT> pt;  
    pt.setInputCloud(cloud);  
    pt.setFilterFieldName("y");  
    pt.setFilterLimits(-0.3, 0.7);
    pt.filter(*cloud_source_filtered);
	*cloud=*cloud_source_filtered;
	//如果需要对其他轴上的限制，再进行设置。
	
	//这里使用去离群点滤波
	pcl::PointCloud<PointT>::Ptr cloud_filtered_source (new pcl::PointCloud<PointT>);
	pcl::VoxelGrid<PointT> sor;											//体素栅格下采样对象
	sor.setInputCloud (cloud);											//原始点云
	sor.setLeafSize (0.01f, 0.01f, 0.01f);								//设置采样体素大小
	sor.filter (*cloud_filtered_source);								//保存
	
	//聚类，把与平面连在一块的点聚到一起
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud_filtered_source);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;					//欧式聚类对象
	
	ec.setClusterTolerance (0.02);										// 设置近邻搜索的搜索半径为2cm
	ec.setMinClusterSize (2000);										//设置一个聚类需要的最少的点数目为2000
	ec.setMaxClusterSize (25000);										//设置一个聚类需要的最大点数目为25000
	ec.setSearchMethod (tree);											//设置点云的搜索机制
	ec.setInputCloud (cloud_filtered_source);							//输入需要聚类的点云指针地址
	ec.extract (cluster_indices);										//得到2000-25000点的聚类
	
	//迭代访问点云索引cluster_indices,直到分割出所有聚类
	pcl::PCDWriter writer;
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{	//循环容器中的点云的索引，并且分开保存索引的点云
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		//设置属性
		cloud_cluster->points.push_back (cloud_filtered_source->points[*pit]); 
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		std::stringstream ss;
		ss << "cloud_cluster_" << j << ".pcd";
		writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
		j++;
		//实际上只会存在一个，因为聚类的点数设置的较大，如果比较小就会有很多，这里是全部读出来  
		std::cout<<"count the number of cluster:"<<j<<"\n"<<std::endl;
	}

	//这里只取最大的聚类作为后面的识别，因为平面一般很大，cluster2，3是在调试聚集点较少的时候用到的，这里没删除
	pcl::PointCloud<PointT>::Ptr cluster1(new pcl::PointCloud<PointT>), cluster2(new pcl::PointCloud<PointT>), cluster3(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	reader.read("cloud_cluster_0.pcd", *cluster1);
	//reader.read("cloud_cluster_1.pcd", *cluster2);
	//reader.read("cloud_cluster_2.pcd", *cluster3);
	*cloud_filtered=*cluster1;
	std::cout << "filted point cloud and cluster succeed!"<< std::endl;
	//如果只是想取出连接在平面上的点，这需要设置很大的聚类点，然后取第一个就可以，也不需要for循环，后续可以进行优化
	
	//设置分割类型
	//这里使用ransac分割平面	
	pcl::SACSegmentation<PointT> seg;									//创建分割对象
	seg.setOptimizeCoefficients (true);									//设置对估计模型参数进行优化处理
	seg.setModelType (pcl::SACMODEL_PLANE);								//设置分割模型类别
	//seg.setNormalDistanceWeight (0.1);
	//seg.setInputNormals (cloud_normals);								//要是使用法线估计pcl::SACMODEL_NORMAL_PLANE就要用上面两句话 normal
	seg.setMethodType (pcl::SAC_RANSAC);								//设置用哪个随机参数估计方法
	seg.setMaxIterations (100);											//设置最大迭代次数
	seg.setDistanceThreshold (0.01);									//判断是否为模型内点的距离阀值
	seg.setInputCloud (cloud_filtered);
	
	//分割平面，并提取数据
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::ExtractIndices<PointT> extract;								//创建点云提取对象
	//pcl::ExtractIndices<pcl::Normal> extract_normals;					//创建法线提取对象
	pcl::PointCloud<PointT>::Ptr cloud_other(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>);
		
	seg.segment (*inliers, *coefficients);								//分割
	std::cerr << "Plane coefficients: " << *coefficients << std::endl;
	extract.setInputCloud (cloud_filtered);								//设置输入点云数据
	extract.setIndices (inliers);										//设置指定点云数据
	extract.setNegative(true);											//指定点外的点云，即除平面外的物体
	extract.filter(*cloud_other);										//输出其他点，平面外
	extract.setNegative(false);
	extract.filter(*cloud_plane);										//输出指定点，平面
	std::cerr << "segment cloud_plane succeed" << std::endl;

	//提取桌上平面的坐标
	Eigen::Vector4f centroid_plane;										//定义向量
	pcl::compute3DCentroid(*cloud_plane, centroid_plane);				//计算点云重心
	std::cerr << "The XYZ coordinates of the planes centroid are: ("	//显示质心坐标
				<< centroid_plane[0] << ", "
				<< centroid_plane[1] << ", "
				<< centroid_plane[2] << ")." << std::endl;
	pcl::PointXYZ minPt, maxPt;											//定义点云类型
	pcl::getMinMax3D(*cloud_plane, minPt, maxPt);						//计算点云的XYZ极值坐标
	std::cerr << "plane_minpt:" << minPt << "\n"						//打印
		<< "plane_maxpt:" << maxPt << "\n" << endl;

	//对平面重心处的点云进行搜集，目的是缩小识别范围
	pt.setInputCloud(cloud_other);
	pt.setFilterFieldName("x"); pt.setFilterLimitsNegative(false);
	pt.setFilterLimits(centroid_plane[0]- 0.1, centroid_plane[0]+0.1);
	pt.filter(*cloud_source_filtered); *cloud_other = *cloud_source_filtered;
	
	pt.setInputCloud(cloud_other);
	pt.setFilterFieldName("y"); pt.setFilterLimitsNegative(false);
	pt.setFilterLimits(centroid_plane[1] - 0.15, centroid_plane[1] + 0.15);
	pt.filter(*cloud_source_filtered);*cloud_other = *cloud_source_filtered; 
	
	pt.setInputCloud(cloud_other);
	pt.setFilterFieldName("z"); pt.setFilterLimitsNegative(false);
	pt.setFilterLimits(centroid_plane[2] - 0.1, centroid_plane[2] + 0.1);
	pt.filter(*cloud_source_filtered); *cloud_other = *cloud_source_filtered;

	//分割圆柱
	//计算法线normal
	pcl::NormalEstimation<PointT, pcl::Normal> ne;						//法线估计对象
	pcl::search::KdTree<PointT>::Ptr tree1(new pcl::search::KdTree<PointT>);//搜索器：二叉树KdTree
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_other(new pcl::PointCloud<pcl::Normal>);

	ne.setSearchMethod(tree1);											//输入搜索器
	ne.setInputCloud(cloud_other);
	ne.setKSearch(50);													//迭代次数
	ne.compute(*cloud_normals_other);									// 过滤后的点云进行法线估计，为后续进行基于法线的分割准备数据
	std::cout << "compute the cloud_normal succeed!" << std::endl;

	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg2;
	pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT>);

	seg2.setOptimizeCoefficients (true);								//设置对估计模型优化
	seg2.setModelType (pcl::SACMODEL_CYLINDER);							//设置分割模型为圆柱形
	seg2.setMethodType (pcl::SAC_RANSAC);								//参数估计方法
	seg2.setNormalDistanceWeight (0.1);									//设置表面法线权重系数
	seg2.setMaxIterations (1000);										//设置迭代的最大次数10000
	seg2.setDistanceThreshold (0.05);									//设置内点到模型的距离允许最大值
	seg2.setRadiusLimits (0, 0.1);										//设置估计出的圆柱模型的半径的范围
	seg2.setInputCloud (cloud_other);
	seg2.setInputNormals (cloud_normals_other);
	
	seg2.segment (*inliers, *coefficients);								//分割*inliers_cylinder, *coefficients_cylinder
	std::cerr << "cylinder coefficients: " << *coefficients << std::endl;
	extract.setInputCloud (cloud_other);								//设置输入点云数据
	extract.setIndices (inliers);										//设置指定点云数据
	extract.setNegative (false);
	extract.filter (*cloud_cylinder);									//滤出分割数据，圆柱
	std::cout << "segment cloud_cylinder succeed" << std::endl;

	pcl::getMinMax3D(*cloud_cylinder, minPt, maxPt);
	std::cerr << "cylinder_minpt:" << minPt << "\n"
	  		  << "cylinder_maxpt:" << maxPt << "\n" << endl;

	//计算圆柱质心坐标,由于cylinder的数据不知道实际的大小是多少，所以不好使用ransac识别出来。这里直接提取
	Eigen::Vector4f centroid_cylinder;
	pcl::compute3DCentroid(*cloud_cylinder, centroid_cylinder);			//计算点云重心
	pcl::PointCloud<PointT>::Ptr add_ptr(new pcl::PointCloud<PointT>);	//定义所需显示的点云类型指针
	std::cerr << "The XYZ coordinates of the cylinder centroid are: ("
		<< centroid_cylinder<< ")." << std::endl;
	PointT point_add = { centroid_cylinder[0],centroid_cylinder[1],centroid_cylinder[2] };
	add_ptr->push_back(point_add);										//将点云坐标添加到点云类型的最后一行

	end = clock();														//计时结束
	std::cout << "run time:" << (double)(end-start)/*/CLOCKS_PER_SEC*/ << "ms" << std::endl;
	
	int a(0);
	pcl::visualization::PCLVisualizer viewer1;							//定义可视化窗口
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(add_ptr, 255, 0, 0);//设置点云显示的颜色，rgb
	viewer1.addPointCloud(cloud_filtered_source, "origin_cloud");		//添加所需显示的点云
	viewer1.addPointCloud(add_ptr, red, "cylinder_centroid");			//添加所需显示的点云，附加颜色
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cylinder_centroid");//设置特定ID点云显示的尺寸
	viewer1.setWindowName("origin_cloud;");								//窗口名称
	viewer1.spinOnce();													//运行一次

	pcl::visualization::PCLVisualizer viewer2,viewer3,viewer4;
	viewer2.setWindowName("plane;");
	viewer2.addPointCloud(cloud_plane, "cloud_plane");
	viewer2.spinOnce();

	viewer3.setWindowName("others;");
	viewer3.addPointCloud(cloud_other, "cloud_other");
	viewer3.spinOnce();
	
	viewer4.setWindowName("cylinder;");
	pcl::visualization::PointCloudColorHandlerCustom<PointT>  cylinder_color(cloud_cylinder, 255, 0, 0);
	viewer4.addPointCloud(cloud_cylinder,cylinder_color, "cloud_cylinder");
	/*
	pcl::visualization::PointCloudColorHandlerCustom<PointT>  cluster_color1(cluster1, 255, 0, 0);
	viewer4.addPointCloud(cluster1,cluster_color1, "cluster1");
	pcl::visualization::PointCloudColorHandlerCustom<PointT>  cluster_color2(cluster2, 0, 155, 0);
	viewer4.addPointCloud(cluster2, cluster_color2, "cluster2");
	pcl::visualization::PointCloudColorHandlerCustom<PointT>  cluster_color3(cluster3, 0, 0, 255);
	viewer4.addPointCloud(cluster3, cluster_color3, "cluster3");*/
	viewer4.spin();
	std::getchar();

	return (0);	
}