## segmentation 分割

- 分割在一般的教程和点云PCL学习教程上写的很简单，都没有说到重点并且详细的讲解，因此，我参考官网上的英文教程进行学习，下面是一些简单的原理介绍和代码分析说明，初学存在很多不足。

---

### RanSaC算法

​    点云分割的目的提取点云中的不同物体，从而实现分而治之，突出重点，单独处理的目的。而在现实点云数据中，往往对场景中的物体有一定先验知识。比如：桌面墙面多半是大平面，桌上的罐子应该是圆柱体，长方体的盒子可能是牛奶盒......对于复杂场景中的物体，其几何外形可以归结于简单的几何形状。这为分割带来了巨大的便利，因为简单几何形状是可以用方程来描述的，或者说，可以用有限的参数来描述复杂的物体。而方程则代表的物体的拓扑抽象。于是，RanSaC算法可以很好的将此类物体分割出来。

RanSaC算法（随机采样一致）原本是用于数据处理的一种经典算法，其作用是在大量噪声情况下，提取物体中特定的成分。下图是对RanSaC算法效果的说明。图中有一些点显然是满足某条直线的，另外有一团点是纯噪声。目的是在大量噪声的情况下找到直线方程，此时噪声数据量是直线的3倍。

![img](https://img-blog.csdn.net/20170929102247363?watermark/2/text/aHR0cDovL2Jsb2cuY3Nkbi5uZXQvQXBwXzEyMDYyMDEx/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70/gravity/Center)

　　如果用最小二乘法是无法得到这样的效果的，直线大约会在图中直线偏上一点。关于随机采样一致性算法的原理，在wiki百科上讲的很清楚，甚至给出了伪代码和matlab,C代码。见网址https://en.wikipedia.org/wiki/RANSAC. 我想换一个不那么严肃或者说不那么学术的方式来解释这个算法。

　　实际上这个算法就是从一堆数据里挑出自己最心仪的数据。所谓心仪当然是有个标准（目标的形式:满足直线方程？满足圆方程？以及能容忍的误差e）。平面中确定一条直线需要2点，确定一个圆则需要3点。随机采样算法，其实就和小女生找男朋友差不多。

1. 从人群中随便找个男生，看看他条件怎么样，然后和他谈恋爱，（平面中随机找两个点，拟合一条直线，并计算在容忍误差e中有多少点满足这条直线）
2. 第二天，再重新找个男生，看看他条件怎么样，和男朋友比比，如果更好就换新的（重新随机选两点，拟合直线，看看这条直线是不是能容忍更多的点，如果是则记此直线为结果）
3. 第三天，重复第二天的行为（循环迭代）
4. 终于到了某个年龄，和现在的男朋友结婚（迭代结束，记录当前结果）

　　显然，如果一个女生按照上面的方法找男朋友，最后一定会嫁一个好的（我们会得到心仪的分割结果）。只要这个模型在直观上存在，该算法就一定有机会把它找到。优点是噪声可以分布的任意广，噪声可以远大于模型信息。

　　这个算法有两个缺点，第一，必须先指定一个合适的容忍误差e。第二，必须指定迭代次数作为收敛条件。

　　综合以上特性，**本算法非常适合从杂乱点云中检测某些具有特殊外形的物体。**（作者这个例子，个人感觉有不太合适，有兴趣的可以查查具体的算法原理）

PCL支持了大量几何模型的RanSaC检测，可以非常方便的对点云进行分割。其调用方法如下：

```
//创建一个模型参数对象，用于记录结果
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  //inliers表示误差能容忍的点 记录的是点云的序号
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // 创建一个分割器
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory-设置目标几何形状
  seg.setModelType (pcl::SACMODEL_PLANE);
  //分割方法：随机采样法
  seg.setMethodType (pcl::SAC_RANSAC);
  //设置误差容忍范围
  seg.setDistanceThreshold (0.01);
  //输入点云
  seg.setInputCloud (cloud);
  //分割点云
  seg.segment (*inliers, *coefficients);
```

除了平面以外，PCL几乎支持所有的几何形状。作为点云分割的基础算法，RanSaC很强大且必收敛，可以作为机器人抓取，识别等后续任务的前处理。

分割给人最直观的影响大概就是邻居和我不一样。比如某条界线这边是中华文明，界线那边是西方文，最简单的分割方式就是在边界上找些居民问:"小伙子，你到底能不能上油管啊？”。然后把能上油管的居民坐标连成一条线，自然就区分开了两个地区。也就是说，除了之前提到的基于采样一致的分割方式以外，应该还存在基于邻近搜索的分割方式。通过对比某点和其最近一点的某些特征，来实现点云的分割。图像所能提供的分割信息仅是灰度或RGB向量，而三维点云却能够提供更多的信息。故点云在分割上的优势是图像所无法比拟的（重要的事情要说三遍）。

下面就是官网上以及书上都使用到的程序例子：

- [**简单的平面的分割**](http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation)

planar_segmentation.cpp

```c++
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>   //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>   //模型定义头文件
#include <pcl/segmentation/sac_segmentation.h>   //基于采样一致性分割的类的头文件

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // 填充点云
  cloud->width  = 15;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  // 生成数据，采用随机数填充点云的x,y坐标，都处于z为1的平面上
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1.0;
  }

  // 设置几个局外点，即重新设置几个点的z值，使其偏离z为1的平面
  cloud->points[0].z = 2.0;
  cloud->points[3].z = -2.0;
  cloud->points[6].z = 4.0;

  std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;  //打印
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " "
                        << cloud->points[i].y << " "
                        << cloud->points[i].z << std::endl;
  //创建分割时所需要的模型系数对象，coefficients及存储内点的点索引集合对象inliers
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // 创建分割对象
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // 可选择配置，设置模型系数需要优化
  seg.setOptimizeCoefficients (true);
  // 必要的配置，设置分割的模型类型，所用的随机参数估计方法，距离阀值，输入点云
  seg.setModelType (pcl::SACMODEL_PLANE);   //设置模型类型
  seg.setMethodType (pcl::SAC_RANSAC);      //设置随机采样一致性方法类型
  seg.setDistanceThreshold (0.01);    //设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件
                                       //表示点到估计模型的距离最大值，

  seg.setInputCloud (cloud);
  //引发分割实现，存储分割结果到点几何inliers及存储平面模型的系数coefficients
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }
  //打印出平面模型
  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (size_t i = 0; i < inliers->indices.size (); ++i)
    std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                               << cloud->points[inliers->indices[i]].y << " "
                                               << cloud->points[inliers->indices[i]].z << std::endl;

  return (0);
}
```

- [实现圆柱体模型的分割](http://pointclouds.org/documentation/tutorials/cylinder_segmentation.php#cylinder-segmentation)

(为了方便显示更改了程序，个人认为这个方法分割效率太低了，运行速度很慢)

cylinder_segmentation.cpp

```
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZ PointT;

int
main (int argc, char** argv)
{
  // All the objects needed
  pcl::PCDReader reader;                    				//PCD文件读取对象
  pcl::PassThrough<PointT> pass;             				//直通滤波对象
  pcl::NormalEstimation<PointT, pcl::Normal> ne; 			//法线估计对象
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; //分割对象
  pcl::PCDWriter writer;            						//PCD文件写入对象
  pcl::ExtractIndices<PointT> extract;      				//点提取对象，放xyz提取数据
  pcl::ExtractIndices<pcl::Normal> extract_normals;    		//点提取对象，放法线提取数据
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);						//点云
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);			//滤波后的点云
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);	//全部点云的法线数据
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);			//滤出的物体点云数据
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);	//滤出的物体法线数据
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

  // Read in the cloud data
  reader.read ("table_scene_mug_stereo_textured.pcd", *cloud);
  std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

  // 直通滤波，将Z轴不在（0，1.5）范围的点过滤掉，将剩余的点存储到cloud_filtered对象中
  pass.setInputCloud (cloud);								//滤波的原始数据
  pass.setFilterFieldName ("z");							//z方向上的滤波
  pass.setFilterLimits (0, 1.5);							//滤波半径
  pass.filter (*cloud_filtered);							//滤波后的数据
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  // 过滤后的点云进行法线估计，为后续进行基于法线的分割准备数据
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);			//设置目标几何形状，有条件限制的平面，最大法线角度偏差
  seg.setNormalDistanceWeight (0.1);						//法线允许偏差
  seg.setMethodType (pcl::SAC_RANSAC);						//设置分割方法：随机采样一致
  seg.setMaxIterations (100);								//设置最大迭代次数
  seg.setDistanceThreshold (0.03);							//平面上的最大偏差
  seg.setInputCloud (cloud_filtered);						//输入点云
  seg.setInputNormals (cloud_normals);						//输入法线
  //获取平面模型的系数和处在平面的内点
  seg.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // 从点云中抽取分割的处在平面上的点集
  extract.setInputCloud (cloud_filtered);					//设置输入点云数据
  extract.setIndices (inliers_plane);						//设置指定点云数据
  extract.setNegative (false);								//不反转设置（反转：上面indices之外的为指定点）

  // 存储分割得到的平面上的点到点云文件
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_plane);							//过滤掉上面其他点，输出指定点，即为平面
  std::cerr << "PointCloud representing the planar component: "
			<< cloud_plane->points.size () 
			<< " data points." << std::endl;
  writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);								//设置指定点反转，除平面之外的点为指定点
  extract.filter (*cloud_filtered2);						//将平面上的物体点云输出到指针
  extract_normals.setNegative (true);						//法线指定点设置反转
  extract_normals.setInputCloud (cloud_normals);			//设置输入法线
  extract_normals.setIndices (inliers_plane);				//设置设定点平面
  extract_normals.filter (*cloud_normals2);					//滤出平面上物体的法线数据

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);   					//设置对估计模型优化
  seg.setModelType (pcl::SACMODEL_CYLINDER); 		 		//设置分割模型为圆柱形
  seg.setMethodType (pcl::SAC_RANSAC);       				//参数估计方法
  seg.setNormalDistanceWeight (0.1);       					//设置表面法线权重系数
  seg.setMaxIterations (10000);              				//设置迭代的最大次数10000
  seg.setDistanceThreshold (0.05);         					//设置内点到模型的距离允许最大值
  seg.setRadiusLimits (0, 0.1);             				//设置估计出的圆柱模型的半径的范围
  seg.setInputCloud (cloud_filtered2);						//输入平面上的物体点云
  seg.setInputNormals (cloud_normals2);						//设置平面上物体的法线数据

  //获取圆柱模型的系数和处在平面的内点
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  //写入圆柱数据
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {
      std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
      writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
	  pcl::visualization::CloudViewer viewer("Cloud Viewer");
	  pcl::visualization::CloudViewer viewer1("Cloud Viewer");
	  pcl::visualization::CloudViewer viewer2("Cloud Viewer");
	  viewer.showCloud(cloud_cylinder);
	  viewer1.showCloud(cloud_filtered2);
	  viewer2.showCloud(cloud_plane);
	  std::getchar();
  }
  return (0);
}
```



![1](F:\program\PCL\learning note\learning_pcl\PCL\segmentation\img\1.jpg)

### kdTree&OcTree回顾

在学习这一部分时，先对前面所学习的KDtree和OCtree重新复习一下，前面就随便浏览一下，并不知道到时是用来干嘛的。

**1.谁是我邻居--kdTree&OcTree**

　　由于分割工作需要对点云的邻近点进行操作，不断对比和访问某个点的邻居，所以决定点云的相邻关系是非常重要的。对于Scan来说，邻居关系是天然的。但对于很多杂乱点云，或者滤波，分割后的点云来说，邻居关系就已经被破坏了。确定一个点云之间的相邻关系可以通过“树”来完成，目前比较主流的方法包括：kdTree和OcTree，这两种方法各有特点。

**1.1.kdTree---一种递归的邻近搜索策略**

　　关于kdTree到底是怎么工作的https://en.wikipedia.org/wiki/K-d_tree这里有非常详细的说明，我不再赘述。但是kdTree实际上包括两个部分：1.建立kdTree，2.在kdTree中查找。建立kdTree实际上是一个不断划分的过程，首先选择最sparse的维度，然后找到该维度上的中间点，垂直该维度做第一次划分。此时k维超平面被一分为二，在两个子平面中再找最sparse的维度，依次类推知道最后一个点也被划分。那么就形了一个不断二分的树。如图所示。

![img](https://img-blog.csdn.net/20170929102434903?watermark/2/text/aHR0cDovL2Jsb2cuY3Nkbi5uZXQvQXBwXzEyMDYyMDEx/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70/gravity/Center)　　

　　显然，一般情况下一个点的邻近点只需要在其父节点和子节点中搜索即可，大大缩小了邻近点的搜索规模。并且kdtree可以有效的对插入点进行判断其最近点在哪个位置。对于低层次视觉来说kdTree算法是非常重要的。在很多情况下需要给出某个点，再查k临近点的编号，或者差某半径范围内的点。

这里简单的理解就是可以按照二分法周围的点一个一个慢慢连接下去。

**1.2 OcTree**

　　OcTree是一种更容易理解也更自然的思想。对于一个空间，如果某个角落里有个盒子我们却不知道在哪儿。但是"神"可以告诉我们这个盒子在或者不在某范围内，显而易见的方法就是把空间化成8个卦限，然后询问在哪个卦限内。再将存在的卦限继续化成8个。意思大概就是太极生两仪，两仪生四象，四象生八卦，就这么一直划分下去，最后一定会确定一个非常小的空间。对于点云而言，只要将点云的立方体凸包用octree生成很多很多小的卦限，那么在相邻卦限里的点则为相邻点。

　　显然，对于不同点云应该采取不同的搜索策略，如果点云是疏散的，分布很广泛，且每什么规律（如lidar测得的点云或双目视觉捕捉的点云）kdTree能更好的划分，而octree则很难决定最小立方体应该是多少。太大则一个立方体里可能有很多点云，太小则可能立方体之间连不起来。如果点云分布非常规整，是某个特定物体的点云模型，则应该使用ocTree，因为很容易求解凸包并且点与点之间相对距离无需再次比对父节点和子节点，更加明晰。典型的例子是斯坦福的兔子。

###欧几里得算法

算法的原理在PCL相关的教程中已经说的比较清楚了，我不再给出伪代码。我想用一个故事来讲讲这个问题。从前有一个脑筋急转弯，说一个锅里有两粒豆子，如果不用手，要怎么把它们分开。当时的答案是豆子本来就是分开的，又没黏在一起，怎么不叫分开。OK，实际上欧几里德算法就是这个意思。两团点云就像是两粒豆子，只要找到某个合适的**度量方式**，就有办法把点云和点云分开。区分豆子我们用的方法可以归结于，两个豆子之间的距离小于分子距离，所以它们并没有连在一起。如果两团点云之间最近两点的距离小于单个点云内部点之间的距离，则可以由算法判断其分为两类。假设总点云集合为A，聚类所得点云团为Q

　　具体的实现方法大致是：

1. 找到空间中某点p10，有kdTree找到离他最近的n个点，判断这n个点到p的距离。将距离小于阈值r的点p12,p13,p14....放在类Q里
2. 在 Q\p10 里找到一点p12,重复1
3. 在 Q\p10,p12 找到找到一点，重复1，找到p22,p23,p24....全部放进Q里
4. 当 Q 再也不能有新点加入了，则完成搜索了

　　听起来好像这个算法并没什么用，因为点云总是连成片的，很少有什么东西会浮在空中让你来分。但是如果和前面介绍的内容联系起来就会发现这个算法威力巨大了。比如

1. 半径滤波删除离群点
2. 采样一致找到桌面
3. 抽掉桌面。。。。。

　　显然，一旦桌面被抽，桌上的物体就自然成了一个个的浮空点云团。就能够直接用欧几里德算法进行分割了。如图所示。

![img](https://img-blog.csdn.net/20170929102450006?watermark/2/text/aHR0cDovL2Jsb2cuY3Nkbi5uZXQvQXBwXzEyMDYyMDEx/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70/gravity/Center)

　　PCL对欧几里德算法进行了很好的封装，其代码如下：

```
  //被分割出来的点云团（标号队列）
  std::vector<pcl::PointIndices> cluster_indices;
  //欧式分割器
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  //搜索策略树
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);
```

