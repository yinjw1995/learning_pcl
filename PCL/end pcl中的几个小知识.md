##时间计算

pcl中计算程序运行时间有很多函数，其中利用控制台的时间计算是：
首先必须包含头文件 `#include <pcl/console/time.h>`，其次，`pcl::console::TicToc time; time.tic(); +程序段 + cout<<time.toc()/1000<<"s"<<endl;`就可以以秒输出“程序段”的运行时间。

##如何实现类似pcl::PointCloud::Ptr和pcl::PointCloud的两个类相互转换？

```
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
 
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointer(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ> cloud;
cloud = *cloudPointer;
cloudPointer = cloud.makeShared();
```

##如何查找点云的x，y，z的极值？

```
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile<pcl::PointXYZ> ("your_pcd_file.pcd", *cloud);
pcl::PointXYZ minPt, maxPt;
pcl::getMinMax3D (*cloud, minPt, maxPt);
```

##如果知道需要保存点的索引，如何从原点云中拷贝点到新点云？

```
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
 
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile<pcl::PointXYZ>("C:\office3-after21111.pcd", *cloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
std::vector<int > indexs = { 1, 2, 5 };
pcl::copyPointCloud(*cloud, indexs, *cloudOut);
```

##如何从点云里删除和添加点？

```
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
 
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile<pcl::PointXYZ>("C:\office3-after21111.pcd", *cloud);
pcl::PointCloud<pcl::PointXYZ>::iterator index = cloud->begin();
cloud->erase(index);//删除第一个
index = cloud->begin() + 5;
cloud->erase(cloud->begin());//删除第5个
pcl::PointXYZ point = { 1, 1, 1 };
//在索引号为5的位置1上插入一点，原来的点后移一位
cloud->insert(cloud->begin() + 5, point);
cloud->push_back(point);//从点云最后面插入一点
std::cout << cloud->points[5].x;//输出1
```

如果删除的点太多建议用上面的方法拷贝到新点云，再赋值给原点云，如果要添加很多点，建议先resize，然后用循环向点云里的添加。

##如何对点云进行全局或局部变换

```
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile("path/.pcd",*cloud);
//全局变化
 //构造变化矩阵
        Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
        float theta = M_PI/4;   //旋转的度数，这里是45度
        transform_1 (0,0) = cos (theta);  //这里是绕的Z轴旋转
        transform_1 (0,1) = -sin(theta);
        transform_1 (1,0) = sin (theta);
        transform_1 (1,1) = cos (theta);
        //   transform_1 (0,2) = 0.3;   //这样会产生缩放效果
        //   transform_1 (1,2) = 0.6;
        //    transform_1 (2,2) = 1;
        transform_1 (0,3) = 25; //这里沿X轴平移
        transform_1 (1,3) = 30;
        transform_1 (2,3) = 380;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud,*transform_cloud1,transform_1);  //不言而喻
        
        //局部
        pcl::transformPointCloud(*cloud,pcl::PointIndices indices,*transform_cloud1,matrix); //第一个参数为输入，第二个参数为输入点云中部分点集索引，第三个为存储对象，第四个是变换矩阵。
```

##链接两个点云字段（两点云大小必须相同）

```
         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
         pcl::io::loadPCDFile("/home/yxg/pcl/pcd/mid.pcd",*cloud);
         pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
        ne.setInputCloud(cloud);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>()); 
        ne.setKSearch(8);
        //ne.setRadisuSearch(0.3);
        ne.compute(*cloud_normals);    
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_nomal (new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields(*cloud,*cloud_normals,*cloud_with_nomal);
```

##如何从点云中删除无效点

pcl中的无效点是指：点的某一坐标值为nan.

```
 #include <pcl/point_cloud.h>
    #include <pcl/point_types.h>
    #include <pcl/filters/filter.h>
    #include <pcl/io/pcd_io.h>
    
    using namespace std;
    typedef pcl::PointXYZRGBA point;
    typedef pcl::PointCloud<point> CloudType;
    
    int main (int argc,char **argv)
    {
            CloudType::Ptr cloud (new CloudType);
            CloudType::Ptr output (new CloudType);
    
            
            pcl::io::loadPCDFile(argv[1],*cloud);
            cout<<"size is:"<<cloud->size()<<endl;
            
            
            vector<int> indices;
            pcl::removeNaNFromPointCloud(*cloud,*output,indices);
            cout<<"output size:"<<output->size()<<endl;
            
    
            pcl::io::savePCDFile("out.pcd",*output);
    
            return 0;
    }
```

##将xyzrgb格式转换为xyz格式的点云

```
#include <pcl/io/pcd_io.h>
#include <ctime>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace std;
typedef pcl::PointXYZ point;
typedef pcl::PointXYZRGBA pointcolor;

int main(int argc,char **argv)
{
        pcl::PointCloud<pointcolor>::Ptr input (new pcl::PointCloud<pointcolor>);
        pcl::io::loadPCDFile(argv[1],*input);
        

        pcl::PointCloud<point>::Ptr output (new pcl::PointCloud<point>);
        int M = input->points.size();
        cout<<"input size is:"<<M<<endl;

        for (int i = 0;i <M;i++)
        {
                point p;
                p.x = input->points[i].x;
                p.y = input->points[i].y;
                p.z = input->points[i].z; 
                output->points.push_back(p);
        }
        output->width = 1;
        output->height = M;
        
        cout<< "size is"<<output->size()<<endl;
        pcl::io::savePCDFile("output.pcd",*output);

}
```

##flann kdtree 查询k近邻

```
   //平均密度计算
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  //创建一个快速k近邻查询,查询的时候若该点在点云中，则第一个近邻点是其本身
        kdtree.setInputCloud(cloud);
        int k =2;
        float everagedistance =0;
        for (int i =0; i < cloud->size()/2;i++)
        {
                vector<int> nnh ;
                vector<float> squaredistance;
                //  pcl::PointXYZ p;
                //   p = cloud->points[i];
                kdtree.nearestKSearch(cloud->points[i],k,nnh,squaredistance);
                everagedistance += sqrt(squaredistance[1]);
                //   cout<<everagedistance<<endl;
        }

        everagedistance = everagedistance/(cloud->size()/2);
        cout<<"everage distance is : "<<everagedistance<<endl;
        
```

```
#include <pcl/kdtree/kdtree_flann.h>
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree; //创建KDtree
        kdtree.setInputCloud (in_cloud);
        pcl::PointXYZ searchPoint; //创建目标点，（搜索该点的近邻）
        searchPoint.x = 1;
        searchPoint.y = 2;
        searchPoint.z = 3;

        //查询近邻点的个数
        int k = 10; //近邻点的个数
        std::vector<int> pointIdxNKNSearch(k); //存储近邻点集的索引
        std::vector<float>pointNKNSquareDistance(k); //近邻点集的距离
        if (kdtree.nearestKSearch(searchPoint,k,pointIdxNKNSearch,pointNKNSquareDistance)>0)
        {
                for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
                        std::cout << "    "  <<   in_cloud->points[ pointIdxNKNSearch[i] ].x 
                                  << " " << in_cloud->points[ pointIdxNKNSearch[i] ].y 
                                  << " " <<in_cloud->points[ pointIdxNKNSearch[i] ].z 
                                  << " (squared distance: " <<pointNKNSquareDistance[i] << ")" << std::endl;
        }

        //半径为r的近邻点
        float radius = 40.0f;  //其实是求的40*40距离范围内的点
        std::vector<int> pointIdxRadiusSearch;  //存储的对应的平方距离
        std::vector<float> a;
        if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, a) > 0 )
        {
          for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
                  std::cout << "    "  <<   in_cloud->points[ pointIdxRadiusSearch[i] ].x 
                            << " " <<in_cloud->points[ pointIdxRadiusSearch[i] ].y 
                            << " " << in_cloud->points[ pointIdxRadiusSearch[i] ].z 
                            << " (squared distance: " <<a[i] << ")" << std::endl;
        }
```

##关于`ply`文件

后缀命名为`.ply`格式文件，常用的点云数据文件。`ply`文件不仅可以存储**点**数据，而且可以存储**网格**数据. 用emacs打开一个`ply`文件，观察表头，如果表头`element face`的值为0,ze则表示该文件为点云文件，如果`element face`的值为某一正整数N，则表示该文件为网格文件，且包含N个网格.
所以利用pcl读取 ply 文件，不能一味用`pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PintT>)`来读取。
在读取`ply`文件时候，首先要分清该文件是点云还是网格类文件。如果是点云文件，则按照一般的点云类去读取即可，[官网例子](http://pointclouds.org/documentation/tutorials/interactive_icp.php),就是这样。
如果`ply`文件是网格类，则需要

```
    pcl::PolygonMesh mesh;
    pcl::io::loadPLYFile(argv[1],mesh);
    pcl::io::savePLYFile("result.ply", mesh);
```

读取。（官网例子之所以能成功，是因为它对模型进行了细分处理，使得网格变成了点）

##计算点的索引

例如sift算法中，pcl无法直接提供索引（主要原因是sift点是通过计算出来的，在某些不同参数下，sift点可能并非源数据中的点，而是某些点的近似），若要获取索引，则可利用以下函数：

```
void getIndices (pointcloud::Ptr cloudin, pointcloud keypoints, pcl::PointIndices::Ptr indices)
{
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloudin);
        std::vector<float>pointNKNSquareDistance; //近邻点集的距离
        std::vector<int> pointIdxNKNSearch;

        for (size_t i =0; i < keypoints.size();i++)
        {
                kdtree.nearestKSearch(keypoints.points[i],1,pointIdxNKNSearch,pointNKNSquareDistance);
                // cout<<"the distance is:"<<pointNKNSquareDistance[0]<<endl;
                // cout<<"the indieces is:"<<pointIdxNKNSearch[0]<<endl;
                
                indices->indices.push_back(pointIdxNKNSearch[0]);
                
        }

}
```

其思想就是：将原始数据插入到flann的kdtree中，寻找keypoints的最近邻，如果距离等于0,则说明是同一点，提取索引即可.

##计算质心

```
     Eigen::Vector4f centroid;  //质心
     pcl::compute3DCentroid(*cloud_smoothed,centroid); //估计质心的坐标
```

##从网格提取顶点（将网格转化为点）

```
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
using namespace pcl;
int main(int argc,char **argv)
{
        pcl::PolygonMesh mesh;
        //   pcl::io::loadPolygonFileOBJ(argv[1], mesh);
        pcl::io::loadPLYFile(argv[1],mesh);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
        pcl::io::savePCDFileASCII("result.pcd", *cloud);
return 0;
}
```

以上代码可以从.obj或.ply面片格式转化为点云类型。