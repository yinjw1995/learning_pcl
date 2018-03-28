由于我对点云库的学习只是出于基础阶段，仅仅局限于傻瓜式使用，因此，对一些原理的知识无法做出明确的解释，只对使用过程中用到的部分进行说明，在使用过程中不断的补充这部分内容。

---

## PCL库简要说明

​     PCL（PointCloudLibrary）是在吸收了前人点云相关研究基础上建立起来的大型跨平台开源C++编程库，它实现了大量点云相关的通用算法和高效数据结构，涉及到点云获取、滤波、分割、配准、检索、特征提取、识别、追踪、曲面重建、可视化等。支持多种操作系统平台，可在Windows、Linux、Android、MacOSX、部分嵌入式实时系统上运行。如果说OpenCV是2D信息获取与处理的结晶，那么PCL就在3D信息获取与处理上具有同等地位，PCL是BSD授权方式，可以免费进行商业和学术应用  。（这部分自行百度了解）

###PCL的结构和内容：

PCL完全是一个模块化的现代C++模板库。其基于以下第三方库：Boost、Eigen、FLANN、VTK、CUDA、OpenNI、QHull，实现点云相关的获取、滤波、分割、配准、检索、特征提取、识别、追踪、曲面重建、可视化等。 
\- Boost：用于共享指针和线程； 
\- Eigen：用于矩阵、向量等数据操作； 
\- FLANN：用于在KD树模块中快速近邻搜索； 
\- VTK：在可视化模块中用于3D点云渲染和可视化；

为了进一步简化和开发，PCL被分成一系列较小的代码库： 
\- libpcl filters：采样、去除离群点、特征提取、拟合估计等过滤器。 
\- libpcl features：实现多种三维特征的筛选，如：曲面法线、曲率、边界点估计等。 
\- libpcl I/O：实现数据的输入和输出操作。 
\- libpcl surface：实现表面重建技术，如网格重建，凸包重建。 
\- libpcl register：实现点云配准方法，如ICP等。 
\- libpclkeypoints：实现不同的关键点提取方法。 
\- libpcl range：实现支持不同点云数据集生成的范围图像。

###常用到的一些函数和点云操作：

（这里都只是简单的预览说明，对基础知识有个了解，后面会具体的对每个使用进行说明）

**I/O操作**

pcl支持obj，pcd，ply等多种格式的点云输入和输出，如下：

```
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// 从文件中读入点云
typedef pcl::PointXYZRGB PointT;
pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

std::string dir = "E:\\datas\\";
std::string filename = "0kinect_1pcl.pcd";

if (pcl::io::loadPCDFile<PointT>((dir + filename), *cloud) == -1)
{
    //* load the file 
    PCL_ERROR("Couldn't read PCD file \n");
    return (-1);
}

// 保存点云为pcd的二进制格式文件
pcl::io::savePCDFileBinary((dir + "handled_" + filename), *handled_cloud)；12345678910111213141516171819
```

**可视化**

- 单个窗口显示

```
#include <pcl/visualization/cloud_viewer.h>
pcl::visualization::CloudViewer viewer("Display");
viewer.showCloud(cloud);

while (! viewer.wasStopped())
{


}123456789
```

- 多个窗口显示

```
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Display"));


int v1(0);
viewer->createViewPort(0, 0, 0.5, 1, v1);
viewer->setBackgroundColor(0, 0, 0, v1);
viewer->addText("0kinect_1pcl" , 10, 10, "v1 text", v1);
pcl::visualization::PointCloudColorHandlerRGBField <PointT> rgb(cloud);
viewer->addPointCloud<PointT>(cloud, rgb, "cloud", v1);


int v2(0);
viewer->createViewPort(0.5, 0, 1, 1, v2);
viewer->setBackgroundColor(0, 0, 0, v2);
viewer->addText("handled_0kinect_1pcl" , 10, 10, "v2 sftext", v2);
pcl::visualization::PointCloudColorHandlerRGBField <PointT> handled_rgb(handled_cloud);
viewer->addPointCloud<PointT>(handled_cloud, handled_rgb, "handled_cloud", v2);


viewer->initCameraParameters();
viewer->spin();123456789101112131415161718192021
```

- 更新点云并显示

```
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Display"));
viewer->addPointCloud<pcl::PointXYZRGB>(cloud);
viewer->spinOnce();
while()
{
    //cloud更新
    viewer->updatePointCloud(cloud);
    viewer->spinOnce();
}123456789
```

**刚体变换**

```
#include<pcl/registration/ndt.h>
// 旋转平移点云B
Eigen::Matrix4f RT;             // 旋转平移矩阵
RT <<
    0.937433421612, - 0.003769298084, - 0.348144203424, 0.286217689514,
    -0.018016768619, 0.998076498508 ,- 0.059319011867, 0.055899277329,
    0.347698122263 ,0.061880055815, 0.935562312603 ,0.052540421486,
    0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000;
pcl::transformPointCloud(*cloudB, *cloudB, RT);123456789
```

**滤波**

- 降采样

##PCL中可用的PointT类型：

PointXYZ——成员变量：float x,y,z;

​     PointXYZ是使用最常见的一个点数据类型，因为他之包含三维XYZ坐标信息，这三个浮点数附加一个浮点数来满足存储对齐，可以通过points[i].data[0]或points[i].x访问点X的坐标值

```
union
{
float data[4];
struct
{
float x;
float y;
float z;
};
};
```

PointXYZI——成员变量：float x,y,z,intensity

PointXYZI是一个简单的X Y Z坐标加intensity的point类型，是一个单独的结构体，并且满足存储对齐，由于point的大部分操作会把data[4]元素设置成0或1（用于变换），

不能让intensity与XYZ在同一个结构体中，如果这样的话其内容将会被覆盖，例如：两个点的点积会把第四个元素设置为0，否则点积没有意义，

```
union{  
float data[4];  
struct  
{  
float x;  
float y;  
float z;  
};  
};  
union{  
struct{  
float intensity;  
};  
float data_c[4];  
};  
```



 PointXYZRGBA——成员变量：float x,y,z;uint32_t  rgba  除了RGBA信息被包含在一个整型变量中，其他的和PointXYZI类似

```
union{
float data[4];
struct
{
float x;
float y;
float z;
};
};
union{
struct{
float rgba;
};
float data_c[4];
};
```

PointXYZRGB——float x,y,z,rgb   除了RGB信息被包含在一个浮点数据变量中，其他的和 PointXYZRGBA

```
union{
float data[4];
struct
{
float x;
float y;
float z;
};
};
union{
struct{
float rgb;
};
float data_c[4];
};
```

PointXY——成员变量：float x,y        简单的二维x-y结构代码

```
struct{
float x;
float y;
};
```

InterestPoint——成员变量：float x,y,z,strength除了strength表示关键点的强度测量值，其他的和PointXYZI

```
union{
float data[4];
struct
{
float x;
float y;
float z;
};
};
union{
struct{
float strength;
};
float data_c[4];
};
```

Normal——成员变量：float normal[3],curvature;

另一个常用的数据类型，Normal结构体表示给定点所在样本曲面上的法线方向，以及对应曲率的测量值，例如访问法向量的第一个坐标可以通过points[i].data_n[0]或者points[i].normal[0]或者points[i]

```
union{
float data_n[4]
float normal[3];
struct
{
float normal_x;
float normal_y;
float normal_z;
};
};
union{
struct{
float curvature;
};
float data_c[4];
};
```

PointNormal——成员变量：float x,y,z;   float normal[3] ,curvature ;  PointNormal是存储XYZ数据的point结构体，并且包括了采样点的法线和曲率

```
union{
float data[4];
struct
{
float x;
float y;
float z;
};
};

union{
float data_n[4]
float normal[3];
struct
{
float normal_x;
float normal_y;
float normal_z;
};
};
union{
struct{
float curvature;
};
float data_c[4];
};
```

## PCL基本数据类型

**PointCloud**

在PCL 1.x中最基本的数据类型就是PointCloud了。它是一个C++类，包含了如下的数据成员（括号中是这个数据的数据类型）： 
\- width(int) 
==指定了点云数据中的宽度==。width有两层含义： 
\- 可以指定点云的数量，但是只是对于**无序点云**而言。 
\- 指定**有序点云**中，一行点云的数量。

> 有序点云（organized point cloud ）：有序点云中的点排列情况类似图片（或矩阵），是按照行和列的结构排列的。这样的点云数据通常来自立体相机或TOF相机。有序点云的优势在于，通过知道两个相邻点的关系，最近邻点操作会变得更加有效，因此可以加速计算，并且降低PCL中一些算法的开销。
>
> 投影点云（projectable point cloud）：投影点云首先属于有序点云，并且投影点云中点的下标表示–（u,v）类似图片中像素的坐标，与这个点的3D坐标（x,y,z）有一定的关系。这个关系可以由针孔相机模型导出： 
> `$u = \frac{fx}{z}$` 和 `$v = \frac{fy}{z}$`

例如：

```
cloud.width = 640; // 每行有640个点1
```

- height(int)

  ==指定了点云数据中的高度==。height有两层含义：

  - 指定**有序点云**中，点云行的数量。
  - 对于**无序点云**，将height设为1（它的width即为点云的大小），以此来区分点云是否有序。 
    例如：

```
// 有序点云--类似图片中像素的组织结构，拥有640行，480列，总共有640*480 = 307200个点
cloud.width = 640; 
cloud.height = 480;123
```

```
// 无序点云--高度为1，宽度与点云大小相同为307200
cloud.width = 307200;
cloud.height = 1;123
```

- points(std::vector) 
  points是存储类型为PointT的点的向量。举例来说，对于一个包含XYZ数据的点云，points成员就是由pcl::PointXYZ类型的点构成的向量：

```
pcl::PointCloud<pcl::PointXYZ> cloud;
std::vector<pcl::PointXYZ> data = cloud.points;12
```

- is_dense(bool) 
  指定点云中的所有数据都是有限的（true），还是其中的一些点不是有限的，它们的XYZ值可能包含inf/NaN 这样的值（false）。
- sensor_origin_(Eigen::Vector4f) 
  指定传感器的采集位姿（==origin/translation==）这个成员通常是可选的，并且在PCL的主流算法中用不到。
- sensor_orientaion_(Eigen::Quaternionf) 
  指定传感器的采集位姿（方向）。这个成员通常是可选的，并且在PCL的主流算法中用不到。 
  为了简化开发，PointCloud类包含许多帮助成员函数。举个例子，如果你想判断一个点云是否有序，不用检查height是否等于1，而可以使用isOrganized()函数来判断：

```
if (!cloud.isOrganized ())
  ...12
```

> PointT代表了点云中的单元-点的类型。



##PCD（Point Cloud Data）文件格式

本文讲解了PCD（Point Cloud Data）的文件格式，还有在PCL中对它的用法

**PCD出现的背景**

PCD文件格式的出现，是为了弥补其他文件格式在PCL处理N维点云中的各种问题。 
PCD不是第一种支持3D点云数据的格式： 
\- PLY - 一种多边形文件格式 
\- STL - CAD软件得到的格式 
\- OBJ - 从几何学上定义的文件 
\- X3D - ISO标准的基于XML的文件格式 
\- 其他 
以上这些文件格式都有各自的缺点，因为他们是在不同时间根据不同的需求而提出来的，当时一些传感器技术和算法都还没有出现。

**文件头**

每个PCD文件都包含一个头部，声明了存于这个文件的点云的一些属性。**PCD的文件头必须用ASCII码编码**。

> 在PCD文件中，文件头的主体和用ASCII码书写的点数据（下面会讲）都要用换行符（\n）来分开。 
> 在0.7版本的PCD文件格式中，PCD头部包含下列字段： 
> \- VESION - 说明PCD文件格式的版本； 
> \- FIELDS - 说明一个点包含的每个维度/字段名称。例如：

```
FIELDS x y z                                # XYZ data
FIELDS x y z rgb                            # XYZ + colors
FIELDS x y z normal_x normal_y normal_z     # XYZ + surface normals
FIELDS j1 j2 j3                             # moment invariants
...12345
```

- SIZE - 说明每个字段的字节数大小。例如：

```
FIELDS x y z rgb
SIZE 4 4 4 412
```

这就说明了对于点云中的每个点，包含了它的X,y,z,rgb信息，并且每个字段都占4个字节 
\- TYPE - 用一个字符来说明每个字段的数据类型，这些数据类型包括： 
\- I - int8(char),int16(short),int32(int) 
\- U - uint8(Unsigned char),uint16(unsigned short),uint32(unsigned int) 
\- F - float 
\- COUNT - 说明了每个字段有多少个元素。举个例子，x字段通常只有一个元素，但是特征描述子，比如VFH就有308个元素。实际上，这是对每个点引入N维直方图描述子，并且将他们作为一个连续的内存块来看待。默认情况下，如果COUNT没有直接表示出来，所有字段的COUNT都是1。 
\- WIDTH - 用点的数量说明了点云数据集宽度。WIDTH有两层含义。 
\- 对于无序点云数据集而言，它就表示点云中点的数量（等价于下面的POINTS）。 
\- 对于有序点云数据集而言，它表示有序点云的宽度（一行点云的数目）。

> 有序点云数据集，意味着点云是类似于图像（或矩阵）的结构，它有行跟列。这种点云可以由立体视觉摄像机和TOF摄像机产生。它的优势在于，预先了解想邻点（和像素类似）的关系，邻域操作更加高效，加速了计算并降低了PCL中某些算法的成本。 
> \- HEIGHT - 用点的数量说明了点云数据集高度。HEIGHT有两层含义。 
> \- 对于有序点云数据集而言，它表示有序点云的高度（一列点云的数目）。 
> \- 对于无序点云数据集而言，通常被设为1（用来区分是否为有序点云）。 
> 举例：

```
WIDTH 640       # 有序类似于图像结构，640列，480行
HEIGHT 480      # 因此这个点云有640*480=307200个点

WIDTH 307200
HEIGHT 1        # 无序点云，有307200个点12345
```

- VIEWPOINT - 说明了这个点云数据集的获取视点。在后续的处理中可能会用到，比如不同坐标系下的变换，或是帮助获取表面法线向量特征，在判断方向一致性时，需要知道视点的方位。 
  视点由一组平移向量（tx,ty,tz）和一组四元数（qw,qx,qy,qz）组成。默认值如下：

```
VIEWPOINT 0 0 0 1 0 0 01
```

- POINTS - 说明了点云中所有点的数量。从0.7版本的PCD开始这个字段就有些多余了，所以我们希望之后可以去除这个字段。举例：

```
POINTS 3072001
```

- DATA - 说明了点云数据存储的数据类型。对于0.7版本PCD来说，支持两种数据类型：ASCII码和二进制码。

   

  ​

  > 在文件头最后一行（DATA）的下一字节就被认为是点云数据了。 
  > **PCD文件头部必须严格按照下面的方式排列：**

```
VERSION
FIELDS
SIZE
TYPE
COUNT
WIDTH
HEIGHT
VIEWPOINT
POINTS
DATA12345678910
```

**数据存储类型**

对于0.7版本PCD，它使用两种模式存储数据： - ASCII码，一个点用一行ASCII码表示：

```
p_1
p_2
p_3
p_4
...
p_n123456
```

> 从1.0.1版本PCD开始用“nan”来表示NaN

- 二进制码，此处的数据完全是pcl:: PointCloud.points(array/vector)的内存拷贝。在Linux系统中，我们用mmap/munmap操作符来以最快的速度进行读写数据。 
  使用哪种存储方式依据我们的应用环境。用简单直白的ASCII码表示，一行表示一个点，中间用空格或者tab键分开，不插入其他字符。或者是，用简单又快速的二进制码。ASCII码形式的点云允许用户打开点云文件，并用标准软件工具比如gnuplot将点云画出来，或者用sed，awk等软件操作点云。

**PCD点云格式的优势**

前面提到的文件结构都不具备PCD的灵活度和速度。下面列出了PCD文件格式的几个明显优势： 
\- 存储和处理有序点云数据集的能力-这对于实时处理应用来说具有重要意义，并且在增强现实（AR），机器人等研究领域应用广泛。 
\- 二进制mmap/munmap数据类型是从硬盘读写点云数据最快的一种格式。 
\- 保存了不同的数据类型（支持char,short,int,float ,double）使得点云数据处理和存储都变得十分灵活高效。无效的点云维度通常被存储为NAN类型。 
\- 特征描述子的N维直方图-对于3D识别/计算机视觉的应用具有重要意义 
另一个重要的优势在于，PCD数据格式是最适配于PCL库的，因此可以在PCL的应用上获得最好的性能，而不是先将其他类型转为PCL内置类型，这样会造成不必要的时间消耗。

> 虽然PCD是PCL的内置文件格式，但是pcl_io库也应该支持存储和加载其他提到的文件格式。

## 举例

下面是一个PCD文件的片段，可以将其导入CloudCompare软件中查看。

```
# .PCD v.7 - Point Cloud Data file format
VERSION .7
FIELDS x y z rgb
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH 213
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 213
DATA ascii
0.93773 0.33763 0 4.2108e+06
0.90805 0.35641 0 4.2108e+06
0.81915 0.32 0 4.2108e+06
0.97192 0.278 0 4.2108e+06
0.944 0.29474 0 4.2108e+06
0.98111 0.24247 0 4.2108e+06
0.93655 0.26143 0 4.2108e+06
0.91631 0.27442 0 4.2108e+06
```

