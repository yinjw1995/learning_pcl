2018.3.28 by yinjw

[my_learning对小车点云的简单可视化.md](https://github.com/yinjw1995/learning_pcl/blob/master/PCL/my_learning%E5%AF%B9%E5%B0%8F%E8%BD%A6%E7%82%B9%E4%BA%91%E7%9A%84%E7%AE%80%E5%8D%95%E5%8F%AF%E8%A7%86%E5%8C%96.md)

## ASCII-->PCD

首先，在对点云数据处理之前，需要先将收集到的数据格式进行转换，因为在PCL中，使用较多并且效率较高的是.pcd文件，但是传感器收集到的数据一般为.asc文件，这里，我通过使用Python编写了一个数据转换的脚本，方便的直接将.asc文件直接转换成.pcd

这里，先简单的回顾一下两种格式的基本知识

### asc文件

传感器收集到的许多数据都是依照ASCII码的形式进行存储，包括点云数据，如果需要打开可以使用Excel或者wps打开都可以，但是推荐使用notepad++，这是一个类似于记事本的工具，可以很开的打开各种程序文件，.c, .cpp, .data等

### pcd文件

如果用pcl来处理点云的话,那么点云需要先转化为pcd格式.pcd格式有很多版本，常用的版本为0.5版本和0.7版本,（可以搜索安装的PCL里面的.pcd文件，看里面是哪个版本）我的是0.7，如果不清楚自己电脑上是哪个可以就使用0.5，。

每一个PCD文件包含一个文件头，它确定和声明文件中存储的点云数据的某种特性。PCD文件头必须用ASCII码来编码。PCD文件中指定的每一个文件头字段以及ascii点数据都用一个新行（\n）分开了，从0.7版本开始，PCD文件头包含下面的字段：（可参考0点云库基础知识）

1. **VERSION** –指定PCD文件版本
2. **FIELDS** –指定一个点可以有的每一个维度和字段的名字

```
FIELDS x y z # XYZ data 
FIELDS x y z rgb # XYZ + colors 
FIELDS x y z normal_xnormal_y normal_z # XYZ + surface normals 
FIELDS j1 j2 j3 # moment invariants 
...
```

3. **SIZE** –说明每个字段的字节数大小

```
unsigned char/char has 1 byte 
unsigned short/short has 2 bytes 
unsignedint/int/float has 4 bytes 
double has 8 bytes
```

```
FIELDS x y z rgb
SIZE 4 4 4 412
```

4. **TYPE** -用一个字符来说明每个字段的数据类型，这些数据类型包括：

```
I –表示有符号类型int8（char）、int16（short）和int32（int）； 
U – 表示无符号类型uint8（unsigned char）、uint16（unsigned short）和uint32（unsigned int）； 
F –表示浮点类型。
```

5. **COUNT** –指定每一个维度包含的元素数目。例如，x这个数据通常有一个元素，但是像VFH这样的特征描述子就有308个。实际上这是在给每一点引入n维直方图描述符的方法，把它们当做单个的连续存储块。默认情况下，如果没有COUNT，所有维度的数目被设置成1。
6. **WIDTH** –用点的数量表示点云数据集的宽度。根据是有序点云还是无序点云，WIDTH有两层解释： 
   1)它能确定无序数据集的点云中点的个数（和下面的POINTS一样）； 
   2)它能确定有序点云数据集的宽度（一行中点的数目）。 
   注意：有序点云数据集，意味着点云是类似于图像（或者矩阵）的结构，数据分为行和列。这种点云的实例包括立体摄像机和时间飞行摄像机生成的数据。有序数据集的优势在于，预先了解相邻点（和像素点类似）的关系，邻域操作更加高效，这样就加速了计算并降低了PCL中某些算法的成本。
7. **HEIGHT** –用点的数目表示点云数据集的高度。类似于WIDTH ，HEIGHT也有两层解释： 
   1)它表示有序点云数据集的高度（行的总数）； 
   2)对于无序数据集它被设置成1（被用来检查一个数据集是有序还是无序）。
8. **VIEWPOINT**–指定数据集中点云的获取视点。VIEWPOINT有可能在不同坐标系之间转换的时候应用，在辅助获取其他特征时也比较有用，例如曲面法线，在判断方向一致性时，需要知道视点的方位， 
   视点信息被指定为平移（txtytz）+四元数（qwqxqyqz）。默认值是： 
   VIEWPOINT 0 0 0 1 0 0 0
9. **POINTS**–指定点云中点的总数。从0.7版本开始，该字段就有点多余了，因此有可能在将来的版本中将它移除。 
   例子： 
   POINTS 307200 #点云中点的总数为307200
10. **DATA** –指定存储点云数据的数据类型。从0.7版本开始，支持两种数据类型：ascii和二进制。查看下一节可以获得更多细节。

---

PCD文件的文件头部分必须以上面的顺序精确指定，也就是如下顺序： 
VERSION、FIELDS、SIZE、TYPE、COUNT、WIDTH、HEIGHT、VIEWPOINT、POINTS、DATA 
之间用换行隔开。

python 程序

```python
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 27 10:03:20 2018

@author: yinjw
"""

import time
from sys import argv
#script ,filename = argv
filename = "depth_0.asc"                                #这里输入需要转换的文件
output_filename="depth_0.pcd"                           #这里输出转换后的文件
print ("the input file name is:%r." %filename)

start = time.time()
print ("open the file...")
file = open(filename,"r+")
count = 0
                                                        #统计源文件的点数
for line in file:
    count=count+1
print ("size is %d" %count)
file.close()

#output = open("out.pcd","w+")
f_prefix = filename.split('.')[0]
output_filename = '{prefix}.pcd'.format(prefix=f_prefix)
output = open(output_filename,"w+")

list = ['# .PCD v.5 - Point Cloud Data file format\n'
        ,'VERSION .5\n'
        ,'FIELDS x y z normal_x normal_y normal_z\n'
        ,'SIZE 4 4 4 4 4 4\n'
        ,'TYPE F F F F F F\n'
        ,'COUNT 1 1 1 1 1 1\n']

output.writelines(list)
output.write('WIDTH ')                                  #注意后边有空格
output.write(str(count))
output.write('\nHEIGHT ')
output.write(str(1))                                    #强制类型转换，文件的输入只能是str格式
output.write('\nPOINTS ')
output.write(str(count))
output.write('\nDATA ascii\n')
file1 = open(filename,"r")
all = file1.read()
output.write(all)
output.close()
file1.close()

end = time.time()
print ("run time is: ", end-start)
```

## 小车数据CloudViewer显示

```
# .PCD v.5 - Point Cloud Data file format
VERSION .5
FIELDS x y z normal_x normal_y normal_z
SIZE 4 4 4 4 4 4
TYPE F F F F F F
COUNT 1 1 1 1 1 1
WIDTH 68408
HEIGHT 1
POINTS 68408
DATA ascii
-0.488211 0.380502 -1.08428 -0.268393 0.275282 -0.923139
-0.492224 0.38519 -1.08171 -0.314159 0.220326 -0.92345
-0.496974 0.390039 -1.07902 -0.330626 0.294386 -0.896674
-0.496972 0.393843 -1.07712 -0.275568 0.435103 -0.857174
-0.496756 0.3976 -1.07525 -0.284788 0.422358 -0.860528
-0.497253 0.401514 -1.07326 -0.280888 0.374121 -0.883818
```

通过上面的程序，将.asc文件转换成.pcd文件，用于程序中对点云数据的读取（有人说可以在PCL中直接使用.asc文件，目前我不懂）。

这里，只是使用CloudViewer进行点云的简单可视化，如果需要进行进一步数据处理，如深度图像等，需要使用到visualization类，这里暂时不考虑。

对于CloudViewer的简单可视化而言，其步骤可以总结为3点：

1. 申明所需要使用到的头文件，主要包括四类pcl/visualization/cloud_viewer，iostream，pcl/io/io，pcl/io/pcd_io，具体说明见程序
2. 初始化可视化窗口函数，viewerOneOff，这是自己定义的，主要是在创立可视化窗口后，进行一些最基本的设置，如背景颜色，该函数只运行一次，也可以不使用。
3. 定义与线程同步使用的函数viewerPsycho，该函数对每一帧图片进行显示时执行一次，也可以不使用
4. 主函数步骤：
   - 定义点云数据类别，PointCloud<PointXYZ>::Ptr cloud()
   - 读取pcl文件，将其赋值到对应的指针,io::loadPCDFile()
   - 创建可视化窗口，visualization::CloudViewer viewer()
   - 显示，viewer.showCloud()

具体程序如下：

```
#include <pcl/visualization/cloud_viewer.h>					//类cloud_viewer头文件申明
#include <iostream>											//标准输入输出头文件申明
#include <pcl/io/io.h>										//I/O相关头文件申明
#include <pcl/io/pcd_io.h>									//PCD文件读取
using namespace std;										//直接使用标准空间

void
viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(1.0, 0.5, 0.5);				//设置背景颜色
	std::cout << "i only run once" << std::endl;
}
/**************************************************************************
  函数是作为回调函数，在主函数中只注册一次 ，
  函数实现对可视化对象背景颜色的设置
**************************************************************************/

void
viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	static unsigned count = 0;								//定义无符号计数器
	stringstream ss;										//定义字符串流
	ss << "次数Once per viewer loop: " << count++;							//组合字符串
	viewer.removeAllShapes(0);								//去掉所有的图形，防止text重新添加
	viewer.addText(ss.str(), 200, 300, "text", 0);			//添加text文本

}


int main()
{
	using namespace pcl;
	PointCloud<PointXYZRGBA>::Ptr cloud(new PointCloud<PointXYZRGBA>);
	io::loadPCDFile("point_cloud.pcd",*cloud);				//读取.pcd文件
	visualization::CloudViewer viewer("Cloud Viewer");		//创建可视化窗口
	viewer.showCloud(cloud);								//显示所需要读取的数据指针
	
	//该线程中执行一次，即窗口弹出时调用一次
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	
	//在该线程中运行，即窗口显示时同步调用
	viewer.runOnVisualizationThread(viewerPsycho);
	std::getchar();
	return 0;
}
```

**程序的编译**

这里使用cmake进行编译，因为按照之前直接在vs上配置经常出错，这里使用较为方便的cmake 。

其程序和CmakeLists.txt文件已经包含在source文件夹里面了，可以直接下载里面的文件，CmakeLists.txt中每行程序的含义见[**1 点云数据的读取**](https://github.com/yinjw1995/learning_pcl/blob/master/PCL/1%20%E7%82%B9%E4%BA%91%E6%95%B0%E6%8D%AE%E7%9A%84%E8%AF%BB%E5%8F%96.md)

对于cmake的使用，这里再复习一遍。

首先要创建一个空文件夹cmake-bin，这个文件夹用来放编译后的文件，里面必须为空，如果以后还要使用这个文件夹，在使用编译其他程序时删除里面的文件就可以了。source文件夹中放编写好的.cpp文件和CmakeLists.txt文件，.cpp文件可以使用记事本编写，也可以使用notepad，使用VS同样也可以。

![1](https://github.com/yinjw1995/learning_pcl/raw/master/PCL/my_learning/img/1.jpg)

![2](https://github.com/yinjw1995/learning_pcl/raw/master/PCL/my_learning/img/2.jpg)

source文件夹和cmake-bin可不再同一目录

![3](https://github.com/yinjw1995/learning_pcl/raw/master/PCL/my_learning/img/3.jpg)

![4](https://github.com/yinjw1995/learning_pcl/raw/master/PCL/my_learning/img/4.jpg)

打开cmake(GUI)，选择对应的两个文件夹目录，configure，generate。然后在cmake-bin文件夹中找到.sln文件，这个就是自己所创建的解决方案，在VS中打开，并且打开源代码处Ctrl+F5编译即可，注意设置启动项目，选择“当前选定内容”，否则会出错。同样也要注意把.pcd文件拷到cmake-bin中。

![5](https://github.com/yinjw1995/learning_pcl/raw/master/PCL/my_learning/img/5.jpg)

在Debug中有对应的.exe文件，可以直接运行，同样，对应的.pcd文件要在同一文件夹上。

运行后的结果如下：

![6](https://github.com/yinjw1995/learning_pcl/raw/master/PCL/my_learning/img/6.jpg)

