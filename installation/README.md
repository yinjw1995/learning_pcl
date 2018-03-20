本仓库主要是对无人车雷达数据的点云处理进行学习，最新的笔记地址：https://github.com/yinjw1995/learning_pcl/blob/master/README.md

---

###使用Cmake对PCL1.8.1和VS2017进行配置

在第一次安装PCL1.8.1和VS2017的过程中，由于对二者都不太熟悉，仅仅安装之后并没有进行实际调试点云相关的程序，因此，实际上里面存在许多关于VS2017配置的问题，（*最终也没有弄清到底怎么实现，问题太多了*）

这里，我直接参考《点云库PCL学习教程》中的安装过程（*使用VS2008*）重新对VS2017重新配置，这次使用到了cmake工具。

在不使用cmake工具时，建立一个新的解决方案后，只要你要使用到PCL，都需要重新配置，修改里面的库目录、包含目录以及附加依赖项，其中的lib添加很容易出错，这样特别麻烦（*可能有方法解决，目前不知道*），通过使用Cmake，只需要一次配置即可。

**配置方法：**

需要下载Cmake工具：[cmake-3.11.0-rc4-win32-x86.msi](https://cmake.org/files/v3.11/cmake-3.11.0-rc4-win32-x86.msi)

需要下载VS2017，自行百度安装了

需要下载PCL1.8.1： [PCL-1.8.1-AllInOne-msvc2017-win32.exe](https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.8.1/PCL-1.8.1-AllInOne-msvc2017-win32.exe)  [ pcl-1.8.1-pdb-msvc2017-win32.zip](https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.8.1/pcl-1.8.1-pdb-msvc2017-win32.zip) 

（*这里我都是用的32位，如果要是用到64位找相应的文件即可*）

**安装过程：**

首先安装PCL1.8.1，安装过的可以不用再安装了

选择第二个将PCL添加到系统路径中，不然后续设置路径会出错，然后安装到自己设置的文件夹下

![1](F:\program\PCL\learning note\learning_pcl\installation\img\1.jpg)

![2](F:\program\PCL\learning note\learning_pcl\installation\img\2.jpg)

安装过程中，会弹出安装第三方库的窗口，这是后需要找到刚才安装目录下的第三方库文件夹"\3rdparty\"，然后继续直到完成

![4](F:\program\PCL\learning note\learning_pcl\installation\img\4.jpg)

1. 配置环境变量，win10是在右键电脑-属性-系统高级设置-环境变量，在下面的系统变量中找到Path，在里面添加下面的环境变量%PCL_ROOT%\bin;%PCL_ROOT%\3rdParty\FLANN\bin;%PCL_ROOT%\3rdParty\VTK\bin;%PCL_ROOT%\Qhull\bin;%PCL_ROOT%\3rdParty\OpenNI2\Tools，可以点击右下方的编辑文本将上面的变量名之间复制过去复制的时候注意在最前面加一个分号，否则不算独立的。

![5](F:\program\PCL\learning note\learning_pcl\installation\img\5.jpg)

![6](F:\program\PCL\learning note\learning_pcl\installation\img\6.jpg)

以上这些配置是最基本的系统配置，必不可少

**下面主要说明Cmake的使用过程：**

下载好安装文件之后，直接不带你next 直到安装完成，很简单

自己建立一个文件夹比如 TEXT，里面在建立两个子文件夹，

在source里面放需要测试的程序，文件：https://github.com/yinjw1995/learning_pcl/tree/master/source

说明一下，bunny.pcd是一部分数据，project_inliner.cpp是网上找的对应程序，当然也可以自己去找，这个任意。cmakelists.txt是必须包含的说明文件，说明了需要编译的文件和配置（可以自己修改里面的项目名），里面代码如下：

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(project_inliers)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (project_inliers project_inliers.cpp)
target_link_libraries (project_inliers ${PCL_LIBRARIES})

cmake-bin是放cmake编译后的文件

![8](F:\program\PCL\learning note\learning_pcl\installation\img\8.jpg)

![7](F:\program\PCL\learning note\learning_pcl\installation\img\7.jpg)

然后打开cmake，在上面两个选项中分别选择source，和cmake-bin，点击configure进行配置，其中会出现让你选择编译器，如果只有VS2017那就直接确定就行了，然后就会开始编译，出现红色很正常，不用管，最后点一下generate就可以了，都done就表示成功了

![9](F:\program\PCL\learning note\learning_pcl\installation\img\9.jpg)

![](F:\program\PCL\learning note\learning_pcl\installation\img\10.jpg)

然后打开cmake-bin文件夹，会发现有个project_inliers.sln，文件，这就是使用Cmake编译后的解决方案，直接使用VS2017打开就可以了，提醒一下，那个bunny.pcd需要自己复制过去，这是数据文件，不会再编译过程中使用的。

![](F:\program\PCL\learning note\learning_pcl\installation\img\11.jpg)

在VS中，直接在.CPP程序编辑框中按Ctrl+f5调试，一般没啥问题的

如果出现 无法启动程序F:\program\PCL\exm_lib\cmake-bin\debug\ALL_BUILD之类的，只需要将工程设置为启动项就好了

![](F:\program\PCL\learning note\learning_pcl\installation\img\13.jpg)

![](F:\program\PCL\learning note\learning_pcl\installation\img\14.jpg)

我的刚开始是在单启动项目那，出错了

如果出现 error LNK2019：无法解析的外部符号，需要在VC++目录--库目录中加入PCL1.8.1/lib文件；

在附加依赖项中加入：pcl下 的lib文件名：

如：

pcl_common_debug.lib
pcl_common_release.lib
pcl_features_debug.lib
pcl_features_release.lib
pcl_filters_debug.lib
pcl_filters_release.lib
pcl_io_debug.lib
pcl_io_ply_debug.lib
pcl_io_ply_release.lib
pcl_io_release.lib
pcl_kdtree_debug.lib
pcl_kdtree_release.lib
pcl_keypoints_debug.lib
pcl_keypoints_release.lib
pcl_ml_debug.lib
pcl_ml_release.lib
pcl_octree_debug.lib
pcl_octree_release.lib
pcl_outofcore_debug.lib
pcl_outofcore_release.lib
pcl_people_debug.lib
pcl_people_release.lib
pcl_recognition_debug.lib
pcl_recognition_release.lib
pcl_registration_debug.lib
pcl_registration_release.lib
pcl_sample_consensus_debug.lib
pcl_sample_consensus_release.lib
pcl_search_debug.lib
pcl_search_release.lib
pcl_segmentation_debug.lib
pcl_segmentation_release.lib
pcl_stereo_debug.lib
pcl_stereo_release.lib
pcl_surface_debug.lib
pcl_surface_release.lib
pcl_tracking_debug.lib
pcl_tracking_release.lib
pcl_visualization_debug.lib
pcl_visualization_release.lib

一般这种情况都是在附加依赖项中少添加了，如果还有其他的没添加，

例如要获取目录中E:\CPlusPlusLib\VTK_debug\VTK2008\lib\vtk-5.8下的所有静态链接库文件名并存储至文本.txt，方法如下： 
1、win+r 
2、输入：cmd回车 
3、输入：cd /d E:\CPlusPlusLib\VTK_debug\VTK2008\lib\vtk-5.8 回车 
4、输入：dir /b *.lib*>0.txt 回车 

在0.txt文件中即可获得。

http://blog.csdn.net/Operaboss/article/details/78761791

中间，我还遇到过程序无法找到OpenNI2.dll的情况，只需要在PCL的第三方库中搜索相应文件就好，看程序调试的时候那缺少就复制过去。

![](F:\program\PCL\learning note\learning_pcl\installation\img\15.jpg)

中间还有需要下载符号库啥的，不需要着急，等着就好，第一次比较慢，后面还有啥问题把错误代码复制百度就可以了，一般没啥问题。

最后运行的结果为：

![](F:\program\PCL\learning note\learning_pcl\installation\img\16.jpg)

