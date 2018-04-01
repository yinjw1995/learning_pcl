## PCL 可视化

可视化（visualization）是利用计算机图形学和图像处理技术，将数据转换图像在屏幕上显示出来，并进行交互处理的的理论，方法和技术，

pcl_visualization库建立了能够快速建立原型的目的和可视化算法对三维点云数据操作的结果。类似于opencv的highgui例程显示二维图像，在屏幕上绘制基本的二维图形，库提供了以下几点：

（1）渲染和设置视觉特性的方法（如颜色、大小、透明度等）在PCL任意n维的点云数据集pcl::PointCloud<T> format

​                           ![img](http://images2015.cnblogs.com/blog/976394/201702/976394-20170225162703507-224431255.jpg)

（2）在屏幕上绘制基本的3D形状的方法（例如，圆柱体，球体，线，多边形等），无论是从点集或参数方程；

​                         ![img](http://images2015.cnblogs.com/blog/976394/201702/976394-20170225163254476-1117619011.jpg)

（3）一个直方图可视化模块（pclhistogramvisualizer）的二维图；

​                      ![img](http://images2015.cnblogs.com/blog/976394/201702/976394-20170225163411148-291383248.jpg)

（4）大量的几何和颜色处理pcl::PointCloud<T> datasets

​                    ![img](http://images2015.cnblogs.com/blog/976394/201702/976394-20170225163519710-713569465.jpg)

（5）a [pcl::RangeImage](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image.html) 可视化模块

​                       ![img](http://images2015.cnblogs.com/blog/976394/201702/976394-20170225163609163-124442403.jpg).

1. class  pcl::visualization::CloudViewer   类CloudViewer实现创建点云可视化的窗口，以及相关的可视化功能  

   | Public Member Functions     |                                                              |
   | --------------------------- | ------------------------------------------------------------ |
   |                             | [CloudViewer](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_cloud_viewer.html#a6266ab90b03695787804cc8625d4d6b3) (const std::string &window_name)   构建可视化点云窗口，窗口名为window_name |
   |                             | [~CloudViewer](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_cloud_viewer.html#a219657de567db61ae47e046b9bcce327) ()     注销窗口相关资源 |
   | void                        | [showCloud](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_cloud_viewer.html#a845aa739fa2ed4c3e57a07e762a48c2b) (const [ColorCloud::ConstPtr](http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html#ad63ba0b541909de6e4a01ea59c7a156d) &cloud, const std::string &cloudname="cloud") |
   |                             | 可视化窗口显示cloud对应的点云，考虑到多个点云用键值cloudname来限定是哪一个点云 |
   | bool                        | [wasStopped](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_cloud_viewer.html#a7225294cad46d65a3f54c8a97dde1982) (int millis_to_wait=1) |
   |                             | 判断用户是否已经关闭窗口，如果是则需要注销窗口               |
   | void                        | [runOnVisualizationThread](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_cloud_viewer.html#a5ef661143b864be17d4c9243723fe0cc) ([VizCallable](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_cloud_viewer.html#adad0094081a2b6069a8d58bc4a0643b2) x, const std::string &key="callable") |
   |                             | 在窗口运行期间处理x的回调函数，key为键值标识此回调函数，知道窗口关闭 |
   | void                        | [runOnVisualizationThreadOnce](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_cloud_viewer.html#ae657abe24e9345e09ab58fb664fafe68) ([VizCallable](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_cloud_viewer.html#adad0094081a2b6069a8d58bc4a0643b2) x) |
   |                             | 值调用回调函数一次                                           |
   | void                        | [removeVisualizationCallable](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_cloud_viewer.html#a6804cb5e9b37362de1959cefd798b835) (const std::string &key="callable") |
   |                             | 删除key对应的回调函数                                        |
   | boost::signals2::connection | [registerKeyboardCallback](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_cloud_viewer.html#ae61290a8fb076cc5cf2901e6916dd238) (void(*callback)(const [pcl::visualization::KeyboardEvent](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_keyboard_event.html) &, void *), void *cookie=NULL) |
   |                             | 注册键盘事件回调函数，cookie为回调时的参数，callback为回调函数的指针 |
   | template<typename T >       |                                                              |
   | boost::signals2::connection | [registerKeyboardCallback](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_cloud_viewer.html#a81e195c5c84645cdb3d6683c1b5fa7f8) (void(T::*callback)(const [pcl::visualization::KeyboardEvent](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_keyboard_event.html) &, void *), T &instance, void *cookie=NULL) |
   |                             | 同上，其中的instance 指向是实现该回到函数的对象              |

2. 太多了这里就贴出所有的关于可视化的类    对于类的成员就不再一一介绍

   | class  | [pcl::visualization::CloudViewer](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_cloud_viewer.html) |
   | ------ | ------------------------------------------------------------ |
   |        | Simple point cloud visualization class. [More...](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_cloud_viewer.html#details) |
   | class  | [pcl::visualization::FloatImageUtils](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_float_image_utils.html) |
   |        | **Provide** some gerneral functionalities regarding 2d float arrays, e.g., for visualization purposes [More...](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_float_image_utils.html#details) |
   | class  | [pcl::visualization::PCLHistogramVisualizer](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_p_c_l_histogram_visualizer.html) |
   |        | PCL histogram visualizer main class. [More...](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_p_c_l_histogram_visualizer.html#details) |
   | class  | [pcl::visualization::ImageViewerInteractorStyle](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_image_viewer_interactor_style.html) |
   |        | An image viewer interactor style, tailored for [ImageViewer](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_image_viewer.html). [More...](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_image_viewer_interactor_style.html#details) |
   | struct | [pcl::visualization::ImageViewer::ExitMainLoopTimerCallback](http://docs.pointclouds.org/trunk/structpcl_1_1visualization_1_1_image_viewer_1_1_exit_main_loop_timer_callback.html) |
   | struct | [pcl::visualization::ImageViewer::ExitCallback](http://docs.pointclouds.org/trunk/structpcl_1_1visualization_1_1_image_viewer_1_1_exit_callback.html) |
   | class  | [pcl::visualization::ImageViewer](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_image_viewer.html) |
   |        | [ImageViewer](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_image_viewer.html) is a class for 2D image visualization. [More...](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_image_viewer.html#details) |
   | class  | [pcl::visualization::PCLVisualizerInteractorStyle](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_p_c_l_visualizer_interactor_style.html) |
   |        | [PCLVisualizerInteractorStyle](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_p_c_l_visualizer_interactor_style.html) defines an unique, custom VTK based interactory style for PCL Visualizer applications. [More...](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_p_c_l_visualizer_interactor_style.html#details) |
   | struct | [pcl::visualization::Figure2D](http://docs.pointclouds.org/trunk/structpcl_1_1visualization_1_1_figure2_d.html) |
   |        | Abstract class for storing figure information. [More...](http://docs.pointclouds.org/trunk/structpcl_1_1visualization_1_1_figure2_d.html#details) |
   | class  | [pcl::visualization::PCLPainter2D](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_p_c_l_painter2_d.html) |
   |        | PCL Painter2D main class. [More...](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_p_c_l_painter2_d.html#details) |
   | class  | [pcl::visualization::PCLPlotter](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_p_c_l_plotter.html) |
   |        | PCL Plotter main class. [More...](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_p_c_l_plotter.html#details) |
   | class  | [pcl::visualization::PCLVisualizer](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_p_c_l_visualizer.html) |
   |        | PCL Visualizer main class. [More...](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_p_c_l_visualizer.html#details) |
   | class  | [pcl::visualization::PointCloudColorHandler< PointT >](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_point_cloud_color_handler.html) |
   |        | Base Handler class for [PointCloud](http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html) colors. [More...](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_point_cloud_color_handler.html#details) |
   | class  | [pcl::visualization::PointCloudColorHandlerRandom< PointT >](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_point_cloud_color_handler_random.html) |
   |        | Handler for random [PointCloud](http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html) colors (i.e., R, G, B will be randomly chosen) [More...](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_point_cloud_color_handler_random.html#details) |
   | class  | [pcl::visualization::PointCloudColorHandlerCustom< PointT >](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_point_cloud_color_handler_custom.html) |
   |        | Handler for predefined user colors. [More...](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_point_cloud_color_handler_custom.html#details) |
   | class  | [pcl::visualization::PointCloudColorHandlerRGBField< PointT >](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_point_cloud_color_handler_r_g_b_field.html) |
   |        | [RGB](http://docs.pointclouds.org/trunk/structpcl_1_1_r_g_b.html) handler class for colors. [More...](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_point_cloud_color_handler_r_g_b_field.html#details) |
   | class  | [pcl::visualization::PointCloudColorHandlerHSVField< PointT >](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_point_cloud_color_handler_h_s_v_field.html) |
   |        | HSV handler class for colors. [More...](http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_point_cloud_color_handler_h_s_v_field.html#details) |



```
#include <pcl/visualization/cloud_viewer.h>   //类cloud_viewer头文件申明
#include <iostream>                           //标准输入输出头文件申明
#include <pcl/io/io.h>                        //I/O相关头文件申明
#include <pcl/io/pcd_io.h>                    //PCD文件读取
    

/**********************************************************************************
  函数是作为回调函数，在主函数中只注册一次 ，函数实现对可视化对象背景颜色的设置，添加一个圆球几何体
*********************************************************************************/
int user_data;
    
void 
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);       //设置背景颜色
    pcl::PointXYZ o;                                  //存储球的圆心位置
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);                  //添加圆球几何对象
    std::cout << "i only run once" << std::endl;
    
}
   /***********************************************************************************
   作为回调函数，在主函数中注册后每帧显示都执行一次，函数具体实现在可视化对象中添加一个刷新显示字符串
   *************************************************************************************/ 
void 
viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);
    
    //FIXME: possible race condition here:
    user_data++;
}
  /**************************************************************
  首先加载点云文件到点云对象，并初始化可视化对象viewer，注册上面的回
   调函数，执行循环直到收到关闭viewer的消息退出程序
   *************************************************************/  
int 
main ()
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);    //声明cloud 
    pcl::io::loadPCDFile ("my_point_cloud.pcd", *cloud);         //加载点云文件
    
    pcl::visualization::CloudViewer viewer("Cloud Viewer");      //创建viewer对象
    
    //showCloud函数是同步的，在此处等待直到渲染显示为止
    viewer.showCloud(cloud);
    
    //该注册函数在可视化的时候只执行一次
    viewer.runOnVisualizationThreadOnce (viewerOneOff);
    
    //该注册函数在渲染输出时每次都调用
    viewer.runOnVisualizationThread (viewerPsycho);
    while (!viewer.wasStopped ())
    {
    //此处可以添加其他处理
    //FIXME: Note that this is running in a separate thread from viewerPsycho
    //and you should guard against race conditions yourself...
    user_data++;
    }
    return 0;
}
```

## 可视化类

PCLVisualizer可视化类是PCL中功能最全的可视化类，与CloudViewer可视化类相比，PCLVisualizer使用起来更为复杂，但该类具有更全面的功能，如显示法线、绘制多种形状和多个视口。本小节将通过示例代码演示PCLVisualizer可视化类的功能，从显示单个点云开始。大多数示例代码都是用于创建点云并可视化其某些特征

代码注释解析

 

```
#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

// --------------
// -----Help-----
// --------------
void
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-s           Simple visualisation example\n"
            << "-r           RGB colour visualisation example\n"
            << "-c           Custom colour visualisation example\n"
            << "-n           Normals visualisation example\n"
            << "-a           Shapes visualisation example\n"
            << "-v           Viewports example\n"
            << "-i           Interaction Customization example\n"
            << "\n\n";
}
 /************************************************************************************************************
/*****************************可视化单个点云：应用PCL Visualizer可视化类显示单个具有XYZ信息的点云****************/
 /************************************************************************************************************/

//simpleVis函数实现最基本的点云可视化操作，
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  //创建视窗对象并给标题栏设置一个名称“3D Viewer”并将它设置为boost::shared_ptr智能共享指针，这样可以保证指针在程序中全局使用，而不引起内存错误
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
   //设置视窗的背景色，可以任意设置RGB的颜色，这里是设置为黑色
  viewer->setBackgroundColor (0, 0, 0); 
 /*这是最重要的一行，我们将点云添加到视窗对象中，并定一个唯一的字符串作为ID 号，利用此字符串保证在其他成员中也能
   标志引用该点云，多次调用addPointCloud可以实现多个点云的添加，，每调用一次就会创建一个新的ID号，如果想更新一个
   已经显示的点云，必须先调用removePointCloud（），并提供需要更新的点云ID 号，
  *******************************************************************************************/
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud"); 
  //用于改变显示点云的尺寸，可以利用该方法控制点云在视窗中的显示方法，
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
/*******************************************************************************************************
  查看复杂的点云，经常让人感到没有方向感，为了保持正确的坐标判断，需要显示坐标系统方向，可以通过使用X（红色）
  Y（绿色 ）Z （蓝色）圆柱体代表坐标轴的显示方式来解决，圆柱体的大小可以通过scale参数来控制，本例中scale设置为1.0
  
 ******************************************************************************************************/ 
  viewer->addCoordinateSystem (1.0);
 //通过设置照相机参数使得从默认的角度和方向观察点云
  viewer->initCameraParameters ();
  return (viewer);
}
/*****************************可视化点云颜色特征******************************************************/
 /**************************************************************************************************
 多数情况下点云显示不采用简单的XYZ类型，常用的点云类型是XYZRGB点，包含颜色数据，除此之外，还可以给指定的点云定制颜色
  以示得点云在视窗中比较容易区分。点赋予不同的颜色表征其对应的Z轴值不同，PCL Visualizer可根据所存储的颜色数据为点云
  赋色， 比如许多设备kinect可以获取带有RGB数据的点云，PCL Vizualizer可视化类可使用这种颜色数据为点云着色，rgbVis函数中的代码
用于完成这种操作。
  ***************************************************************************************************/
  /**************************************************************************
   与前面的示例相比点云的类型发生了变化，这里使用的点云带有RGB数据的属性字段，
  ****************************************************************************/
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  /***************************************************************************************************************
  设置窗口的背景颜色后，创建一个颜色处理对象，PointCloudColorHandlerRGBField利用这样的对象显示自定义颜色数据，PointCloudColorHandlerRGBField
   对象得到每个点云的RGB颜色字段，
  **************************************************************************************************************/
  
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}
/******************可视化点云自定义颜色特征**********************************************************/
 /****************************************************************************************************
 演示怎样给点云着上单独的一种颜色，可以利用该技术给指定的点云着色，以区别其他的点云，
 *****************************************************************************************************/
  //点云类型为XYZ类型，customColourVis函数将点云赋值为绿色，
boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  //创建一个自定义的颜色处理器PointCloudColorHandlerCustom对象，并设置颜色为纯绿色
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
  //addPointCloud<>()完成对颜色处理器对象的传递
  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

//*******************可视化点云法线和其他特征*************************************************/
 /*********************************************************************************************
  显示法线是理解点云的一个重要步骤，点云法线特征是非常重要的基础特征，PCL visualizer可视化类可用于绘制法线，
   也可以绘制表征点云的其他特征，比如主曲率和几何特征，normalsVis函数中演示了如何实现点云的法线，
  ***********************************************************************************************/
boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //实现对点云法线的显示
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

  //*****************绘制普通形状************************************************//
 /**************************************************************************************************************
  PCL visualizer可视化类允许用户在视窗中绘制一般图元，这个类常用于显示点云处理算法的可视化结果，例如 通过可视化球体
  包围聚类得到的点云集以显示聚类结果，shapesVis函数用于实现添加形状到视窗中，添加了四种形状：从点云中的一个点到最后一个点
  之间的连线，原点所在的平面，以点云中第一个点为中心的球体，沿Y轴的椎体
 *************************************************************************************************************/
boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud添加点云到视窗实例代码-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  /************************************************************************************************
  绘制形状的实例代码，绘制点之间的连线，
*************************************************************************************************/
  viewer->addLine<pcl::PointXYZRGB> (cloud->points[0],
                                     cloud->points[cloud->size() - 1], "line");
  //添加点云中第一个点为中心，半径为0.2的球体，同时可以自定义颜色
  viewer->addSphere (cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

  //---------------------------------------
  //-----Add shapes at other locations添加绘制平面使用标准平面方程ax+by+cz+d=0来定义平面，这个平面以原点为中心，方向沿着Z方向-----
  //---------------------------------------
  pcl::ModelCoefficients coeffs;
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (1.0);
  coeffs.values.push_back (0.0);
  viewer->addPlane (coeffs, "plane");
  //添加锥形的参数
  coeffs.values.clear ();
  coeffs.values.push_back (0.3);
  coeffs.values.push_back (0.3);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (1.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (5.0);
  viewer->addCone (coeffs, "cone");

  return (viewer);
}
/******************************************************************************************
 多视角显示：PCL  visealizer可视化类允许用户通过不同的窗口（Viewport）绘制多个点云这样方便对点云比较
 viewportsVis函数演示如何用多视角来显示点云计算法线的方法结果对比
******************************************************************************************/

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1, pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();
   //以上是创建视图的标准代码
  
  int v1(0);  //创建新的视口
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);  //4个参数分别是X轴的最小值，最大值，Y轴的最小值，最大值，取值0-1，v1是标识
  viewer->setBackgroundColor (0, 0, 0, v1);    //设置视口的背景颜色
  viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);  //添加一个标签区别其他窗口  利用RGB颜色着色器并添加点云到视口中
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud1", v1);
   //对第二视口做同样的操作，使得做创建的点云分布于右半窗口，将该视口背景赋值于灰色，以便明显区别，虽然添加同样的点云，给点云自定义颜色着色
  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
  viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud2", v2);
  //为所有视口设置属性，
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
  viewer->addCoordinateSystem (1.0);
  //添加法线  每个视图都有一组对应的法线
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals1, 10, 0.05, "normals1", v1);
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals2, 10, 0.05, "normals2", v2);

  return (viewer);
}
/*******************************************************************************************************
 这里是处理鼠标事件的函数，每次相应鼠标时间都会回电函数，需要从event实例提取事件信息，本例中查找鼠标左键的释放事件
 每次响应这种事件都会在鼠标按下的位置上生成一个文本标签。
 *********************************************************************************************************/

unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;

    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf (str, "text#%03d", i);
      viewer->removeShape (str);
    }
    text_id = 0;
  }
}
 /********************************************************************************************
 键盘事件 我们按下哪个按键  如果按下r健   则删除前面鼠标所产生的文本标签，需要注意的是，当按下R键时 3D相机仍然会重置
  所以在PCL中视窗中注册事件响应回调函数，不会覆盖其他成员对同一事件的响应
**************************************************************************************************/
void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

    char str[512];
    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}
 
/******************自定义交互*****************************************************************************/
 /******************************************************************************************************
  多数情况下，默认的鼠标和键盘交互设置不能满足用户的需求，用户想扩展函数的某一些功能，  比如按下键盘时保存点云的信息，
  或者通过鼠标确定点云的位置   interactionCustomizationVis函数进行演示如何捕捉鼠标和键盘事件，在窗口点击，将会显示
  一个2D的文本标签，按下r健出去文本
  ******************************************************************************************************/
  
boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  //以上是实例化视窗的标准代码
  viewer->addCoordinateSystem (1.0);
  //分别注册响应键盘和鼠标事件，keyboardEventOccurred  mouseEventOccurred回调函数，需要将boost::shared_ptr强制转换为void*
  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());
  viewer->registerMouseCallback (mouseEventOccurred, (void*)viewer.get ());
   
  return (viewer);
}


// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }
  bool simple(false), rgb(false), custom_c(false), normals(false),
    shapes(false), viewports(false), interaction_customization(false);
  if (pcl::console::find_argument (argc, argv, "-s") >= 0)
  {
    simple = true;
    std::cout << "Simple visualisation example\n";
  }
  else if (pcl::console::find_argument (argc, argv, "-c") >= 0)
  {
    custom_c = true;
    std::cout << "Custom colour visualisation example\n";
  }
  else if (pcl::console::find_argument (argc, argv, "-r") >= 0)
  {
    rgb = true;
    std::cout << "RGB colour visualisation example\n";
  }
  else if (pcl::console::find_argument (argc, argv, "-n") >= 0)
  {
    normals = true;
    std::cout << "Normals visualisation example\n";
  }
  else if (pcl::console::find_argument (argc, argv, "-a") >= 0)
  {
    shapes = true;
    std::cout << "Shapes visualisation example\n";
  }
  else if (pcl::console::find_argument (argc, argv, "-v") >= 0)
  {
    viewports = true;
    std::cout << "Viewports example\n";
  }
  else if (pcl::console::find_argument (argc, argv, "-i") >= 0)
  {
    interaction_customization = true;
    std::cout << "Interaction Customization example\n";
  }
  else
  {
    printUsage (argv[0]);
    return 0;
  }

  // ------------------------------------
  // -----Create example point cloud-----
  // ------------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::cout << "Genarating example point clouds.\n\n";
  // We're going to make an ellipse extruded along the z-axis. The colour for
  // the XYZRGB cloud will gradually go from red to green to blue.
  uint8_t r(255), g(15), b(15);
  for (float z(-1.0); z <= 1.0; z += 0.05)
  {
    for (float angle(0.0); angle <= 360.0; angle += 5.0)
    {
      pcl::PointXYZ basic_point;
      basic_point.x = 0.5 * cosf (pcl::deg2rad(angle));
      basic_point.y = sinf (pcl::deg2rad(angle));
      basic_point.z = z;
      basic_cloud_ptr->points.push_back(basic_point);

      pcl::PointXYZRGB point;
      point.x = basic_point.x;
      point.y = basic_point.y;
      point.z = basic_point.z;
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back (point);
    }
    if (z < 0.0)
    {
      r -= 12;
      g += 12;
    }
    else
    {
      g -= 12;
      b += 12;
    }
  }
  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  basic_cloud_ptr->height = 1;
  point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
  point_cloud_ptr->height = 1;

  // ----------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.05-----
  // ----------------------------------------------------------------
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (point_cloud_ptr);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.05);
  ne.compute (*cloud_normals1);

  // ---------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.1-----
  // ---------------------------------------------------------------
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.1);
  ne.compute (*cloud_normals2);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  if (simple)
  {
    viewer = simpleVis(basic_cloud_ptr);
  }
  else if (rgb)
  {
    viewer = rgbVis(point_cloud_ptr);
  }
  else if (custom_c)
  {
    viewer = customColourVis(basic_cloud_ptr);
  }
  else if (normals)
  {
    viewer = normalsVis(point_cloud_ptr, cloud_normals2);
  }
  else if (shapes)
  {
    viewer = shapesVis(point_cloud_ptr);
  }
  else if (viewports)
  {
    viewer = viewportsVis(point_cloud_ptr, cloud_normals1, cloud_normals2);
  }
  else if (interaction_customization)
  {
    viewer = interactionCustomizationVis();
  }

  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
```

 

编译生成可执行文件后，运行查看

   （1）  ./pcl_visualizer_demo -h

![img](http://images2015.cnblogs.com/blog/976394/201702/976394-20170226165406085-1624834740.png)  ![img](http://images2015.cnblogs.com/blog/976394/201702/976394-20170226170940163-1104964637.png)

 

依次执行查看结果

![img](http://images2015.cnblogs.com/blog/976394/201702/976394-20170226165604179-2109995964.png)   ![img](http://images2015.cnblogs.com/blog/976394/201702/976394-20170226170058007-1050676507.png)

 

![img](http://images2015.cnblogs.com/blog/976394/201702/976394-20170226170430023-1507316509.png)  ![img](http://images2015.cnblogs.com/blog/976394/201702/976394-20170226170534429-27121823.png)

![img](http://images2015.cnblogs.com/blog/976394/201702/976394-20170226170653757-1925111468.png)  ![img](http://images2015.cnblogs.com/blog/976394/201702/976394-20170226170756616-138797262.png)

## 深度图像

目前深度图像的获取方法有激光雷达深度成像法，计算机立体视觉成像，坐标测量机法，莫尔条纹法，结构光法等等，针对深度图像的研究重点主要集中在以下几个方面，深度图像的分割技术 ，深度图像的边缘检测技术 ，基于不同视点的多幅深度图像的配准技术，基于深度数据的三维重建技术，基于三维深度图像的三维目标识别技术，深度图像的多分辨率建模和几何压缩技术等等，在PCL 中深度图像与点云最主要的区别在于  其近邻的检索方式的不同，并且可以互相转换。

（这一章是我认为非常重要的）

模块RangeImage相关概念以及算法的介绍

深度图像（Depth Images）也被称为距离影像（Range Image），是指将从图像采集器到场景中各点的距离值作为像素值的图像，它直接反应了景物可见表面的几何形状，利用它可以很方便的解决3D目标描述中的许多问题，深度图像经过点云变换可以计算为点云数据，有规则及有必要信息的点云数据可以反算为深度图像数据

![img](http://images2015.cnblogs.com/blog/976394/201702/976394-20170227143414173-301559318.png)

不同视角获得深度图像的过程：

​                                  ![img](http://images2015.cnblogs.com/blog/976394/201702/976394-20170227143440204-95635786.png)

   （1）PCL中的模块RangeImage相关类的介绍

 pcl_range_image库中包含两个表达深度图像和对深度图像进行操作的类，其依赖于pcl::common模块，深度图像（距离图像）的像素值代表从传感器到物体的距离以及深度，  深度图像是物体的三维表示形式，一般通过立体照相机或者ToF照相机获取，如果具备照相机的内标定参数，就可以将深度图像转换为点云

1.class pcl::RangeImage

​    RangeImage类继承于PointCloud，主要功能是实现一个特定视点得到一个三维场景的深度图像。其继承关系如下：

​                                        ![img](http://images2015.cnblogs.com/blog/976394/201702/976394-20170227145032110-816579.png)

类RangeImage的成员有：

| Public Member Functions                          |                                                              |
| ------------------------------------------------ | ------------------------------------------------------------ |
| template<typename PointCloudType >               |                                                              |
| void                                             | [createFromPointCloud](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image.html#a82f6a143de2d73a0ad9fea6c527a2efb) (const PointCloudType &point_cloud, float angular_resolution=[pcl::deg2rad](http://docs.pointclouds.org/trunk/group__common.html#ga25b0ce695e2a10abb0130bcb5cf90eb6)(0.5f), float max_angle_width=pcl::deg2rad(360.0f), float max_angle_height=pcl::deg2rad(180.0f), const Eigen::Affine3f &sensor_pose=Eigen::Affine3f::Identity(), CoordinateFrame coordinate_frame=CAMERA_FRAME, float noise_level=0.0f, float min_range=0.0f, int border_size=0) |
|                                                  | 从点云创建深度图像，point_cloud为指向创建深度图像所需要的点云的引用，angular_resolution为模拟的深度传感器的角度分辨率，即深度图像中一个像素对应的角度大小，max_angle_width为模拟的深度传感器的水平最大采样角度，max_angle_height为模拟传感器的垂直方向最大采样角度，sensor_pose设置模拟的深度传感器的位姿是一个仿射变换矩阵，默认为4*4的单位矩阵变换，coordinate_frame定义按照那种坐标系统的习惯默认为CAMERA_FRAME，noise_level获取深度图像深度时，近邻点对查询点距离值的影响水平，min_range设置最小的获取距离，小于最小获取距离的位置为传感器的盲区，border_size获得深度图像的边缘的宽度 默认为0 该函数中涉及的角度的单位都是弧度 |
| template<typename PointCloudType >               |                                                              |
| void                                             | [createFromPointCloudWithKnownSize](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image.html#afbb553405d9b1df2b10b08c98c052250) (const PointCloudType &point_cloud, float angular_resolution, const Eigen::Vector3f &point_cloud_center, float point_cloud_radius, const Eigen::Affine3f &sensor_pose=Eigen::Affine3f::Identity(), [CoordinateFrame](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image.html#a8b5785b0499f0a70d5c87fceba55992f) coordinate_frame=[CAMERA_FRAME](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image.html#a8b5785b0499f0a70d5c87fceba55992facc23a9ec8c5fe7011f1168b87ec945bf), float noise_level=0.0f, float min_range=0.0f, int border_size=0) |
|                                                  | 从点云创建深度图像，其中参数中有关场景大小的提示，提高了获取深度图像时的计算速度。point_cloud为指向创建深度图像所需要的点云的引用，angular_resolution为模拟的深度传感器的角度分辨率，弧度表示，point_cloud_center为点云外接球体的中心，默认为（0，0，0）point_cloud_radius为点云外接球体的半径，sensor_pose设置模拟的深度传感器的位姿是一个仿射变换矩阵，默认为4*4的单位矩阵变换，coordinate_frame定义按照那种坐标系统的习惯默认为CAMERA_FRAME，noise_level获取深度图像深度时，近邻点对查询点距离值的影响距离，以米为单位，min_range设置最小的获取距离，小于最小获取距离的位置为传感器的盲区，border_size获得深度图像的边缘的宽度 默认为0 该函数中涉及的角度的单位都是弧度 |
| template<typename PointCloudTypeWithViewpoints > |                                                              |
| void                                             | [createFromPointCloudWithViewpoints](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image.html#a91cb7406e0e57923a7f5d533ea578c0f) (const PointCloudTypeWithViewpoints &point_cloud, float angular_resolution, float max_angle_width, float max_angle_height, [CoordinateFrame](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image.html#a8b5785b0499f0a70d5c87fceba55992f) coordinate_frame=[CAMERA_FRAME](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image.html#a8b5785b0499f0a70d5c87fceba55992facc23a9ec8c5fe7011f1168b87ec945bf), float noise_level=0.0f, float min_range=0.0f, int border_size=0) |
|                                                  | 从点云创建深度图像，点云中包含深度信息，其中，point_cloud为指向创建深度图像所需要的点云的引用，angular_resolution为模拟的深度传感器的角度分辨率，即深度图像中一个像素对应的角度大小，max_angle_width为模拟的深度传感器的水平最大采样角度，max_angle_height为模拟传感器的垂直方向最大采样角度，sensor_pose设置模拟的深度传感器的位姿是一个仿射变换矩阵，默认为4*4的单位矩阵变换，coordinate_frame定义按照那种坐标系统的习惯默认为CAMERA_FRAME，noise_level获取深度图像深度时，近邻点对查询点距离值的影响水平，如果该值比较小，则常用Z-缓冲区中深度平均值作为查询点的深度，min_range设置最小的可视深度，小于最小获取距离的位置为传感器的盲区，border_size获得深度图像的边缘的宽度 默认为0 该函数中涉及的角度的单位都是弧度 |
| void                                             | [createEmpty](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image.html#a46cb318d3d6f1cf6aace8d2ed10ed7fa) (float angular_resolution, const Eigen::Affine3f &sensor_pose=Eigen::Affine3f::Identity(), [RangeImage::CoordinateFrame](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image.html#a8b5785b0499f0a70d5c87fceba55992f) coordinate_frame=[CAMERA_FRAME](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image.html#a8b5785b0499f0a70d5c87fceba55992facc23a9ec8c5fe7011f1168b87ec945bf), float angle_width=[pcl::deg2rad](http://docs.pointclouds.org/trunk/group__common.html#ga25b0ce695e2a10abb0130bcb5cf90eb6)(360.0f), float angle_height=pcl::deg2rad(180.0f)) |
|                                                  | 创建一个空的深度图像，以当前视点不可见点填充，其中，angle_width为模拟的深度传感器的水平采样角度，默认为PI*2（360）；angle_height垂直方向的采样角度默认为PI（180）*****其他参数同上 |
| template<typename PointCloudType >               |                                                              |
| void                                             | [integrateFarRanges](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image.html#a6c3ceb66679ccc84587eef5c29a16386) (const PointCloudType &far_ranges) |
|                                                  | 将已有的远距离测量结果融合到深度图像中，                     |
| PCL_EXPORTS void                                 | [cropImage](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image.html#a20f223d93080e9ce32122f90a265f7c4) (int border_size=0, int top=-1, int right=-1, int bottom=-1, int left=-1) |
|                                                  | 裁剪深度图像到最小尺寸，使这个最小尺寸包含所有点云，其中，board_size设置裁剪后深度图像的边界尺寸， top为裁剪框的边界***********默认都为-1 |
| void                                             | [setTransformationToRangeImageSystem](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image.html#ab0419b66dd9e76fbc6d568049abfad3c) (const Eigen::Affine3f &to_range_image_system) |
|                                                  | 设置从深度图像坐标系（传感器的坐标系）转换到世界坐标系的变换矩阵 |
| float                                            | [getAngularResolution](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image.html#ac9bef829a1bcb990501409b3ac514f7f) () const |
|                                                  | 获得深度图像X和Y方向的角分辨率  弧度表示                     |
| void                                             | [setAngularResolution](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image.html#a2720450161caef9a2992b0ac943ab2b7) (float angular_resolution) |
|                                                  | 设置深度图像在X方向和Y方向的新的角分辨率，angular_resolution即每个像素所对应的弧度 |
| void                                             | [calculate3DPoint](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image.html#ab2612a9c5a458e5b763246709c850fc2) (float image_x, float image_y, float range, [PointWithRange](http://docs.pointclouds.org/trunk/structpcl_1_1_point_with_range.html) &point) const |
|                                                  | 根据深度图像点（X Y）和距离（range）计算返回场景中的3D点的point |
| void                                             | [calculate3DPoint](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image.html#aa83fef4d0f5cdcac2d8bdc1a188f1ad9) (float image_x, float image_y, [PointWithRange](http://docs.pointclouds.org/trunk/structpcl_1_1_point_with_range.html) &point) const |
|                                                  | 根据给定的深度图像点和离该点最近像素上的距离值计算返回场景中的3D 点point |

(详细的解释请官网查看)       

  (2)class pcl::RangeImagePlanner             

RangeImagePlanner 来源于最原始的深度图像，但又区别于原始的深度图像，因为该类不使用球类投影方式，而是通过一个平面投影方式进行投影（相机一一般采用这种投影方式），因此对于已有的利用深度传感器获取的深度图像来说比较实用，类的继承关系如下：

​                                                         ![img](http://images2015.cnblogs.com/blog/976394/201702/976394-20170227161237782-1845747165.png)

| Public Member Functions            |                                                              |
| ---------------------------------- | ------------------------------------------------------------ |
| PCL_EXPORTS void                   | [setDisparityImage](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image_planar.html#a751d0027979b31ccb2dbde563c8af11e) (const float *disparity_image, int di_width, int di_height, float focal_length, float base_line, float desired_angular_resolution=-1) |
|                                    | 从给定的视差图像中创建图像，其中disparity_image是输入的视差图像，di_width视差图像的宽度di_height视差图像的高度focal_length, 产生视差图像的照相机的焦距，base_line是用于产生视差图像的立体相对的基线长度，desired_angular_resolution预设的角分辨率 每个像素对应的弧度，该值不能大于点云的密度， |
| PCL_EXPORTS void                   | [setDepthImage](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image_planar.html#ae62978a6c6a5f50a794b930d104ebd24) (const float *depth_image, int di_width, int di_height, float di_center_x, float di_center_y, float di_focal_length_x, float di_focal_length_y, float desired_angular_resolution=-1) |
|                                    | 从已存在的深度影像中创建深度图像，其中，depth_image是输入的浮点形的深度图像，di_width,深度图像的宽度，di_height图像的高度，di_center_x  di_center_y 是照相机投影中心XY的坐标。di_focal_length_x  di_focal_length_y是照相机水平  垂直方向上的焦距，desired_angular_resolution预设的角分辨率 每个像素对应的弧度，该值不能大于点云的密度， |
| template<typename PointCloudType > |                                                              |
| void                               | [createFromPointCloudWithFixedSize](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image_planar.html#a47fc71b0700792cb0b078e9bbbc537f5) (const PointCloudType &point_cloud, int di_width, int di_height, float di_center_x, float di_center_y, float di_focal_length_x, float di_focal_length_y, const Eigen::Affine3f &sensor_pose, [CoordinateFrame](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image.html#a8b5785b0499f0a70d5c87fceba55992f) coordinate_frame=[CAMERA_FRAME](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image.html#a8b5785b0499f0a70d5c87fceba55992facc23a9ec8c5fe7011f1168b87ec945bf), float noise_level=0.0f, float min_range=0.0f) |
|                                    | 从已存在的点云中创建图像，point_cloud为指向创建深度图像所需要的点云对象的引用，di_width视差图像的宽度di_height视差图像的高度 di_center_x  di_center_y 是照相机投影中心XY的坐标  di_focal_length_x  di_focal_length_y是照相机水平  垂直方向上的焦距    sensor_pose是模拟深度照相机的位姿  coordinate_frame为点云所使用的坐标系  noise_level传感器的水平噪声， |
| virtual void                       | [calculate3DPoint](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image_planar.html#a8156313c723effdd2bac64edb41e2a49) (float image_x, float image_y, float range, Eigen::Vector3f &point) const |
|                                    | 根据给定图像点和深度图创建3D点，其中image_x  iamge_y range  分别为XY 坐标和深度，point为生成的3D点 |
| virtual void                       | [getImagePoint](http://docs.pointclouds.org/trunk/classpcl_1_1_range_image_planar.html#a3d1056d87f9a14379dc651b55b199a62) (const Eigen::Vector3f &point, float &image_x, float &image_y, float &range) const |
|                                    | 从给定的3D点point中计算图像点（X Y）和深度值                 |

​    等等具体看官网

（3）应用实例

 如何从点云创建深度图，如何从点云和给定的传感器的位置来创建深度图像，此程序是生成一个矩形的点云，然后基于该点云创建深度图像

新建文件range_image_creation.cpp：

```
#include <pcl/range_image/range_image.h>    //深度图像的头文件

int main (int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ> pointCloud;   //定义点云的对象
  
  // 循环产生点云的数据
  for (float y=-0.5f; y<=0.5f; y+=0.01f) {
    for (float z=-0.5f; z<=0.5f; z+=0.01f) {
      pcl::PointXYZ point;
      point.x = 2.0f - y;
      point.y = y;
      point.z = z;
      pointCloud.points.push_back(point); //循环添加点数据到点云对象
    }
  }
  pointCloud.width = (uint32_t) pointCloud.points.size();
  pointCloud.height = 1;                                        //设置点云对象的头信息
    //实现一个呈矩形形状的点云
  // We now want to create a range image from the above point cloud, with a 1deg angular resolution
   //angular_resolution为模拟的深度传感器的角度分辨率，即深度图像中一个像素对应的角度大小
  float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
   //max_angle_width为模拟的深度传感器的水平最大采样角度，
  float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
  //max_angle_height为模拟传感器的垂直方向最大采样角度  都转为弧度
  float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
   //传感器的采集位置
  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
   //深度图像遵循坐标系统
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  float noiseLevel=0.00;    //noise_level获取深度图像深度时，近邻点对查询点距离值的影响水平
  float minRange = 0.0f;     //min_range设置最小的获取距离，小于最小获取距离的位置为传感器的盲区
  int borderSize = 1;        //border_size获得深度图像的边缘的宽度
  
  pcl::RangeImage rangeImage;
  rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
  
  std::cout << rangeImage << "\n";
}
```

 

显示结果

![img](http://images2015.cnblogs.com/blog/976394/201702/976394-20170227172634923-852284437.png)

（1）点云到深度图与可视化的实现

区分点云与深度图本质的区别

1.深度图像也叫距离影像，是指将从图像采集器到场景中各点的距离（深度）值作为像素值的图像。获取方法有：激光雷达深度成像法、计算机立体视觉成像、坐标测量机法、莫尔条纹法、结构光法。
2.点云：当一束激光照射到物体表面时，所反射的激光会携带方位、距离等信息。若将激光束按照某种轨迹进行扫描，便会边扫描边记录到反射的激光点信息，由 于扫描极为精细，则能够得到大量的激光点，因而就可形成激光点云。点云格式有*.las ;*.pcd; *.txt等。
深度图像经过坐标转换可以计算为点云数据；有规则及必要信息的点云数据可以反算为深度图像

rangeimage是来自传感器一个特定角度拍摄的一个三维场景获取的有规则的有焦距等基本信息的深度图。

深度图像的像素值代表从传感器到物体的距离或者深度值。

RangeImage类的继承于PointCloud主要的功能实现一个特定的视点得到的一个三维场景的深度图像，继承关系为

![img](http://images2015.cnblogs.com/blog/976394/201703/976394-20170322150648658-1265536269.png)

所以我们知道有规则及必要信息就可以反算为深度图像。那么我们就可以直接创建一个有序的规则的点云，比如一张平面，或者我们直接使用Kinect获取的点云来可视化深度的图，所以首先分析程序中是如果实现的点云到深度图的转变的，（程序的注释是我自己的理解，注释的比较详细）

```
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>    //关于深度图像的头文件
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>   //深度图可视化的头文件
#include <pcl/visualization/pcl_visualizer.h>      //PCL可视化的头文件
#include <pcl/console/parse.h>
 
typedef pcl::PointXYZ PointType;
//参数
float angular_resolution_x = 0.5f,//angular_resolution为模拟的深度传感器的角度分辨率，即深度图像中一个像素对应的角度大小
      angular_resolution_y = angular_resolution_x;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;//深度图像遵循坐标系统
bool live_update = false;
//命令帮助提示
void 
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options] <scene.pcd>\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-rx <float>  angular resolution in degrees (default "<<angular_resolution_x<<")\n"
            << "-ry <float>  angular resolution in degrees (default "<<angular_resolution_y<<")\n"
            << "-c <int>     coordinate frame (default "<< (int)coordinate_frame<<")\n"
            << "-l           live update - update the range image according to the selected view in the 3D viewer.\n"
            << "-h           this help\n"
            << "\n\n";
}

void 
setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f(0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f(0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
                            up_vector[0], up_vector[1], up_vector[2]);
}

//主函数
int 
main (int argc, char** argv)
{
  //输入命令分析
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }
  if (pcl::console::find_argument (argc, argv, "-l") >= 0)
  {
    live_update = true;
    std::cout << "Live update is on.\n";
  }
  if (pcl::console::parse (argc, argv, "-rx", angular_resolution_x) >= 0)
    std::cout << "Setting angular resolution in x-direction to "<<angular_resolution_x<<"deg.\n";
  if (pcl::console::parse (argc, argv, "-ry", angular_resolution_y) >= 0)
    std::cout << "Setting angular resolution in y-direction to "<<angular_resolution_y<<"deg.\n";
  int tmp_coordinate_frame;
  if (pcl::console::parse (argc, argv, "-c", tmp_coordinate_frame) >= 0)
  {
    coordinate_frame = pcl::RangeImage::CoordinateFrame (tmp_coordinate_frame);
    std::cout << "Using coordinate frame "<< (int)coordinate_frame<<".\n";
  }
  angular_resolution_x = pcl::deg2rad (angular_resolution_x);
  angular_resolution_y = pcl::deg2rad (angular_resolution_y);
  
  //读取点云PCD文件  如果没有输入PCD文件就生成一个点云
  pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());   //申明传感器的位置是一个4*4的仿射变换
  std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
  if (!pcd_filename_indices.empty ())
  {
    std::string filename = argv[pcd_filename_indices[0]];
    if (pcl::io::loadPCDFile (filename, point_cloud) == -1)
    {
      std::cout << "Was not able to open file \""<<filename<<"\".\n";
      printUsage (argv[0]);
      return 0;
    }
   //给传感器的位姿赋值  就是获取点云的传感器的的平移与旋转的向量
    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                             point_cloud.sensor_origin_[1],
                                                             point_cloud.sensor_origin_[2])) *
                        Eigen::Affine3f (point_cloud.sensor_orientation_);
  }
  else
  {  //如果没有给点云，则我们要自己生成点云
    std::cout << "\nNo *.pcd file given => Genarating example point cloud.\n\n";
    for (float x=-0.5f; x<=0.5f; x+=0.01f)
    {
      for (float y=-0.5f; y<=0.5f; y+=0.01f)
      {
        PointType point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
        point_cloud.points.push_back (point);
      }
    }
    point_cloud.width = (int) point_cloud.points.size ();  point_cloud.height = 1;
  }
  
  // -----从创建的点云中获取深度图--//
  //设置基本参数
  float noise_level = 0.0;
  float min_range = 0.0f;
  int border_size = 1;
  boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
  pcl::RangeImage& range_image = *range_image_ptr;  
/*
 关于range_image.createFromPointCloud（）参数的解释 （涉及的角度都为弧度为单位） ：
   point_cloud为创建深度图像所需要的点云
  angular_resolution_x深度传感器X方向的角度分辨率
  angular_resolution_y深度传感器Y方向的角度分辨率
   pcl::deg2rad (360.0f)深度传感器的水平最大采样角度
   pcl::deg2rad (180.0f)垂直最大采样角度
   scene_sensor_pose设置的模拟传感器的位姿是一个仿射变换矩阵，默认为4*4的单位矩阵变换
   coordinate_frame定义按照那种坐标系统的习惯  默认为CAMERA_FRAME
   noise_level  获取深度图像深度时，邻近点对查询点距离值的影响水平
   min_range 设置最小的获取距离，小于最小的获取距离的位置为传感器的盲区
   border_size  设置获取深度图像边缘的宽度 默认为0 
*/ 
  range_image.createFromPointCloud (point_cloud, angular_resolution_x, angular_resolution_y,pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
  
  //可视化点云
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor (1, 1, 1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
  viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
  //viewer.addCoordinateSystem (1.0f, "global");
  //PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
  //viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
  viewer.initCameraParameters ();
  //range_image.getTransformationToWorldSystem ()的作用是获取从深度图像坐标系统（应该就是传感器的坐标）转换为世界坐标系统的转换矩阵
  setViewerPose(viewer, range_image.getTransformationToWorldSystem ());  //设置视点的位置
  
  //可视化深度图
  pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
  range_image_widget.showRangeImage (range_image);
  
  while (!viewer.wasStopped ())
  {
    range_image_widget.spinOnce ();
    viewer.spinOnce ();
    pcl_sleep (0.01);
    
    if (live_update)
    {
      //如果选择的是——l的参数说明就是要根据自己选择的视点来创建深度图。
     // live update - update the range image according to the selected view in the 3D viewer.
      scene_sensor_pose = viewer.getViewerPose();
      range_image.createFromPointCloud (point_cloud, angular_resolution_x, angular_resolution_y,
                                        pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                        scene_sensor_pose, pcl::RangeImage::LASER_FRAME, noise_level, min_range, border_size);
      range_image_widget.showRangeImage (range_image);
    }
  }
}
```

 

在代码利解释的十分详细，编译查看结果

![img](http://images2015.cnblogs.com/blog/976394/201703/976394-20170322154936424-1229728789.png)

​                                             没有输入PCD点云文件的结果

![img](http://images2015.cnblogs.com/blog/976394/201703/976394-20170322154959674-2067166937.png)

​                                                                   输入点云的原始图

![img](http://images2015.cnblogs.com/blog/976394/201703/976394-20170322154913080-2062801694.png)

​                                                                        输入的结果及其深度图

 

 

（2）如何从深度图像中提取边界

  从深度图像中提取边界（从前景跨越到背景的位置定义为边界），对于物体边界：这是物体的最外层和阴影边界的可见点集，阴影边界：毗邻与遮挡的背景上的点集，Veil点集，在被遮挡物边界和阴影边界之间的内插点，它们是有激光雷达获取的3D距离数据中的典型数据类型，这三类数据及深度图像的边界如图：

​                                                                  ![img](http://images2015.cnblogs.com/blog/976394/201702/976394-20170227193310173-2126815590.png)

代码解析：从磁盘中读取点云，创建深度图像并使其可视化，提取边界信息很重要的一点就是区分深度图像中当前视点不可见点几何和应该可见但处于传感器获取距离范围之外的点集 ，后者可以标记为典型边界，然而当前视点不可见点则不能成为边界，因此，如果后者的测量值存在，则提供那些超出传感器距离获取范围之外的数据对于边界的提取是非常重要的，

新建文件range_image_border_extraction.cpp：

```
#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/console/parse.h>

typedef pcl::PointXYZ PointType;

// --------------------
// -----Parameters-----
// --------------------
float angular_resolution = 0.5f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;

// --------------
// -----Help-----
// --------------
void 
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options] <scene.pcd>\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-r <float>   angular resolution in degrees (default "<<angular_resolution<<")\n"
            << "-c <int>     coordinate frame (default "<< (int)coordinate_frame<<")\n"
            << "-m           Treat all unseen points to max range\n"
            << "-h           this help\n"
            << "\n\n";
}

// --------------
// -----Main-----
// --------------
int 
main (int argc, char** argv)
{
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }
  if (pcl::console::find_argument (argc, argv, "-m") >= 0)
  {
    setUnseenToMaxRange = true;
    cout << "Setting unseen values in range image to maximum range readings.\n";
  }
  int tmp_coordinate_frame;
  if (pcl::console::parse (argc, argv, "-c", tmp_coordinate_frame) >= 0)
  {
    coordinate_frame = pcl::RangeImage::CoordinateFrame (tmp_coordinate_frame);
    cout << "Using coordinate frame "<< (int)coordinate_frame<<".\n";
  }
  if (pcl::console::parse (argc, argv, "-r", angular_resolution) >= 0)
    cout << "Setting angular resolution to "<<angular_resolution<<"deg.\n";
  angular_resolution = pcl::deg2rad (angular_resolution);
  
  // ------------------------------------------------------------------
  // -----Read pcd file or create example point cloud if not given-----
  // ------------------------------------------------------------------
  pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
  pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());  //传感器的位置
  std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
  if (!pcd_filename_indices.empty ())
  {
    std::string filename = argv[pcd_filename_indices[0]];
    if (pcl::io::loadPCDFile (filename, point_cloud) == -1)   //打开文件
    {
      cout << "Was not able to open file \""<<filename<<"\".\n";
      printUsage (argv[0]);
      return 0;
    }
    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                               point_cloud.sensor_origin_[1],
                                                               point_cloud.sensor_origin_[2])) *
                        Eigen::Affine3f (point_cloud.sensor_orientation_);  //仿射变换矩阵
  
    std::string far_ranges_filename = pcl::getFilenameWithoutExtension (filename)+"_far_ranges.pcd";
    if (pcl::io::loadPCDFile(far_ranges_filename.c_str(), far_ranges) == -1)
      std::cout << "Far ranges file \""<<far_ranges_filename<<"\" does not exists.\n";
  }
  else
  {
    cout << "\nNo *.pcd file given => Genarating example point cloud.\n\n";
    for (float x=-0.5f; x<=0.5f; x+=0.01f)      //填充一个矩形的点云
    {
      for (float y=-0.5f; y<=0.5f; y+=0.01f)
      {
        PointType point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
        point_cloud.points.push_back (point);   
      }
    }
    point_cloud.width = (int) point_cloud.points.size ();  point_cloud.height = 1;
  }
  
  // -----------------------------------------------
  // -----Create RangeImage from the PointCloud-----
  // -----------------------------------------------
  float noise_level = 0.0;      //各种参数的设置
  float min_range = 0.0f;
  int border_size = 1;
  boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
  pcl::RangeImage& range_image = *range_image_ptr;   
  range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
  range_image.integrateFarRanges (far_ranges);
  if (setUnseenToMaxRange)
    range_image.setUnseenToMaxRange ();

  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");   //创建视口
  viewer.setBackgroundColor (1, 1, 1);                      //设置背景颜色
  viewer.addCoordinateSystem (1.0f);              //设置坐标系
  pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 0, 0, 0);
  viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");   //添加点云
  //PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 150, 150, 150);
  //viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
  //viewer.setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 2, "range image");
  
  // -------------------------
  // -----Extract borders提取边界的部分-----
  // -------------------------
  pcl::RangeImageBorderExtractor border_extractor (&range_image);
  pcl::PointCloud<pcl::BorderDescription> border_descriptions;
  border_extractor.compute (border_descriptions);     //提取边界计算描述子
  
  // -------------------------------------------------------
  // -----Show points in 3D viewer在3D 视口中显示点云-----
  // ----------------------------------------------------
  pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),  //物体边界
                                            veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),     //veil边界
                                            shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);   //阴影边界
  pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr,
                                      & veil_points = * veil_points_ptr,
                                      & shadow_points = *shadow_points_ptr;

  for (int y=0; y< (int)range_image.height; ++y)
  {
    for (int x=0; x< (int)range_image.width; ++x)
    {
      if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
        border_points.points.push_back (range_image.points[y*range_image.width + x]);

      if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
        veil_points.points.push_back (range_image.points[y*range_image.width + x]);

      if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
        shadow_points.points.push_back (range_image.points[y*range_image.width + x]);
    }
  }
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> border_points_color_handler (border_points_ptr, 0, 255, 0);
  viewer.addPointCloud<pcl::PointWithRange> (border_points_ptr, border_points_color_handler, "border points");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "border points");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> veil_points_color_handler (veil_points_ptr, 255, 0, 0);
  viewer.addPointCloud<pcl::PointWithRange> (veil_points_ptr, veil_points_color_handler, "veil points");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "veil points");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> shadow_points_color_handler (shadow_points_ptr, 0, 255, 255);
  viewer.addPointCloud<pcl::PointWithRange> (shadow_points_ptr, shadow_points_color_handler, "shadow points");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "shadow points");
  
  //-------------------------------------
  // -----Show points on range image-----
  // ------------------------------------
  pcl::visualization::RangeImageVisualizer* range_image_borders_widget = NULL;
  range_image_borders_widget =
    pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget (range_image, -std::numeric_limits<float>::infinity (), std::numeric_limits<float>::infinity (), false,
                                                                          border_descriptions, "Range image with borders");
  // -------------------------------------
  
  
  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer.wasStopped ())
  {
    range_image_borders_widget->spinOnce ();
    viewer.spinOnce ();
    pcl_sleep(0.01);
  }
}
```

编译的运行的结果./range_image_border_extraction -m

![img](http://images2015.cnblogs.com/blog/976394/201702/976394-20170227195434516-766577342.png)

这将一个自定生成的，矩形状浮点型点云，有显示结果可以看出检测出的边界用绿色较大的点表示，其他的点用默认的普通的大小点来表示.