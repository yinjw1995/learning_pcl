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

