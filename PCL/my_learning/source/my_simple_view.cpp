//coded by yinjw

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
