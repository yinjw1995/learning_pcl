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

