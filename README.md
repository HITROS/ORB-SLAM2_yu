# ORB-SLAM2编译环境安装
### https://github.com/raulmur/ORB_SLAM2

只需注意一点：打开<font color="#006600">ORB_SLAM2/Examples/ROS/ORB_SLAM2</font>文件夹下的CMakeLists.txt，在<font color="#dd0000"> set(LIBS ...)</font>中添加上述的两个库文件。
```
/usr/lib/x86_64-linux-gnu/libboost_system.so 
```
```
/usr/lib/x86_64-linux-gnu/libboost_filesystem.so
```

# ORB-SLAM2中双目部分示例代码的运行：
在[链接](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)中下载一个序列(名为Vicon_Room_102,大小为1.2G左右的数据集)

执行命令
```
./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml PATH_TO_SEQUENCE/cam0/data PATH_TO_SEQUENCE/cam1/data Examples/Stereo/EuRoC_TimeStamps/SEQUENCE.txt
```
其中<font color="#006600">PATH_TO_SEQUENCE</font>为数据集所在路径

例如我的代码输入为：
```
./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml  ~/yu/ttee/mav0/cam0/data ~/yu/ttee/mav0/cam1/data  Examples/Stereo/EuRoC_TimeStamps/V102.txt
```

### 10月27日更新：src/Comment-yu/zhushi/ORB-SLAM2源码详解.pdf 为源码主体脉络梳理
