1 ORB-SLAM2编译环境的安装
可参考链接：https://blog.csdn.net/radiantjeral/article/details/82193370
只有一点需要注意：
打开ORB_SLAM2/Examples/ROS/ORB_SLAM2文件夹下的CMakeLists.txt，在set(LIBS ...)中同样添加上述的两个库文件。

/usr/lib/x86_64-linux-gnu/libboost_system.so
/usr/lib/x86_64-linux-gnu/libboost_filesystem.so

2 ORB-SLAM2中双目部分示例代码的运行：
在 http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets 下载一个序列 Vicon Room 102  大小1.2GB

执行命令为 ./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml PATH_TO_SEQUENCE/cam0/data PATH_TO_SEQUENCE/cam1/data Examples/Stereo/EuRoC_TimeStamps/SEQUENCE.txt
其中PATH_TO_SEQUENCE : 为数据集所在路径

例如我的代码输入为：
./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml  ~/yu/ttee/mav0/cam0/data ~/yu/ttee/mav0/cam1/data  Examples/Stereo/EuRoC_TimeStamps/V102.txt
