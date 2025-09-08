# pcmerge_for_banana (点云融合方法抓取香蕉的项目)

本项目旨在提供一个尽可能简化的 Franka Panda 抓取环境，方便快速复现 **ROS + Gazebo + MoveIt** 下的抓取实验。

---

## 构建步骤

### 1. 编译工程
建议在ubuntu20.04版本编译运行，在~文件夹（家目录）运行git命令下载此项目
```bash
git clone https://github.com/zlwq/pcmerge_for_banana.git 
```
你需要事先下载好gpd和pcl，pcl用ubuntu apt自带的即可，gpd需要mkdir build和cd build之后再cmake ..再make再sudo make install即可。
如你所见，很多无关的文件我全部删除，至于本项目删除的头文件实际上会在你的/usr里面找到，包括但不限于franka_ros，这个在ubuntu20.04里面也有 
```bash
sudo apt install ros-noetic-franka-ros
``` 
进入工作空间，然后先编译gpd_ros，虽然我删了无关的文件但是不影响编译。先编译它的原因是为了把msg文件转化为对应的头文件，提供给剩下的包用，然后再全部编译：
```bash
cd pcmerge_for_banana
catkin build gpd_ros
catkin build
```
在编译过程中，可能会因为缺少依赖而报错。
此时可以根据报错信息，利用 AI 或搜索引擎来提示需要安装的依赖，例如：
```bash
sudo apt install ros-noetic-<package-name>
```
逐步补齐所有依赖后即可完成构建。 
### 2. 配置环境变量
在 ~/.bashrc 中添加以下行，使 Gazebo 能找到自定义模型： 
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/zlwq/pcmerge_for_banana/src/panda_demo/urdf
```
保存后刷新： 
```bash
source ~/.bashrc
```
### 3. 加载工作空间环境
构建完成后，在 ~/.bashrc 输入以下命令重新打开终端，或手动执行： 
```bash
source ~/pcmerge_for_banana/devel/setup.bash
```
### 4、打开launch文件和对应的banana_grasp
完成以上步骤后，即可在 Gazebo 中启动抓取环境，并基于此进行抓取复现。
```bash
roslaunch panda_moveit_config demo_gazebo.launch
```
等待的时间非常快，直到你看见香蕉稳稳地停在桌面上，并且三个摄像头加载好之后，新开一个终端窗口，运行以下命令
```bash
rosrun panda_demo cloud_fusion_node \
  _cloud1_topic:=/cam1/depth/points \
  _cloud2_topic:=/cam2/depth/points \
  _cloud3_topic:=/cam3/depth/points \
  _target_frame:=panda_link0 \
  _out_topic:=/fusion/points \
  _voxel_leaf:=0.01 
```
然后新开一个终端窗口，开始通过/fusion/points发布的点云计算对应的抓取并显示。
```bash 
roslaunch panda_demo gpd_run.launch
``` 
特别注意，当你看到带有彩色夹爪和点云的可视化界面的时候，不要动它，先新开一个终端窗口。
注意不要让你的~/.bashrc里面出现如下命令，否则看不到夹爪且报错。因为gpd内置的可视化界面和gazebo根本不一样。
如果你是vmware虚拟机下运行的ubuntu，则gazebo只能支持虚拟化的gpu。 
```bash
# export LIBGL_ALWAYS_SOFTWARE=0
# export SVGA_VGPU10=0
```
现在你已经新开一个终端窗口，然后执行以下命令
```bash 
rosrun panda_demo gpd_method
``` 
当你看到19这个数字的时候(本人风格的调试信息，懒得删了，正好能看到代码执行到哪里，何乐而不为)，鼠标先点击那个带有点云和彩色夹爪的可视化界面的窗口，然后按一下键盘q。
你会看到rviz界面开始有变化，此时立刻跳转到gazebo界面，你会看到机械臂以极快的速度夹起香蕉然后抓到空中，立刻松开夹爪。香蕉在桌面上弹了几下之后不动了。
有时候没有变化，你要注意观察调试信息，重新安装上面的步骤运行roslaunch panda_demo gpd_run.launch和rosrun panda_demo gpd_method。
当然这个时候点云你就不用再重新退出然后加载了，会实时更新的，频率为1hz。
### 注意事项
许多非必要功能和插件已被精简，建议使用ubuntu20.04版本进行构建复现。遇到任何问题可在issue板块发布。  
如需额外功能，请在编译过程中根据报错提示自行安装缺失的 ROS 依赖包。
### 致谢
本环境基于 franka_ros和官方教程moviet_tutorial gpd_ros gpd改造而来，感谢原作者的开源贡献。
