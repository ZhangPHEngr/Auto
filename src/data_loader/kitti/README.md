可视化数据集步骤：
```shell
# 在命令行中依次执行以下命令
cd Auto
catkin_make -j6
source devel/setup.zsh 或者 source devel/setup.bash
roslaunch kitti all_raw.launch
# 此时数据已经开始播放，接下里需要使用ros自带的可视化GUI显示
rviz &
# 在rviz中加载Auto/source/rviz_show.cfg.rviz配置文件
```