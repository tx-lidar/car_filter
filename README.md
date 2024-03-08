##启动雷达驱动##
..............

##launch文件中做相应修改
#xyz范围
<param name="min_x" value="-1.6"/>
<param name="min_y" value="-0.6"/>
<param name="min_z" value="-100"/>

<param name="max_x" value="0.6"/>
<param name="max_y" value="0.6"/>
<param name="max_z" value="100"/>

#输入和输出点云话题
<param name="points_topic" type="string" value="/rslidar_points"/>
<param name="filter_topic" type="string"  value="filter_points"/>

##加载点云过滤节点

roslaunch pcl_test box_filter.launch

