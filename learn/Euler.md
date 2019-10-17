# 旋转矩阵、旋转向量、欧拉角与四元数

## 1.旋转矩阵 R

向量的旋转可以用向量间的外积来表示。
机器人学中的旋转，可以理解为坐标系的旋转（还有平移），例如相机坐标系在世界坐标系下运动，这个运动就是旋转和平移，也称为欧式变换。
如何求？
想象一个向量a在世界坐标下的表达式和相机坐标系下的表达式，由于都表示向量a，因此两个表达式相等，就可以求得旋转矩阵R。

绕3个轴的旋转矩阵：
z轴：
$R_{z}(\theta)=\left[\begin{array}{ccc}{\cos \theta} & {-\sin \theta} & {0} \\ {\sin \theta} & {\cos \theta} & {0} \\ {0} & {0} & {1}\end{array}\right]$
x轴：
$R_{x}(\theta)=\left[\begin{array}{ccc}{1} & {0} & {0} \\ {0} & {\cos \theta} & {-\sin \theta} \\ {0} & {\sin \theta} & {\cos \theta}\end{array}\right]$

y轴：
$R_{y}(\theta)=\left[\begin{array}{ccc}{\cos \theta} & {0} & {\sin \theta} \\ {0} & {1} & {0} \\ {-\sin \theta} & {0} & {\cos \theta}\end{array}\right]$

旋转矩阵R：
1.行列式为1的正交矩阵(3x3)
TODO：行列式定义、正交矩阵、正交矩阵性质


## 2.旋转向量
旋转矩阵为3x3矩阵，9个量表达三个自由度的旋转，过于荣誉
旋转矩阵充分必要条件是正交、行列式为1，求解时增加困难

轴角(Axis-Angle)，即用旋转轴和旋转角来表示旋转，即Eigen中的`Eigen::AngleAxisf`。
我们用一个向量，其方向与旋转**轴**一致，长度等于旋转角表达旋转，即旋转向量。其实和轴角是一样的。

旋转矩阵与旋转向量的转换关系：罗德里格斯公式。
TODO：简单推导罗德里格斯公式、矩阵的迹Tr

## 欧拉角
旋转向量不够直观，因此用欧拉角来表示，即把一次旋转分离成绕三个轴的旋转。但是欧拉角存在万向锁问题，因此计算时不用欧拉角。

欧拉角根据三个轴旋转的顺序不同，得到的最终旋转也不同，有很多种旋转顺序。
如果你要是看wiki，会有一张动图，很多博客也用这个动图，它是ZXZ轴这样的顺序的。

但是在SLAM书中，采用的是ZYX的，即rpy的（ZYX 和rpy顺序其实是相反的），即如下代码：
```
Eigen::Translation3f init_translation(predict_ndt_pose.x, predict_ndt_pose.y, predict_ndt_pose.z);
Eigen::AngleAxisf init_rotation_x(predict_ndt_pose.roll, Eigen::Vector3f::UnitX());
Eigen::AngleAxisf init_rotation_y(predict_ndt_pose.pitch, Eigen::Vector3f::UnitY());
Eigen::AngleAxisf init_rotation_z(predict_ndt_pose.yaw, Eigen::Vector3f::UnitZ());

Eigen::Matrix4f tf_btol_;
Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x) * tf_btol;
```
不要疑惑，我们还是采用rpy这种的。


## 四元数



