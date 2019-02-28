# Eigen in Autoware
在Autoware中用到了Eigen库的一些方法，涉及到矩阵变换。也可参见：https://eigen.tuxfamily.org/dox/group__TutorialGeometry.html

##1. `Eigen::Matrix4f`

举例：
```
Matrix3f A ;
Matrix4d B ;
```
1.The variable A is of type Matrix3f — that is, a 3x3 matrix of floats.
2.The variable B is a 4x4 matrix of doubles.
PS：大部分可以这样，但不是所有。



##2. `Eigen::Translation3f`

>Represents a translation transformation.

代表一个平移矩阵，可以如下方式初始化：
`Eigen::Translation3f init_translation (0, 0, 2.0);`
或者
`init_translation << 0.0, 0.0, 2.0`



##3. `Eigen::AngleAxisf`

>Represents a 3D rotation as a rotation angle around an arbitrary 3D axis.

代表一个旋转向量（注意是向量），轴角，底层不直接是Matrix，但因为重载了运算符，运算（相乘）可以当作矩阵。

例：
```
Eigen::AngleAxisd v(M_PI/4,Eigen::Vector3d::UnitZ()); //沿z轴旋转了45度
cout << "rotation vector: Angle is: " << v.angle() * (180 / M_PI)<<endl;//旋转角
cout << "Axis is: " << v.axis().transpose() << endl<<endl; //旋转轴
```
如果想要把它转换成旋转矩阵，两种方式：
```
//方式一：用matrix()
R=v.matrix();
//方式二：用toRotationMatrix()
R=v.toRotationMatrix();
```


##4.`Eigen` 三维空间变换（旋转+平移）
```
Eigen::Translation3f init_translation(predict_ndt_pose.x, predict_ndt_pose.y, predict_ndt_pose.z);
Eigen::AngleAxisf init_rotation_x(predict_ndt_pose.roll, Eigen::Vector3f::UnitX());
Eigen::AngleAxisf init_rotation_y(predict_ndt_pose.pitch, Eigen::Vector3f::UnitY());
Eigen::AngleAxisf init_rotation_z(predict_ndt_pose.yaw, Eigen::Vector3f::UnitZ());

Eigen::Matrix4f tf_btol_;
Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x) * tf_btol;
```

上面代码涉及到了旋转向量、欧拉角和旋转矩阵，先记住几点
1.相乘最左边表示最先进行的变化(旋转或平移)
2.不同的相乘顺序得到的旋转是不同的，一般是按照z轴y轴x轴(yaw, pitch, roll)进行的，