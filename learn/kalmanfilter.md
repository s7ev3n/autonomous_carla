# Kalman Filter

> 经历了几次周折，好像这次终于比较好的理解了卡尔曼滤波。

## Pre

1. 多元高斯概率密度函数（协方差矩阵）
2. 高斯分布性质：1）高斯变量的仿射变换（线性变换+平移）还是高斯随机变量；2）多个高斯变量的随机组合还是高斯变量。**协方差矩阵传递。**
3. 一个多元高斯联合分布可以拆分成一个条件分布和先验分布（**组合和拆分公式必须知道**）
4. 高斯分布的边缘化

## Kalman Filter

### 要点

1. 所有变量都是随机变量,由概率密度函数(高斯概率密度函数)来描述
2. 由预测、更新两部分组成:
   1. 预测:根据自身的运动特性,预测下一时刻的状态分布
   2. 更新:根据新的观测值,调整预测的状态分布

3. 可以看图理解一下，如果只是用起来的话：

![Kalman Filter](/home/smartcar/harddisk/ubundisk/repos/autonomous_carla/learn/assets/kalman.png)

4. 主要关心：转移矩阵  $F_{k}$ 和观测矩阵 $H_{k}$。

### 理论

主要参考：[基于环境结构化特性的视觉SLAM方法-邹丹平](/home/smartcar/harddisk/ubundisk/repos/autonomous_carla/learn/assest/基于环境结构化特性的视觉SLAM方法-邹丹平.pdf)







