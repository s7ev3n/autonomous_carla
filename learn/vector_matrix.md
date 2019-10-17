# 向量、矩阵和线性代数概念(Intro to Vector, Matrix and Linear Algebra Concept) 

## 1. 向量 Vector
### 自由向量
一个既有大小又有方向的量被叫做向量，与之对应的标量(Scalar)只有大小。一般用一个有向线段表示向量，例如 $\overrightarrow{AB}$. 而向量在物理中被称为矢量。
一个向量在保持长度和方向不变的条件下可以自由平移，它可以不依赖于任何坐标系存在，可以说是“自由”向量。无数的向量可能有相同的表述，所有这些向量都是互相平行、相等的，并具有相同的大小和方向。
### 向量的代数表示
向量这种表达太自由了，我们还是得解析地计算向量。那我们把所有向量的尾部都拉到同一个坐标原点，这样$n$维点空间就可以和$n$维向量空间建立一一对应关系了：$n$维空间中的点$(0,0,0,\dots)$取作原点，那么每一个点都可以让一个向量和它对应，这个向量就是从坐标原点出发到这个点为止的向量。例如，$\vec a=(x,y,z)$，这里，有序数组$(x,y,z)$称之为向量。注意，千万别和空间中的点混了。
一个向量可以被分解为三个单位坐标向量的线性表示。例如：$\vec a=(x,y,z)=x\vec i+y\vec j+z\vec k$.

**列向量：** 上面的向量$\vec a$可以称为行向量，但是在**线性代数**中一般说向量，指得都是**列向量**，这样更方便线性方程式的书写。参见知乎答案[为什么一般把向量定义为列向量？](https://www.zhihu.com/question/26304877) 下面的向量性质，尽量用列向量来表示。

### 向量的乘法
### 1. 向量的内积
两个向量的内积，也称为**点积**(`np.dot()`)、数量积等，它们相乘是一个**标量**，即是个数。
$$
\overrightarrow{\mathbf{a}} \cdot \overrightarrow{\mathbf{b}}=\left[ \begin{array}{c}{a_{1}} \\ {a_{2}} \\ {\dots} \\ {a_{n}}\end{array}\right] \cdot \left[ \begin{array}{c}{b_{1}} \\ {b_{2}} \\ {\dots} \\ {b_{n}}\end{array}\right]=a_{1} b_{1}+a_{2} b_{2}+\ldots+a_{n} b_{n}
$$

举一个代码的例子
```python
import numpy as np
a = np.array([3, 5, 2])
b = np.array([1, 4, 7])
print a.dot(b)  # 37
print np.dot(a, b)  # 37（另一种等价写法）
```
**内积的几何意义**
向量和长度：$\|\overrightarrow{\mathbf{a}}\|=\sqrt{a_{1}^{2}+a_{2}^{2}+\ldots+a_{n}^{2}}=\sqrt{\overrightarrow{\mathbf{a}} \cdot \overrightarrow{\mathbf{a}}}$

向量的内积： $\overrightarrow{\mathbf{a}} \cdot \overrightarrow{\mathbf{b}}=\|\overrightarrow{\mathbf{a}}\|\|\overrightarrow{\mathbf{b}}\| \cos \theta$
它表示$\vec a$和$\vec b$长度之积再乘以它们之间的夹角的余弦。更形象来说，就是一个量向量在另一个向量上的投影的积，也就是同方向的积。

### 2. 向量的外积
两个向量的外积，也称为叉积(cross product)(`np.cross()`)。两个向量确定了一个二维平面，它们的外积得到垂直于这个平面的向量。垂直于平面有两个方向，规定用右手法则来确定叉积的方向，即右手四指平直指向第一个向量，弯曲指向第二个向量，则右手大拇指方向为叉积结果向量的方向。

向量的叉积只在$\mathbb{R}^{2}$和$\mathbb{R}^{3}$中定义：
$$
\left[ \begin{array}{l}{a_{1}} \\ {a_{2}}\end{array}\right] \times \left[ \begin{array}{l}{b_{1}} \\ {b_{2}}\end{array}\right]=\left[a_{1} \cdot b_{2}-a_{2} \cdot b_{1}\right]
$$
$$
\left[ \begin{array}{l}{a_{1}} \\ {a_{2}} \\ {a_{3}}\end{array}\right] \times \left[ \begin{array}{l}{b_{1}} \\ {b_{2}} \\ {b_{3}}\end{array}\right]=\left[ \begin{array}{l}{a_{2} \cdot b_{3}-a_{3} \cdot b_{2}} \\ {a_{3} \cdot b_{1}-a_{1} \cdot b_{3}} \\ {a_{1} \cdot b_{2}-a_{2} \cdot b_{1}}\end{array}\right]
$$

**叉积的几何意义**
$\|\overrightarrow{\mathbf{a}} \times \overrightarrow{\mathbf{b}}\|=(\|\overrightarrow{\mathbf{a}}\|\|\overrightarrow{\mathbf{b}}\| \sin \theta) \overrightarrow \mathbf{n_{0}}$
其中$\overrightarrow \mathbf{n_{0}}$是v垂直于$\overrightarrow{\mathbf{a}}$和$\overrightarrow{\mathbf{b}}$展成平面的单位法向量。

```python
import numpy as np
a = np.array([3, 5, 2])
b = np.array([1, 4, 7])
print np.cross(a, b)  # [27, -19, 7]
```

### 3.向量与矩阵相乘
当矩阵$\vec A$的列数与向量$\vec x$的分量数相同时，矩阵和向量的积有定义：
$$
\underset{m\times n}{A}\vec{\mathbf{x}}=\begin{bmatrix}a_{11} & a_{12} & \ldots & a_{1n} \\ a_{21} & a_{22} & \ldots & a_{2n} \\ \ldots \\ a_{m1} & a_{m2} & \ldots & a_{mn}\end{bmatrix}\begin{bmatrix}x_1 \\ x_2 \\ \ldots \\ x_n \end{bmatrix} = \begin{bmatrix}a_{11}x_1 + a_{12}x_2 + \ldots + a_{1n}x_n \\ a_{21}x_1 + a_{22}x_2 + \ldots + a_{2n}x_n \\ \ldots \\ a_{m1}x_1 + a_{m2}x_2 + \ldots + a_{mn}x_n \\ \end{bmatrix}
$$
这个就很像线性代数中的线性方程组了吧，向量得是列向量。
**矩阵的向量积可以当成是矩阵的所有列向量的线性组合**：
$$
\underset { m\times n }{ \mathbf{A} } \vec { \mathbf{x} } =\begin{bmatrix} \underbrace { \begin{bmatrix} a_{ 11 } \\ a_{ 21 } \\ \ldots \\ a_{ m1 } \end{bmatrix} }_{ \vec { \mathbf{ a }_{ 1 } }  }  & \underbrace { \begin{bmatrix} a_{ 12 } \\ a_{ 22 } \\\ldots  \\ a_{ m2 } \end{bmatrix} }_{ \vec { \mathbf{ a_{ 2 } } }  } & \ldots & \underbrace { \begin{bmatrix} a_{ 1n } \\ a_{ 2n } \\ \ldots \\ a_{ mn } \end{bmatrix} }_{ \vec { \mathbf{ a_{ n } } }  }  \end{bmatrix}\begin{bmatrix} x_{ 1 } \\ x_{ 2 } \\ \ldots \\ x_{ n } \end{bmatrix}=x_1\vec{\mathbf{a}_1}+x_2\vec{\mathbf{a}_2}+\ldots+x_n\vec{\mathbf{a}_n}
$$

**而向量$\vec x$的每一个分量可以看成是$\vec A$的每个列向量的加权。**
>**一个矩阵其实就是一个线性变换。一个矩阵乘以一个向量后得到的向量，其实就相当于将这个向量进行了线性变换。**

REF: [机器学习的数学基础：向量篇](https://www.hahack.com/math/math-vector/)
## 2. 线性代数 Linear Algebra

### 线性
线性代数是研究线性问题的代数理论。代数应该比较清楚，是研究数、数量、关系、结构与代数方程（组）的通用解法及其性质的数学分支(wiki)。那么线性呢？

线性代数里面的**线性**主要意思是线性空间里的线性变换。

### 线性函数
直觉上，线性就是两个变量的关系是一条直线。但这还不够，数学上，线性满足两个性质：
1. 可加性，如果函数$f(x)$是线性的：
$f(x_{1}+x_{2})=f(x_{1})+f(x_{2})$
2. 比例性，如果函数$f(x)$是线性的：
$f(kx)=kf(x)$
注意，函数$f(x)=ax+b$不满足比例性，严格上说不算是**线性代数**里的线性函数。

总结起来就是：
$f(k_{1}x_{1}+k_{2}x_{2})=k_{1}f(x+{1})+k_{2}f(x_{2})$

### 线性映射/变换
参见《线性代数的几何意义》P11-17，看着图理解一下，不过书中还是没有形象话矩阵K是如何做到线性变换的。


## 3. 矩阵