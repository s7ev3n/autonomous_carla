# ROS NodeHandle句柄

## 1. NodeHandle是干啥的？

StackOverflow有人问：我有`ros::Publisher`和`ros::Subscriber`，我为啥还需要这个`ros::NodeHandle`呢？是每个Topic都需要这么一个`ros::NodeHandle`吗？

[先来看官方的解答]：The `ros::NodeHandle` class serves two purposes. First, it provides [RAII](http://en.wikipedia.org/wiki/Resource_Acquisition_Is_Initialization)-style startup and shutdown of the internal node inside a [roscpp](http://library.isr.ist.utl.pt/docs/roswiki/roscpp.html) program. Second, it provides an extra layer of **namespace** resolution that can make writing subcomponents easier.

[StackOverflow的解答]：一个`NodeHandle`是一个代表你的ROS结点(Node)的对象，一般情况下，只需要一个或两个`NodeHandle`:

```c++
ros::NodeHandle nh; //public node handle
ros::NodeHandle nhp("~"); // private node handle
```

**Keep in mind that all the topics, services, parameters you manipulate are resolved depending on where your node is placed in the ROS namespace hierarchy **(see ROS_NAMESPACE environment variable for instance). 

**【我的总结】：即NodeHandle有两个作用：1.`roscpp`用来内部启动和终止结点的；2.句柄可以让你通过构造函数指定命名空间。**



## 2.命名空间namespace

命名空间对ROS中的通讯来说或者对C++编程来说是重要的概念，命名空间保证了各个节点、话题、消息和参数的名称必须是唯一的，否则通讯会发生冲突。

因此最简单的两种方法，来区分相同名字的资源：

1. 给两个名字前加上定语，就是  **添加命名空间**；

1. 给两个名字取个不同的别名，就是 **重映射remap**；

在ROS中有四种类型的图资源名称：

base（基本名称）：基本名称是没有命名空间限定符的相对名称（即没有/号）。

relative/name （相对名称）：相对名称的解析是依赖默认命名空间的。如默认命名空间为”/” 则名称“A/B”被解析为”/A/B”。

/global/name  （全局名称），不建议：以“/”开头的名称称为全局名称，代表该名称属于全局命名空间。意思是在ROS系统的任何地方都可以使用。无论在ROS系统的任何地方它都以明确的意义。

~private/name  （私有名称）：私有名称以”~”开头，它与相对图名称的区别是，它的解析不依赖与默认命名空间，而是依赖包名称。



1. 创建Nodehandle时，可以指定命名空间的名字，私有名称时候用到：

```c++
ros::init(argc, argv, "my_node_name"); //指定节点名字
ros::NodeHandle nh("my_namespace");
```

上面的nh的命名空间为`/my_namespace`

```
ros::NodeHandle nh2(nh,"my_namespace_2");
```

上面的nh的命名空间为`/my_namespace/my_namespace_2`

**需要注意的是**：上面的命名空间的确定前提是launch文件中没有定义`ns=="node_namespace"`，如果定义的话，需要加上`/node_namespace/my_namespace`.

2. 私有名字

```c++
ros::init(argc, argv, "my_node_name"); //指定节点名字
ros::NodeHandle pnh("~my_namespace");
```

pnh的命名空间为`/my_node_name/my_namespace`.

再来一个完整例子：

```c++
// launch 文件中 ns=="node_namespace"

ros::init(argc, argv, "node_name"); // node name

ros::NodeHandle n; //n 命名空间为/node_namespace

ros::NodeHandle n1("sub"); // n1命名空间为/node_namespace/sub

ros::NodeHandle n2(n1,"sub2");// n2命名空间为/node_namespace/sub/sub2

ros::NodeHandle pn1("~"); //pn1 命名空间为/node_namespace/node_name

ros::NodeHandle pn2("~sub"); //pn2 命名空间为/node_namespace/node_name/sub

ros::NodeHandle pn3("~/sub"); //pn3 命名空间为/node_namespace/node_name/sub

ros::NodeHandle gn("/global"); // gn 命名空间为/global

```









Ref:

1.https://answers.ros.org/question/68182/what-is-nodehandle/

2.http://library.isr.ist.utl.pt/docs/roswiki/roscpp(2f)Overview(2f)NodeHandles.html