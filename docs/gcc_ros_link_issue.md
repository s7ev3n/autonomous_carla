# gcc版本与ros link issue

一个在重搞ndt_mapping搞了很久的bug，报错：

```c++
CMakeFiles/talker.dir/src/talker.cpp.o: In function 'main':
talker.cpp:(.text+0x70): undefined reference to 'ros::init(int&, char**, std::string const&, unsigned int)'
talker.cpp:(.text+0xcc): undefined reference to 'ros::NodeHandle::NodeHandle(std::string const&, std::map<std::string, std::string, std::less<std::string>, std::allocator<std::pair<std::string const, std::string> > > const&)'
talker.cpp:(.text+0x29a): undefined reference to 'ros::console::initializeLogLocation(ros::console::LogLocation*, std::string const&, ros::console::levels::Level)'
```

如果搜索：`undefined reference to 'ros::init(int&, char**, std::string const&, unsigned int)'`，大部分的解决方案都是初级的错误，比如`CMakeList.txt`中各个部分的顺序。但是检查了好多遍，顺序没有问题。百思不得其解。重写编译最简单的ros教程中的代码，错误一模一样，所以，肯定不是`CMakeList.txt`的问题了。

首先尝试了ros安装出错？不对。忽然想到，gcc的问题，因为不知什么原因，gcc由原生的5.4升级到了5.5（没法降级），5.5和CUDA的编译之间存在bug，所以当时降到了4.8，解决了CUDA编译的问题，也就保留了gcc-4.8。

但是我安装ros是用原生的gcc-5.4安装的，所以用4.9编译肯定会出错，将版本改到5.5，一切顺利解决。这个bug有点难搞。

然后找到了类似的问题：

ref: https://answers.ros.org/question/291910/linking-problem-with-catkin_libraries/