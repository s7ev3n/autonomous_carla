# Parameter Server

> Ref:
>
> 1. http://wiki.ros.org/roscpp/Overview/Parameter%20Server
>
> 2. https://blog.csdn.net/u014695839/article/details/78348600

可以通过`ros::param`命名空间，也可以通过`ros::NodeHandle`。一般，参数保存在`yaml`文件中。

## 1.Getting Parameters

1.`ros::NodeHandle::getParam()`

```c++
ros::NodeHandle nh;
std::string global_name, relative_name, default_param;
if (nh.getParam("/global_name", global_name))
{
  ...
}

if (nh.getParam("relative_name", relative_name))
{
...
}
```

获得默认值，如果parameter server中没有，那么就使用default值：

`nh.param<std::string>("default_param", default_param, "default_value");`

> Assign value from parameter server, with default. This method tries to retrieve the indicated parameter value from the parameter server, storing the result in param_val. If the value cannot be retrieved from the server, default_val is used instead.