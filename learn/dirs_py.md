# Python目录和文件操作小结
## 常用的几个函数
1. `os.listdir(path)`: 返回path指定的文件夹包含的文件或文件夹的名字的列表
2. `os.makedir(path[, mode])`:以数字mode的mode创建一个名为path的文件夹,默认的 mode 是 0777 (八进制)[从来没有写过这个mode，其实就是文件操作权限]
3. `os.makedirs(path[, mode])`:递归文件夹创建函数。像mkdir(), 但创建的所有intermediate-level文件夹需要包含子文件夹
4. `os.remove(path)`:删除路径为path的文件。如果path 是一个文件夹，将抛出OSError
5. `os.removedirs(path)`：递归删除目录。
## `os.path`模块
1. `os.path.basename(path)`：返回文件名
2. `os.path.dirname(path)`：返回文件路径
3. `os.path.exists(path)`：如果路径 path 存在，返回 True；如果路径 path 不存在，返回 False。
4. `os.path.isfile(path)`：判断路径是否为文件
5. `os.path.isdir(path)`：判断路径是否为目录
6. **`os.path.join(path1[, path2[, ...]])`**：把目录和文件名合成一个路径，非常常用。
7. `os.path.split(path)`：把路径分割成 dirname 和 basename，返回一个元组
8. `os.path.walk(path, visit, arg)`：比较麻烦，还是看下面的`os.walk`

## `os.walk`
Ref：[os.walk教程](http://www.runoob.com/python/os-walk.html)
语法：`os.walk(top[, topdown=True[, onerror=None[, followlinks=False]]])`
例子：
```python
#!/usr/bin/python
# -*- coding: UTF-8 -*-

import os
for root, dirs, files in os.walk(".", topdown=False):
    for name in files:
        print(os.path.join(root, name))
    for name in dirs:
        print(os.path.join(root, name))
```
