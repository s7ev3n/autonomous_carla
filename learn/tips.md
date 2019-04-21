1. 保存成`.pcd`文件

```c++
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud)
```

2. 读取pcd


3. numpy array `(n,1)` 和`(n,)`
原来两个有些区分不清，`np.array([1, 2, 3]).shape`这个就是`(3,)`这个是一维向量
`np.array([[1, 2, 3]]).shape`，这个就是二维向量了，`(1,3)`
`np.array(array([[1],
                [2],
                [3]]))`，这个是二维向量，`(3,1)`


