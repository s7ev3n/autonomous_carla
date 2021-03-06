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

4.关于numpy中的axis
有时候会让人有些懵的参数，看过一些资料，都不好。我发现了一个简单的方法记住。
有两个`ndarray`a和b，shape都是`(2,2)`，下面来看一些操作：
4.1合并操作
`np.concatenate((a, b), axis=0)`
结果的shape是`(4,2)`
`np.concatenate((a, b), axis=1)`
结果的shape是`(2,4)`
看到了吗？shape里面的`(2,2)`，其中第一个2就是`axis=0`，第二个2就是`axis=1`，如果想要在`axis=0`上concatenate，那么一定是在`axis=0`上增加维度，即从2变成4，`axis=1`同理。不用记住什么`axis=0`代表行之类的，搞得自己乱七八糟的。
4.2相加操作
```
a = np.array([[1,2,3],[4,5,6]]) # (2,3)
b = a.sum(axis=0) # [5,7,9] (3,)
c = a.sum(axis=1) # [6,15] (2,)
```
a在`axis=0`相加，那么`(2,3)`中2的维度就要消失，3保持不变；a中有2个`(3,)`，那就是这两个逐个元素相加，消减维度2.
a在`axis=1`相加，就是3的维度消失，那就是每个`(3,)`内部相加。

5.`np.where()`

```
a = np.array([[1,2,3],[4,5,6]])
idx = (a>2)
print(idx) # [[False False  True],[ True  True  True]]
b = a[idx]
print(b) # [3 4 5 6]

idx_ = np.where(a>2)
print(idx_) # (array([0, 1, 1, 1]), array([2, 0, 1, 2]))
c = a[idx_]
print(c) # [3 4 5 6]
```
如果想从一个ndarray取值满足某些条件，例如大于某个值，上面的是常见的两种做法。
值得注意的是`np.where()`返回的是一个tuple，tuple第一个ndarray和第二个ndarry每个元素要联合起来一起看，就是满足条件的index。

6.`tf.reduce_max()`
```
d = tf.constant([[10, 220, 55], [4, 3, 56]]) # d.shape = (2,3) 三维
vals = tf.reduce_max(d, axis=0)
with tf.Session() as sess:
    vals = sess.run(vals)
    print(vals.shape)
    print(vals)
```
要在axis=0，即`(2,3)`中2的维度中每个元素取最大值。
结果：
```
(3,)
[ 10 220  56]
```
7.`tf.split()`
根据指定axis的split数组或整数进行切分，看官方例子：
```
# 'value' is a tensor with shape [5, 30]
# Split 'value' into 3 tensors with sizes [4, 15, 11] along dimension 1
split0, split1, split2 = tf.split(value, [4, 15, 11], 1)
tf.shape(split0)  # [5, 4]
tf.shape(split1)  # [5, 15]
tf.shape(split2)  # [5, 11]
# Split 'value' into 3 tensors along dimension 1
split0, split1, split2 = tf.split(value, num_or_size_splits=3, axis=1)
tf.shape(split0)  # [5, 10]
```
8.`tf.tile()`
```
tf.tile(
    input,
    multiples,
    name=None
)
```
This operation creates a new tensor by replicating input multiples times. The output tensor's i'th dimension has input.dims(i) * multiples[i] elements, and the values of input are replicated multiples[i] times along the 'i'th dimension. For example, tiling [a b c d] by [2] produces [a b c d a b c d].
在multiples制定的维度上复制，举个例子：
```
a = tf.constant([[1,2],[3,4]])
b = tf.tile(a,[2,3])
with tf.Session() as tf:
    print(sess.run(b))
```
结果：
```
[[1 2 1 2 1 2]
 [3 4 3 4 3 4]
 [1 2 1 2 1 2]
 [3 4 3 4 3 4]]
```
9.`sys.path.append`
这个用法见于f-pointnet代码中，例如`train.py`：
```
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
sys.path.append(BASE_DIR)
sys.path.append(os.path.join(ROOT_DIR, 'models'))
```
把当前文件地址BASE_DIR加入到系统路径中；因为用到了'models'文件夹，所以把它也加入系统目录，这样子即使不在当前文件夹运行脚本也不会有问题。
详细参见：https://askubuntu.com/questions/470982/how-to-add-a-python-module-to-syspath

10.`super(, self).__init__()`
Python的`super()`函数
