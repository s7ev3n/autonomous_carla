# gcc 和 cuda

目前安装CUDA版本为9.0.176，gcc(g++)5.5(好像是不小心升级了)，在编译`ndt_gpu`时出现错误：`/usr/lib/gcc/x86_64-linux-gnu/5/include/avx512vlintrin.h(11300)`。该错误应该是gcc5.5版本的bug，gcc5.4在邓电脑上就可以编译`ndt_gpu`。尝试各种降级到gcc5.4未果，于是安装了gcc-4.8和g++4.8编译成功。具体：
1. 先用`sudo apt-get install gcc-4.8 g++-4.8`安装
2. 创建软链接：`sudo ln -s /usr/bin/g++-4.8 /usr/bin/g++`和` sudo ln -s /usr/bin/gcc-4.8 /usr/bin/gcc`
3. 编译`ndt_gpu`：`catkin_make -DCMAKE_BUILD_TYPE=Release`
4.成功 
