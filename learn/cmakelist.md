# CMakeList.txt

> 参考：http://wiki.ros.org/catkin/CMakeLists.txt

## 概述

`CMakeList.txt`是编译ROS build system的输入，它告诉编译器如何build以及怎样install。`CMakeList.txt`是 standard vanilla `CMakeLists.txt` file with a few additional constraints.

## 结构

`CMakeList.txt`的结构很重要，甚至顺序都很重要

1. Required CMake Version (`cmake_minimum_required`)
2. Package Name (`project()`)
3. **Find other CMake/Catkin packages needed for build** (`find_package()`)
4. **Enable Python module support** (`catkin_python_setup()`)
5. **Message/Service/Action Generators**(`add_message_files(), add_service_files(), add_action_files()`)
6. **Invoke message/service/action generation** (`generate_messages()`)
7. **Specify package build info export** (`catkin_package()`)
8. **Libraries/Executables to build** (`add_library()/add_executable()/target_link_libraries()`)
9. Tests to build (`catkin_add_gtest()`)
10. Install rules (`install()`)

## 详细

### 1.`find_package()`

一般情况，一定要有`catkin`:

`find_package(catkin REQUIRED)`

如果还需要其他package，就加入`COMPONENTS`:

`find_package(catkin REQUIRED COMPONENTS nodelet)`

#### `find_package()`干了什么？

If a package is found by `CMake` through find_package, it results in the creation of several `CMake` environment variables that give information about the found package. These environment variables can be utilized later in the `CMake` script. The environment variables describe where the packages exported header files are, where source files are, what libraries the package depends on, and the paths of those libraries. The names always follow the convention of `<PACKAGE NAME>_<PROPERTY>`:

- `<NAME>_FOUND` - Set to true if the library is found, otherwise false
- `<NAME>_INCLUDE_DIRS` or `<NAME>_INCLUDES` - The include paths exported by the package
- `<NAME>_LIBRARIES` or `<NAME>_LIBS` - The libraries exported by the package
- `<NAME>_DEFINITIONS` - ?

### 2.`catkin_package()`

