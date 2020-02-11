# CMAKE
CMake is a tool used to organize your C++ project and link external libraries 
that you are going to use. Your project could be built from the command line 
using gcc with specified options; however, this is going to be less intuitive 
for most people. The gcc commands can be orgainzed into a Makefile, but these 
still are not going to be very readable. With CMake, you write a CMakeLists.txt 
file that is more readable than the other options. CMake has a lot of 
functionality, but for most cases it is suffiecient to be familiar with just a 
couple of the functions. This folder contains an example 
[CMakeLists.txt](CMakeLists.txt) that can be used as a starting place for many 
projects. Let's walk through the file to understand what is happening.

## Configure Project
```
cmake_minimum_required(VERSION 2.8.3)
project_name(my_project)
```

These 2 lines are always included at the top of a CMakeLists.txt. 
The first line specifies the minimum required version of CMake. Honestly, I 
don't think much thought needs to go into this line. I pretty much always 
use the same version number here and I can't remember a time that it didn't 
work. I would recommend either using version 2.8.3 or else finding a project 
that is doing something similar to what you want to do and copy the version 
from that project. The version of CMake on your machine is likely much 
higher than 2.8.3, so their shouldn't be any issues using it.

The second line above specifies the name of your project. You can call it 
whatever you want. I believe uppercase, lowercase, and underscores are 
available. If you ever receive a warning message saying you didn't follow 
naming conventions, then just modify the name how it wanted.

## Add Desired Libraries
```
find_package(Eigen3 REQUIRED)
#add_subdirectory(lib/<lib_dir_name>)
```

The `find_package()` function looks for libraries installed to your system 
that have their own CMake configuration. These are things that need to be 
installed manually by the user, such as Eigen. If the package is not 
optional then you should include the keyword `REQUIRED` after specifying 
the package name. Also, if you only need some pieces of the library you 
can use the keyword `COMPONENTS`. For example, you can do this with a ROS 
package:
```
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
```

The `add_subdirectory()` function looks for local libraries inside of your 
project. Typically, people will create a `lib` folder in their project 
and clone/create any libraries they want to use locally into that folder. 
Our example doesn't use any local libraries, so this line is commented 
out. If you're interested, this command looks for a `CMakeLists.txt` 
inside the location you specify and runs that CMake configuration, adding 
it to your project.

## Include Directories
```
include_directories(include ${EIGEN_INCLUDE_DIRS})
```

The `include_directories()` function specifies where your project will 
look for `#include` statements contained in your code files. Typically, 
people create an `include` directory in their project where header 
files are stored. It is also common to have another directory layer, 
such as `include/<project_name>`. In this case the `include_dirs()` 
function we defined only looks inside of include, so your code would 
have to use `#include "<project_name>/<file_name>.h"` rather than just 
using `#include "<file_name>.h"`.

You can include multiple locations in the `include_directories()` 
function. Our example is also inluding `${EIGEN_INCLUDE_DIRS}`. When 
you call `find_package()`, that defines a few variables for the 
package you are including. One of these variables is 
`<package_name>_INCLUDE_DIRS` and to access the value of a variable in 
CMake you need to use `${<variable>}`. Hopefully it is easy to see how 
you could modify the `include_directories()` function to fit the needs 
of your own project.

**NOTE: If your build fails because it cannot find an existing header 
file, this is the first place you should look to debug.**

## Declare Executable
```
add_executable(${PROJECT_NAME} main.cpp)
#target_link_libraries(${PROJECT_NAME} ${EIGEN3_LIBRARIES})
```

The `add_executable()` function will create an executable file with the 
name of the first argument passed into the function. In this case, the 
`PROJECT_NAME` CMake variable was used; therefore, the executable will 
be called `learn_cpp`. All of the code files included in the project 
should be listed in this function. You can use multiple lines to keep 
things more readable. Here is an example:
```
add_executable(my_exe
	main.cpp
	src/my_class.cpp
	include/my_class.h
	)
```

If your project is using a library with compiled code, then you will 
have to link those libraries to executable you are creating (also 
called a `TARGET`). That is what `target_link_libraries()` does. In 
our case, Eigen is a header only library and doen't contain any 
compiled code, so we did not need to link anything. If Eigen did 
have compiled code already, the `find_package()` function creates 
another variable of the form `<package_name>_LIBRARIES>` which we 
would have to include. The example above used `${EIGEN3_LIBRARIES}`, 
but the line is commented out because it isn't needed. If you 
uncommented the line, everything would still work, but nothing would 
change. This is because the variable `EIGEN3_LIBRARIES` is just 
empty, so nothing actually is linked.

**NOTE: if you run into build errors that mention linking errors, 
you might want to check your `target_link_libraries()` function. **