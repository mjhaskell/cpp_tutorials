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