# C++ Tutorials
This repository was created to help BYU students that are beggining to dive into C++ programming
deeper than is taught in the ME curriculum (maybe others as well). It is assumed that the reader is
already familiar with basic data types, loops (do, while, if, for), and functions.

## How to Use
All presentations are contained in the [wiki]. This repository contatins the accompanying code that
you can build and run with a debugger to help solidify concepts from the presentations. The file
structure here should be the same as what you will see on the [wiki]. To build code in *debug* mode,
navigate to the desired directory containing a `CMakeLists.txt` file and follow these steps:
```shell
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
```
**Please report issues!** That could be code that doesn't build, typos, unclear explanations, etc.
Also, feel free to provide suggestions or ask for additional topics to be covered. I might not be
able to get to them immediately, but I really want to make this repository as useful as possible!

## IDE
Use whichever IDE with a debugger that you prefer. If you don't have one you currently use, then I
would recommend [Qt Creator] or [VS Code]. I really like both of these, and they are free!

As far as I know, [Qt Creator] really only supports C/C++. It is really useful for designing Qt GUI
applications, but I also like to use it just for debugging C++ code. [Qt Creator] can be installed
with `apt`, but that isn't the most recent version (which is likely just fine). If you want the
newest version, you can download the open source version for free but you now have to make an
account with them to do so.

[VS Code] is a little more involved to setup debugging properly, but it is not soley focused on
C/C++. You can also debug Python, have syntax highlighting and auto-complete for various languages,
locally view markdown files, view latex files, etc. Many capabilities come by default when you
download and install the deb, but there are also several free extensions available to get a lot
of extra functionality. One really nice feature is the Live Share extension, which is basically the
equivalent of Overleaf for collaborative programming.

<!-- Link Definitions -->
[wiki]: https://github.com/mjhaskell/cpp_tutorials/wiki
[Qt Creator]: https://www.qt.io/download
[VS Code]: https://code.visualstudio.com/download
