# C++ Build Process
The build process consists of 3 main steps in order to generate a working 
executable.

## Pre-Processing
The pre-processor handles all of the lines that begin with `#` in your code 
files. The most common pre-processor directive is `#include`, which is how you 
include header files inside of the source files.
```cpp
#include <iostream>
#include "my_class.h"
```

Using `<>` specifically instructs your machine to look for system files and the 
`""` is meant to find local files (most likely that you created). Although this 
is the intent, it often works using either method. So what actually happens 
when you call `#include`? Behind the scenes during the build process, your 
machine is practically copying all of the header file you are including and 
pasting it where you called `#include`. Even if your weren't using classes, you 
could move all of your function forward declarations before `int main()` into a 
header file, and if you `#include` that header file, it does the exact same 
thing.

**NOTE: pre-processor directives do not end with a semi-colon because they are 
not C++ code.**

Header guards are also important to understand. Let's consider a scenario. Say 
your `main.cpp` calls `#include <iostream>` as well as some other custom header 
file...and that header file also calls `#include <iostream>`. Now that we know 
`#include` literally copies the file, `<iostream>` would be copied into your 
`main.cpp` twice. This causes all sorts of issues because you cannot have 2 
definitions of the same variable or function; otherwise, the linker would not 
know what to do. The scenario above actually happens all of the time, so how do 
we prevent the problem? Header guards using pre-processor directives!
```cpp
#ifndef MY_FILE_H
#define MY_FILE_H

... (add the contents of the header file here)

#endif // MY_FILE_H
```

The pre-processor has access to variables declared with `#define`. These 
variables can be given a value, and if no value is specified they act like a 
boolean. So the script above tells the pre-processor, if the variable 
`MY_FILE_H` has not been defined then define it. If it has already been defined 
then nothing will happen because all of the code is inside of the if statement. 
Back to the problem we were facing. Now it doesn't matter how many times any 
particular header file is included because the first time it is copied, 
`MY_FILE_H` will be defined, meaning that all other times the `#include` calls 
will not copy anything. The take away is that you should add header guards to 
all of your header files.

Also, other `#if` statements can be used to specify sections of code that you 
do or do not want compiled. This can be useful if you have print statements to 
help you debug that you don't want to print all of the time. So you can just 
define a variable like `#define DISPLAY` and then you can use it like this:
```cpp
#ifdef DISPLAY
std::cout << "whatever you want to display" << std::endl;
#else
...(do something else you might want)
#endif //DISPLAY
```

Now when you want to debug/display, you can `#define DISPLAY` at the top of the 
file, otherwise just don't define it. Sections of code inside of a 
pre-processor `#if` statement that is false will not be compiled.

The last pre-processor directive I want to mention is macros. As mentioned
before, you can define variables for the pre-processor, but you can also define 
functions (which are called macros because they aren't real C++ functions).
```cpp
#define ONE 1

#define MAX(a,b) (a > b ? a : b)

#define PRINT_EIGEN(A) (std::cout << #A << ":\n" << A << std::endl)
```

The same copying principle applies with macros. Wherever your file uses a 
macro, the preprocess will do a literal text replacement. So wherever your file 
contained `MAX(var1, var2)` the pre-processor would replace it with `(var1 >
var2 ? var1 : var2)`. It is probably best practice to use parenthesis around 
the operation you want your macro to perform so that you don't run into 
difficult issues to debug. Also, the dummy variables in the macro are replaced 
with the actual variables with the macro is called. For example:
```cpp
int i = 1;
int j = 9;

int max = MAX(i,j);

// the above line will be replaced with this
int max = (i < j ? i : j);

One other fancy trick is seen in the `PRINT_EIGEN` macro. If # is used before a 
variable, the preprocessor will replace it with the name the user assigned the 
variable:
```cpp
Eigen::Matrix3d my_mat;

PRINT_EIGEN(my_may);

// the above line will be replaced with this
(std::cout << "my_mat" << "\n:" << my_mat << std::endl);
```

Another thing to note is that macros are C-styled code rather than C++. With 
C++, there is almost always multiple ways to do things; with macros, there is 
probably a better way to accomplish what you want. For instance, the `MAX` 
macro could be defined with a templated function. I would also suggest 
replacing the `DISPLAY` variable for debugging with an actual boolean that is 
passed in as a command line argument. That said, I'm not aware of another way 
to have a function that prints out the name of a variable, so I still use the 
`PRINT_EIGEN` macro.
