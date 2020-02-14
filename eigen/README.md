# Eigen Introduction
Eigen is a C++ header-only library that provides matrix and vector 
functionality. It is basically the Numpy of C++. Eigen is written to be 
lightweight, fast, and efficient. Their 
[documentation](https://eigen.tuxfamily.org/dox/) is really good, so please 
refer to it for everything that isn't covered here. These items will be covered 
here:

  1. [Installation](#installation)
  2. [Basic Syntax](#basic-syntax)
  3. [Initialization](#initialization)
  4. [Accessing Elements](#accessing-elements)
  5. [Matrix Operations](#matrix-operations)
  6. [Warnings](#warnings)

## Installation
If you don't really care too much about which version you install, then the 
easiest way to install Eigen is with apt:
```
sudo apt install libeigen3-dev
```

At the time this document was created, this installed version 3.3.4 while the 
newest version is 3.3.7. So here is how you can install from source (you can 
choose a different location, but I use `~/software`):
```
cd ~/software
git clone https://gitlab.com/libeigen/eigen.git
git checkout tags/3.3.7
mkdir build && cd build
cmake ..
sudo make install
```

Hopefully those steps are pretty straight forward, especially if you already 
went through the [CMake tutorial](../cmake). Note that these instructions are 
specifically installing version 3.3.7, so as new releases come out you can 
always `git pull`, `checkout` the desired tag, and then install the desired 
release.

## Basic Syntax
Let's see how to create a 3x3 matrix first, since those are used a lot:
```cpp
Eigen::Matrix<double,3,3> stack_mat;
Eigen::Matrix<int,Eigen::Dynamic,Eigen::Dynamic> heap_mat(3,3);
```

Alright, so we see that we have to specify a few parameters inside of `<>` in 
order to create a `Matrix`. This is because Eigen uses templated classes, but 
we won't go into detail on what that means here; you can note that it is 
similar to `std::vector`. The 1st parameter is the data type you want to use 
for the values inside of the matrix. The example above used `double` and `int`, 
but you could also use `float`. The 2nd parameter is the number of rows and the 
3rd parameter is the number of columns. I named the first matrix `stack_mat` 
because memory for this matrix is stored on the stack, which basically means 
that the size is known and allocated at compile time and can _not_ change. The 
second matrix was named `heap_mat` because it uses dynamic memory (which is 
allocated on the heap). Dynamic memory is used for large objects that take up 
a lot of memory and when you want to be able to change an object's size. With 
Eigen, you use `Dynamic` when specifying the size of the matrix to use dynamic 
memory. Notice that we specified the size of the dynamic matrix in the 
constructor rather than in the `<>`. These dynamic matrices also have methods 
that allow you to reshape them; they can do this because the memory is 
allocated at run time (working with the heap in general means that the memory 
management is during run time). It is OK if this doesn't make a lot of sense 
right now. This is all related to pointers and memory allocation, which will be 
covered in a different tutorial.

For now, know that you can either explicitly specify the size (like 
`stack_mat`) or you can have dynamic sizes (like `heap_mat`). Eigen's 
documentation says that there is a huge performance boost using fixed size 
matrices when the size is roughly less than 16, so try to use fixed sizes for
smaller matrices and vectors. Eigen also specifies that after roughly a size of 
32, the performance boost of using fixed sizes becomes negligible. I will also 
note that reshaping memory is an expensive operation, so even when you do use 
dynamic sizes for large matrices and vectors, avoid reshaping them when you 
can (and this is the case with any dynamic memory not just Eigen matrices). 
Also, for large matrices, Eigen's documentation says dynamic matrices can be 
more aggressive trying to vectorize the operation (although their 
[page](https://eigen.tuxfamily.org/dox/TopicVectorization.html) that 
supposedly talks about their vectorization is empty :/ ...).

That was quite a bit of information not necessarily related to Eigen directly, 
but I figured someone reading this might have wondered the difference between 
the 2 ways to create an Eigen `Matrix`. Now that we're past that, let's look at 
some convenient typedefs Eigen has provided us:
```cpp
Eigen::Matrix3d a; // 3x3 matrix of doubles
Eigen::Matrix2i b; // 2x2 matrix of ints
Eigen::Matrix4f c; // 4x4 matrix of floats
Eigen::MatrixXd d(2,5); // 2x5 matrix of doubles (dynamic memory)
Eigen::Vector4d x; // 4x1 matrix of doubles
Eigen::VectorXf y(30); // 30x1 matrix of floats (dynamic memory)
```

For fixed sized matrices you can use `Matrix#t` or `Vector#t` (where # is a 
number up to 4, and t is the type) rather than typing `Matrix<t,#,#>`. For 
dynamic sized matrices you can use `MatrixXt` or `VectorXt` (where t is the 
type again) rather than typing `Matrix<t,Dynamic,Dynamic>` or 
`Matrix<t,Dynamic,1>`. If you want a row vector instead, similar rules apply 
except with `RowVector#t`.

## Initialization
Let's move on to adding data into a matrix. There is a constructor option for 
fixed sized vectors up to size 4:
```cpp
Eigen::Vector2d a{5.0, 6.0};
Eigen::Vector3d b{5.0, 6.0, 7.0};
Eigen::Vector4d c{5.0, 6.0, 7.0, 8.0};
```

The more general option uses the `<<` operator to initialize:
```cpp
Eigen::Matrix3i mat;
mat << 1,2,3, 
       4,5,6, 
       7,8,9;

Eigen::Matrix2d A;
A << 4,3, 2,1;

Eigen::Vector3d x;
x << 1,2,3;

Eigen::VectorXd z(12);
z << x,x,x,x;
```

The `<<` operator can be used for any size `Matrix` where you just give it a 
comma separated list of numbers (they can all be on the same line or you can 
break it out into multiple lines). If you provide an incorrect amount of 
numbers I believe it builds just fine, but then throws a runtime error. As seen 
in the last example above, you can also use `Matrix` objects with the `<<` 
operator to initialize (as long as the sizes line up).

There are also a few other convenient initialization methods:
```cpp
Eigen::Matrix4d a;
a.setZero(); // set everything to 0
a.setOnes(); // set everything to 1
a.setConstant(3.5); // set everything to 3.5
a.setRandom(); // set everything randomly between -1 and 1

Eigen::Matrix3d b = Eigen::Matrix3d::Zero();
Eigen::Matrix3d c = Eigen::Matrix3d::Identity();
Eigen::Matrix<int,10,10> I;
I.setZero(); // set everything to zero
I.diagonal().setOnes(); // set diagonal to ones
``

There are very similar options for dynamic sizes, except you have to specify 
the size in the function calls. There are too many methods that can be used to 
show here. Look at Eigen's 
[documentation](https://eigen.tuxfamily.org/dox/group__TutorialAdvancedInitialization.html) 
for more information.



## Accessing Elements
A single element can be accessed using `()` with the row and column as 
arguments. You can read the value or assign the value:
```cpp
Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
I(2,2) = 3;
double val = I(0,1);
```

There are several methods you can use as well (see documentation for more 
detail):
```cpp
Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
I.row(0) << 1,2,3;
I.col(2) << 3,2,1;
I.diagonal() << 9,9,9;
I.block<2,2>(0,0) << 1,1, 2,2; // takes a 2x2 block starting at (0,0)
I.bottomRightCorner<2,3>() << 1,2,3, 4,5,6; // takes 2x3 block at bottom right
```

These are a few operations to give you an idea. Others exist like `topRows()`, 
`bottomRows()`, `leftCols()`, `topRightCorner()`, etc. The syntax is a slightly 
different for dynamic matrices, so be sure to look it up. Also, vectors have a 
few of their own special methods like `head()`, `tail()`, and `segment()` which 
are used to access segments of the vector at the beginning, end, or anywhere in 
between. If you ever ask yourself "I wonder if Eigen can do this...", just 
search online and they probably already have a function for it!

## Matrix Operations
Eigen will be bit different than Numpy here. Eigen uses regular operators 
(+,-,\*,/) to do arithmetic, so you don't have to think about using np.dot() or 
anything like that. The dimensions do have to line up or else errors will be 
thrown. An IDE might catch some of these before you actually try to run the 
code. 
```cpp
Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
Eigen::Matrix<double,3,2> B = Eigen::Matrix<double,3,2>::Random();
Eigen::Vector3d x{1,2,3};

Eigen::Matrix<double,3,2> C = 2*A.conjugate()*B + A.transpose()*A/3.0;
Eigen::Vector3d y = A.adjoint()*x + x;
```

I think for the most part things will behave how you expect; however, some 
things might not. For example, you can not add or subtract a scalar to a matrix 
or a vector. Technically, these operations don't make sense from a linear 
algebra point of view anyway, but the freedoms Numpy provides might throw you 
off for a bit until you get used to Eigen. There is one place Numpy fails where 
Eigen probably does what you would expect to happen: inner products and outer 
products of vectors. Coefficient-wise operations do not happen by default with 
Eigen either (again they don't really make sense in linear algebra), but Eigen 
does have methods that allow for it with things like `A.cwiseProduct(B)`. The 
shapes of the matrices/vectors will have to be the same for these types of 
operations, as expected.

That's probably all I will introduce in this document. I have found it fairly 
easy to find answers to questions I had about Eigen as I was learning (and even 
now) by searching online. It might come from Eigen's documentation or places 
like Stack Overflow. Either way, answers to your questions most likely are out 
there!

## Warnings
There is one main caution to be aware of: aliasing. To understand the issue, I 
will explain a little about Lazy Evaluation. This is when something on the left 
hand side of the `=` is being manipulated directly. So when saying `A = B+C`, 
there is no temporary object stored that equals B+C to then be copied into A 
(which is what happens normally). This is an issue if you tried to do 
`A = A.transpose()`, because A is being changed while doing the operation. This 
can cause very unexpected behavior. Eigen was designed really well still. For 
matrix multiplication operations, temporary values are stored. So `A = A*x` or 
even `A *= x` do not have any problems. The main functions to pay attention to 
are `transpose()` and block operations that overlap on the same matrix. See 
[here](https://eigen.tuxfamily.org/dox/group__TopicAliasing.html) for more 
information.
