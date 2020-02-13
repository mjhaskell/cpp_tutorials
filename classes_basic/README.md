# Object-Oriented Programming Intro
Most companies who write production level code will probably want you to be 
familiar with object-oriented programming. Classes provide a way to organize 
your data so that it can only be accessed by things that need access, and in a 
way that is easy to read. Also, understanding classes will most likely make it 
easier for you to read and understand other people's code. Let's look at a 
familiar example:
```cpp
std::vector<double> vec{0.1, 3, 2.9};
vec.push_back(1.5);
int vec_length = vec.size();
```

In this example the class is `vector` (which is inside of the `std` namespace). 
The object (or instance of the class) is `vec`. Initially, `vec` is constructed 
with 3 numbers. `push_back()` is a member method of the `vector` class, meaning 
that you can call this method from an object. In this case `push_back()` 
appends an aditional value to `vec`. `size` is another member function that 
simply returns the value of a member variable that is private (hidden) from 
the end user. We will learn more about all if this throughout this document, so 
don't worry if any of these concepts seem fuzzy right now.

Classes can store variables (data) and have functions that use and/or 
manipulate that data. So when you have several pieces of related data in your 
project, consider using a class to organize it. Breaking your code up into 
classes (which will typically each have their own file) can also have the 
benefit of keeping individual files shorter and easier to parse.

After that basic introduction, we can start to discuss in more detail the 
properties and capabilities of classes. Here are some things we will look at:

  1. [Defining a Class](#defining-a-class)
  2. [Member Variables](#member-variables)
  3. [Member Functions](#member-functions)
  4. [Encapsulation](#encapsulation)
  5. [Constructors](#constructors)
  6. [Destructors](#destructors)

## Defining a Class
Just to get some syntax down, let's create our own version of `int` as a class:
```cpp
class MyInt
{
    int value = 0;
public:
    void setValue(int new_value) { value = new_value; }
    int getValue() { return value; }
};

#include <iostream>

int main()
{
    MyInt i;
    std::cout << "Value is " << i.getValue() << std::endl; // value is 0
    i.setValue(9);
    std::cout << "Value is " << i.getValue() << std::endl; // value is 9

    return 0;
}
```

For simplicity here, I left everything together rather than separating it out 
into 3 files. You can see that there isn't much to creating a basic class. We 
had to give it a name, and then everything we defined inside of the `{}` 
belonged to the class. This example should build and run just fine. The main 
function declared an instance of the `MyInt` class, returned 0 with the 1st
`getValue()` (because we defined that behavior in the class), and then set the 
value to 9 with `setValue()`.

**NOTE: make sure to end class definitions with a semi-colon!**

Now let's look at how to separate class definitions into different files. Just 
like you can do forward declarations of functions before actually defining the 
function, you can have a declares member variables and functions in the class 
without defining them and then define them outside of the class:
```cpp
class MyInt
{
    int value = 0;
public:
    void setValue(int new_value);
    int getValue();
};

// above this point would go in a header file, below in a source file
void MyInt::setValue(int new_value)
{
    value = new_value;
}

int MyInt::getValue()
{
    return value;
}
```

This will behave exactly the same as before now that we have defined the 
member functions outside of the class. Notice that we have to use similar 
syntax to namespaces to do so: `MyInt::setValue`. This is to set the scope of 
the function we are defining to be part of the class. Now you should be able to 
see how just class definition could be isolated into a header file and then the 
actual member function definitions could happen in a separate cpp file. One
thing to note is that the cpp file would have to `#include` the header file to 
copy the forward declarations in the pre-processor build step.

## Member Variables
We have already talked about member variable a bit, but let's make sure it is 
clear how to create and use them. Here is a simple class to store the 
properties of a rectangle:
```cpp
struct Rectangle
{
    double width;
    double height
};

int main(int argc, char** argv)
{
    Rectangle rect;
    rect.width = 2.5;
    rect.height = 2;

    double area = rect.width * rect.height; // area is 5

    return 0;
}
```

Notice that I used a `struct` rather than a `class` here. These 2 data types 
are basically the same with 1 minor difference that we will discuss later. So 
you can mentally substitute `struct` with `class` for this example. It is an 
accepted practice to use a `struct` if there are no member functions and a 
`class` if there are, which is why I used `struct` here.

Also, member variables (and functions) are accessed from an object with a `.` 
(unless the object is a pointer, then it would be `->`). For our Rectangle 
class, we could assign and access both of the member variables outside of the 
class. I will note, just so it is clear, that any data type can be used as a 
member variable of a class, including other classes. All of our examples so far 
have used doubles and ints, but anything can be used.

## Member Functions
We have seen member functions before, but let's spice up our Rectangle class 
and use them.
```cpp
class Rectangle
{
public:
    double width;
    double height;
    double area;
    double perimeter;

    void setDimentions(double w, double h)
    {
        width = w;
        height = h;
        area = width * height;
        perimeter = 2*width + 2*height;
    }
    double getArea() { return area; }
    double getPerimeter() { return perimeter; }
};

int main(int argc, char** argv)
{
    Rectangle rect;
    rect.setDimentions(2.5, 2);

    double A = rect.getArea();      // A is 5
    double P = rect.getPerimeter(); // P is 9

    return 0;
}
```

Now our Rectangle is a `class` rather than a `struct` with 4 member variables 
and 3 member functions. The end user no longer has to remember area and 
perimeter formulas for a rectangle; all they have to do is set the dimensions 
of the rectangle. This is a useful feature of classes, because now any time I 
change the dimensions of the rectangle all 4 member variables are updated. 
Another added benefit is that the code is easy to read; if I didn't know 
the formula for area for some reason, I could still understand this code 
because I have a rectangle object that I call "get area" on. Granted, you could 
also have written a function called `calculateAreaOfRectangle()` which is also 
easy to read, but the name is longer and more cumbersome to have the same 
readability AND you would have to remember to call this funtion yourself every 
time you updated the dimensions of the rectangle. Hopefully you can see how 
`classes` with member functions can be very useful.

## Encapsulation
In the last section we beefed up our Rectangle class, but there are still many 
flaws in it :(. To fix 1 of these flaws, it is finally time to address the 
elephant in the code...why I have used the keyword `public`. The idea of 
encapsulation is to be able to contain related data in an organized way, but 
also to allow access to that data only when it is needed. Is is common practice 
to hide all (or many) member variables of a class from the end user. This is 
prevent the data from being used incorrectly, and potentially to hide 
proprietary information. Let's modify our last main script to see this:
```cpp
int main(int argc, char** argv)
{
    Rectangle rect;
    rect.setDimentions(2.5, 2);

    double A = rect.getArea();      // A is 5
    double P = rect.getPerimeter(); // P is 9

    rect.width = 20; // valid because width is a public member variable

    A = rect.getArea(); // A is still 5, why didn't it change?
    rect.perimeter = 999; // valid operation but it makes no sense

    return 0;
}
```

So the word `public` means that member variables and functions can be accessed 
outside of the class (which is inside the main loop in this case). Well we 
definitely don't want `area` or `perimeter` to be public because those are not 
meant to be set, they are properties of a rectangle based on the width and 
height. Even though we intend for the user to use `width` and `height`, we want 
to make sure the `area` and `perimeter` are updated correctly when those values 
change. Really, we want to hide all 4 of our member variables from the user. 
This is done with the keyword `private`. We can't just replace `public` with 
`private` because the user needs access to the member functions:
```cpp
class Rectangle
{
private:
    double width;
    double height;
    double area;
    double perimeter;

public:
    void setDimentions(double w, double h)
    {
        width = w;
        height = h;
        area = width * height;
        perimeter = 2*width + 2*height;
    }
    double getArea() { return area; }
    double getPerimeter() { return perimeter; }
};

int main(int argc, char** argv)
{
    Rectangle rect;
    rect.setDimentions(2.5, 2);

    double A = rect.getArea();      // A is 5
    double P = rect.getPerimeter(); // P is 9

//    rect.width = 20; // invalid because width is private
    rect.setDimentions(20, 2); // valid way to update width

    A = rect.getArea(); // A is now 40
//    P = rect.perimeter; // invalid - perimeter was updated, but it's private

    return 0;
}
```

By making the member variables private, we have encapsulated the data. Now the 
interface with our class should work as expected, even if it doesn't have all 
the functionality we want. It is also very common to see "getters" and 
"setters", which are just member functions like we defined to either access 
or set the member variables. Often, "getters" and "setters" are as simple as 
assigning and returning values; however, there are times (like our setter) 
where they are more involved. With encapsulation, elements of the class are 
able to act like a black box. If you are the only intended audience to use 
your class, then you might not need to worry about making anything private. I 
recommend using private variables even if it is only for you to get practice so 
that you don't have to think about it when you are writing production level 
code at a company. Also, it can help restrict you from doing something dumb 
with your own class (like overiding the width without updating area/perimeter 
as we saw with the example above). Those types of bugs can be hard to find.

## Constructors
Now let's address another issue that still exists with `Rectangle`. What would 
happen in this scenario?
```cpp
int main(int argc, char** argv)
{
    Rectangle rect;

    double A = rect.getArea(); // What would A be?

    return 0;
}
```

This is somewhat of a trick question because non of the member variables were 
initialized. Memory locations would have been allocated for the 4 member 
variables, but their current value would be whatever existed in memory before 
running your program...so the answer is that A would be garbage. And it would 
most likely be a different value every time you ran the program. To fix this 
issue, we could just set each member varialbe to 0 in the class definition. A 
better approach is to define a constructor (i.e. a function that is called when 
an instance of your class is created). A constructor specifies how to 
initialize your class. In fact, without you even knowing it, a default 
constructor was created for `Rectangle` allowing it to be used. The default 
constructor in most cases will likely not be sufficient. So here is how to 
define one:
```cpp
class Rectangle
{
public:
    Rectangle() : width{0}, height{0}
    {
        area = 0;
        perimeter = 0;
    }
    void setDimentions(double w, double h)
    {
        width = w;
        height = h;
        area = width * height;
        perimeter = 2*width + 2*height;
    }
    double getArea() { return area; }
    double getPerimeter() { return perimeter; }

private:
    double width;
    double height;
    double area;
    double perimeter;
};
```

The syntax for a constructor is a bit different than other functions. First, 
the name of a constructor must match the name of your class. Second, there is 
no return type (not even void). Lastly, you can use an initializer list to 
instatiate member variables. This happens before the function definition and
after using a colon, and each variable initialized here is separated with a 
comma (except no comma after the last variable). In this example, half of the 
member variables were set in the initializer list and the other half in the
constructor's actual function. This is just to show that you can use either
method; however, it is probably better to initialize simple variables in the 
initializer list and then variables that are unable to do so in the function 
portion.

**NOTE: initializer lists need to be in the same order that variables were 
declared.**

A constructor can also take arguments. In our example, every rectangle has a 
width and a height, so it would make sense to construct a `Rectangle` by
providing these values:

```cpp
class Rectangle
{
public:
    Rectangle(double w, double h) : width{w}, height{h}
    {
        update();
    }
    void setDimentions(double w, double h)
    {
        width = w;
        height = h;
        update();
    }
    double getArea() { return area; }
    double getPerimeter() { return perimeter; }

private:
    void update()
    {
        area = width * height;
        perimeter = 2*width + 2*height;
    }

    double width;
    double height;
    double area;
    double perimeter;
};

int main(int argc, char** argv)
{
    Rectangle rect{2.5, 2};

    double A = rect.getArea(); // A is 5 like we would expect

    return 0;
}
```

Now the constructor requires the width and height to be supplied when you 
create an object. Also, I created a new private function to update area and 
perimeter so that I didn't have code duplication - it is always good to reduce 
code duplication! 

A few other things should be noted about constructors. You should initialize 
all member variables to a default value that makes sense, which avoids 
unexpected behavior. Multiple constructors can be defined as long as they take 
different arguments, just like regular functions.

## Destructors
Just like constructors are called when you create an object, destructors are 
called when an object goes out of scope. The main purpose of destructors is to 
properly de-allocate any memory that was manually allocated in the class. This 
has to do with pointers and dynamic memory, which is a topic for a different 
time. Just know that if the class ever calls `new` or some function with 
`alloc()`, that there needs to be a matching call of `delete` or `dealloc()`. 
These often go in the destructor. There are other times where you might want a 
non-empty destructor, but that is project dependent. Here is the syntax for an 
empty destructor:
```cpp
class Rectangle
{
public:
    Rectangle(double w, double h) : width{w}, height{h}
    {
        update();
    }
    ~Rectangle()
    {
        // do nothing because we didn't allocate memory
    }
    void setDimentions(double w, double h)
    {
        width = w;
        height = h;
        update();
    }
    double getArea() { return area; }
    double getPerimeter() { return perimeter; }

private:
    void update()
    {
        area = width * height;
        perimeter = 2*width + 2*height;
    }

    double width;
    double height;
    double area;
    double perimeter;
};

int main(int argc, char** argv)
{
    Rectangle rect{2.5, 2};

    double A = rect.getArea(); // A is 5 like we would expect

    return 0;
}
```

As you can see, the destructor has the same name as the class except with a ~ 
before the name. If you want to test when/if the destructor is called, you can 
always put a print statement in it.

That's all for now folks!
