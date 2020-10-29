#include <iostream>
#include "rectangle.hpp"

#define PRINT_RECTANGLE(r) printRectangle(r, #r)

void printRectangle(Rectangle& rectangle, std::string name)
{
    std::cout << name << ":" << std::endl;
    std::cout << "\twidth: " << rectangle.getWidth() << std::endl;
    std::cout << "\theight: " << rectangle.getHeight() << std::endl;
    std::cout << "\tarea: " << rectangle.getArea() << std::endl;
    std::cout << "\tperimeter: " << rectangle.getPerimeter() << std::endl;
}

int main()
{
    // constructor without arguments
    Rectangle rect1;
    PRINT_RECTANGLE(rect1);

    rect1.setWidth(2.5);
    PRINT_RECTANGLE(rect1);

    rect1.setHeight(2);
    PRINT_RECTANGLE(rect1);

    // constructor with arguments
    Rectangle rect2{2,4};
    PRINT_RECTANGLE(rect2);

    rect2.setDimensions(10, 10);
    PRINT_RECTANGLE(rect2);

    return 0;
}
