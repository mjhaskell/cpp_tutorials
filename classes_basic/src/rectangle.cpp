#include <iostream>
#include "rectangle.hpp"

Rectangle::Rectangle() :
    _width{0},
    _height{0}
{
    std::cout << "[Rectangle] default constructor" << std::endl;
    updateProperties();
}

Rectangle::Rectangle(double w, double h)
{
    std::cout << "[Rectangle] constructor with args" << std::endl;
    setDimensions(w,h);
}

Rectangle::~Rectangle() 
{
    std::cout << "[Rectangle] destructor" << std::endl;
}

void Rectangle::setWidth(double w)
{
    _width = w;
    updateProperties();
}

void Rectangle::setHeight(double h)
{
    _height = h;
    updateProperties();
}

void Rectangle::setDimensions(double w, double h)
{
    _width = w;
    _height = h;
    updateProperties();
}

double Rectangle::getWidth()
{
    return _width;
}

double Rectangle::getHeight()
{
    return _height;
}

double Rectangle::getArea()
{
    return _area;
}

double Rectangle::getPerimeter()
{
    return _perimeter;
}

void Rectangle::updateProperties()
{
    _area = _width * _height;
    _perimeter = 2*_width + 2*_height;
}
