#include "rectangle.hpp"

Rectangle::Rectangle() : 
    _width{0},
    _height{0}
{
    update();
}

Rectangle::Rectangle(double w, double h)
{
    setDimensions(w,h);
}

Rectangle::~Rectangle() {}

void Rectangle::setWidth(double w)
{
    _width = w;
    update();
}

void Rectangle::setHeight(double h)
{
    _height = h;
    update();
}

void Rectangle::setDimensions(double w, double h)
{
    _width = w;
    _height = h;
    update();
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

void Rectangle::update()
{
    _area = _width * _height;
    _perimeter = 2*_width + 2*_height;
}
