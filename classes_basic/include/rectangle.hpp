#ifndef RECTANGLE_HPP
#define RECTANGLE_HPP

class Rectangle
{
public:
    Rectangle();
    Rectangle(double width, double height);
    Rectangle(const Rectangle& other);
    virtual ~Rectangle();
    
    void setWidth(double width);
    void setHeight(double height);
    void setDimensions(double width, double height);
    double getWidth();
    double getHeight();
    double getArea();
    double getPerimeter();

private:
    void updateProperties();

    double _width;
    double _height;
    double _area;
    double _perimeter;
};

#endif // RECTANGLE_HPP
