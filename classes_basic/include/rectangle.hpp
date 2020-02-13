#ifndef RECTANGLE_HPP
#define RECTANGLE_HPP

class Rectangle
{
public:
    Rectangle();
    Rectangle(double w, double h);
    virtual ~Rectangle();
    
    void setWidth(double w);
    void setHeight(double h);
    void setDimensions(double w, double h);
    double getWidth();
    double getHeight();
    double getArea();
    double getPerimeter();

private:
    void update();

    double _width;
    double _height;
    double _area;
    double _perimeter;
};

#endif // RECTANGLE_HPP
