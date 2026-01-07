#pragma once
#include "Vector3.h"

struct Ray {
    Vector3 origin, direction;
    
    Ray(const Vector3& o, const Vector3& d) : origin(o), direction(d.normalize()) {}
    
    Vector3 at(double t) const { return origin + direction * t; }
};
