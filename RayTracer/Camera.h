#pragma once
#include "Vector3.h"
#include "Ray.h"

class Camera {
private:
    Vector3 eye;
    Vector3 corner1, corner2, corner4;
    
public:
    Camera();
    Ray getRay(double u, double v) const;
};
