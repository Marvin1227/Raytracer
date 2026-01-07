#pragma once
#include "Vector3.h"

enum MaterialType { 
    LAMBERTIAN, 
    PERFECT_REFLECTOR, 
    LIGHT_SOURCE 
};

struct HitRecord {
    Vector3 point;
    Vector3 normal;
    double t = 0.0;
    MaterialType material = LAMBERTIAN;
    Color color;
    bool hit = false;
};
