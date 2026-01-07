#pragma once
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

constexpr double EPSILON = 1e-6;
constexpr double RAY_OFFSET = 0.001;

struct Vector3 {
    double x, y, z;
    
    Vector3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    
    Vector3 operator+(const Vector3& v) const { return Vector3(x + v.x, y + v.y, z + v.z); }
    Vector3 operator-(const Vector3& v) const { return Vector3(x - v.x, y - v.y, z - v.z); }
    Vector3 operator*(double t) const { return Vector3(x * t, y * t, z * t); }
    Vector3 operator/(double t) const { return Vector3(x / t, y / t, z / t); }
	Vector3 operator-() const { return Vector3(-x, -y, -z); }
    
    double operator[](int index) const {
        switch (index) {
            case 0: return x;
            case 1: return y;
            case 2: return z;
            default: return 0;
        }
    }
    
    double dot(const Vector3& v) const { return x * v.x + y * v.y + z * v.z; }
    Vector3 cross(const Vector3& v) const {
        return Vector3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }
    
    double length() const { return std::sqrt(x * x + y * y + z * z); }
    Vector3 normalize() const { 
        double l = length(); 
        return l > 0 ? *this / l : Vector3(); 
    }
};

struct Color {
    double r, g, b;
    
    Color(double r = 0, double g = 0, double b = 0) : r(r), g(g), b(b) {}
    
    Color operator+(const Color& c) const { return Color(r + c.r, g + c.g, b + c.b); }
    Color operator*(double t) const { return Color(r * t, g * t, b * t); }
    Color operator*(const Color& c) const { return Color(r * c.r, g * c.g, b * c.b); }
    
    Color clamp() const {
        return Color(std::min(1.0, std::max(0.0, r)),
                    std::min(1.0, std::max(0.0, g)),
                    std::min(1.0, std::max(0.0, b)));
    }
};
