#include "Camera.h"

Camera::Camera() {
    eye = Vector3(-1, 0, 0);
    corner1 = Vector3(0, -1, -1);
    corner2 = Vector3(0, 1, -1);
    corner4 = Vector3(0, -1, 1);
}

Ray Camera::getRay(double u, double v) const {
    Vector3 horizontal = corner2 - corner1;
    Vector3 vertical = corner4 - corner1;
    Vector3 pixelPos = corner1 + horizontal * u + vertical * v;
    
    Vector3 direction = (pixelPos - eye).normalize();
    return Ray(eye, direction);
}