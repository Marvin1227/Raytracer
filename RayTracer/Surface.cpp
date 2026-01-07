#include "Surface.h"
#include <stdexcept>
#include <limits>

// ============================================================================
// Sphere Implementation
// ============================================================================
Sphere::Sphere(const Vector3& c, double r, MaterialType mat, const Color& col)
    : Surface(mat, col), center(c), radius(r) {
}

bool Sphere::intersect(const Ray& ray, HitRecord& record) const {
    Vector3 oc = ray.origin - center;
    double a = ray.direction.dot(ray.direction);
    double b = 2.0 * oc.dot(ray.direction);
    double c = oc.dot(oc) - radius * radius;
    
    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0) return false;
    
    double sqrt_discriminant = std::sqrt(discriminant);
    double t1 = (-b - sqrt_discriminant) / (2.0 * a);
    double t2 = (-b + sqrt_discriminant) / (2.0 * a);
    
    double t = (t1 > RAY_OFFSET) ? t1 : t2;
    if (t <= RAY_OFFSET) return false;
    
    record.t = t;
    record.point = ray.at(t);
    record.normal = getNormal(record.point);
    record.material = material;
    record.color = color;
    record.hit = true;
    
    return true;
}

Vector3 Sphere::getNormal(const Vector3& point) const {
    return (point - center).normalize();
}

// ============================================================================
// Polygon Implementation
// ============================================================================
Polygon::Polygon(const std::vector<Vector3>& verts, MaterialType mat, const Color& col)
    : Surface(mat, col), vertices(verts) {
    if (vertices.size() < 3) {
        throw std::invalid_argument("Polygon must have at least 3 vertices");
    }
    calculateNormal();
    calculateCentroid();
}

void Polygon::calculateNormal() {
    normal = Vector3(0, 0, 0);
    for (size_t i = 0; i < vertices.size(); i++) {
        const Vector3& v1 = vertices[i];
        const Vector3& v2 = vertices[(i + 1) % vertices.size()];
        normal.x += (v1.y - v2.y) * (v1.z + v2.z);
        normal.y += (v1.z - v2.z) * (v1.x + v2.x);
        normal.z += (v1.x - v2.x) * (v1.y + v2.y);
    }
    normal = normal.normalize();
}

void Polygon::calculateCentroid() {
    centroid = Vector3(0, 0, 0);
    for (const auto& vertex : vertices) {
        centroid = centroid + vertex;
    }
    centroid = centroid / static_cast<double>(vertices.size());
}

bool Polygon::intersect(const Ray& ray, HitRecord& record) const {
    double denom = normal.dot(ray.direction);
    if (std::fabs(denom) < EPSILON) return false;
    
    double t = (vertices[0] - ray.origin).dot(normal) / denom;
    if (t < RAY_OFFSET) return false;
    
    Vector3 hitPoint = ray.at(t);
    
    if (!pointInPolygon(hitPoint)) return false;
    
    record.t = t;
    record.point = hitPoint;
    
    Vector3 outward_normal = normal;
    if (ray.direction.dot(outward_normal) > 0) {
        outward_normal = outward_normal * (-1.0);
    }
    record.normal = outward_normal;
    
    record.material = material;
    record.color = color;
    record.hit = true;
    return true;
}

bool Polygon::pointInPolygon(const Vector3& point) const {
    int maxAxis = 0;
    if (std::fabs(normal.y) > std::fabs(normal.x)) maxAxis = 1;
    if (std::fabs(normal.z) > std::fabs(normal[maxAxis])) maxAxis = 2;
    
    std::vector<std::pair<double, double>> poly2D;
    std::pair<double, double> point2D;
    
    for (const auto& v : vertices) {
        if (maxAxis == 0) {
            poly2D.push_back({ v.y, v.z });
            point2D = { point.y, point.z };
        }
        else if (maxAxis == 1) {
            poly2D.push_back({ v.x, v.z });
            point2D = { point.x, point.z };
        }
        else {
            poly2D.push_back({ v.x, v.y });
            point2D = { point.x, point.y };
        }
    }
    
    bool inside = false;
    for (size_t i = 0, j = poly2D.size() - 1; i < poly2D.size(); j = i++) {
        if (((poly2D[i].second > point2D.second) != (poly2D[j].second > point2D.second)) &&
            (point2D.first < (poly2D[j].first - poly2D[i].first) *
                (point2D.second - poly2D[i].second) / (poly2D[j].second - poly2D[i].second) + poly2D[i].first)) {
            inside = !inside;
        }
    }
    return inside;
}

Vector3 Polygon::getNormal(const Vector3& point) const {
    return normal;
}

// ============================================================================
// Mesh Implementation
// ============================================================================
Mesh::Mesh(MaterialType mat, const Color& col) : Surface(mat, col) {
}

void Mesh::addFace(const std::vector<Vector3>& vertices) {
    faces.push_back(std::make_unique<Polygon>(vertices, material, color));
    updateBoundingBox(vertices);
}

void Mesh::updateBoundingBox(const std::vector<Vector3>& vertices) {
    if (faces.size() == 1) {
        boundingBoxMin = boundingBoxMax = vertices[0];
    }
    for (const auto& v : vertices) {
        boundingBoxMin.x = std::min(boundingBoxMin.x, v.x);
        boundingBoxMin.y = std::min(boundingBoxMin.y, v.y);
        boundingBoxMin.z = std::min(boundingBoxMin.z, v.z);
        boundingBoxMax.x = std::max(boundingBoxMax.x, v.x);
        boundingBoxMax.y = std::max(boundingBoxMax.y, v.y);
        boundingBoxMax.z = std::max(boundingBoxMax.z, v.z);
    }
}

bool Mesh::intersect(const Ray& ray, HitRecord& record) const {
    HitRecord tempRecord;
    bool hitAnything = false;
    double closestSoFar = std::numeric_limits<double>::max();
    
    for (const auto& face : faces) {
        if (face->intersect(ray, tempRecord) && tempRecord.t < closestSoFar) {
            hitAnything = true;
            closestSoFar = tempRecord.t;
            record = tempRecord;
        }
    }
    
    return hitAnything;
}

Vector3 Mesh::getNormal(const Vector3& point) const {
    return Vector3(0, 1, 0);
}

// ============================================================================
// Quad Implementation
// ============================================================================
Quad::Quad(const Vector3& vertex0, const Vector3& vertex1, const Vector3& vertex2, 
           const Vector3& vertex3, MaterialType mat, const Color& col)
    : Surface(mat, col), v0(vertex0), v1(vertex1), v2(vertex2), v3(vertex3) {
    normal = (v1 - v0).cross(v2 - v0).normalize();
}

bool Quad::intersect(const Ray& ray, HitRecord& record) const {
    if (intersectTriangle(ray, v0, v1, v2, record)) {
        return true;
    }
    if (intersectTriangle(ray, v0, v2, v3, record)) {
        return true;
    }
    return false;
}

bool Quad::intersectTriangle(const Ray& ray, const Vector3& vert0, const Vector3& vert1, 
                             const Vector3& vert2, HitRecord& record) const {
    Vector3 edge1 = vert1 - vert0;
    Vector3 edge2 = vert2 - vert0;
    Vector3 h = ray.direction.cross(edge2);
    double a = edge1.dot(h);
    
    if (a > -0.00001 && a < 0.00001) return false;
    
    double f = 1.0 / a;
    Vector3 s = ray.origin - vert0;
    double u = f * s.dot(h);
    
    if (u < 0.0 || u > 1.0) return false;
    
    Vector3 q = s.cross(edge1);
    double v = f * ray.direction.dot(q);
    
    if (v < 0.0 || u + v > 1.0) return false;
    
    double t = f * edge2.dot(q);
    
    if (t <= RAY_OFFSET) return false;
    
    record.t = t;
    record.point = ray.at(t);
    record.normal = normal;
    record.material = material;
    record.color = color;
    record.hit = true;
    
    return true;
}

Vector3 Quad::getNormal(const Vector3& point) const {
    return normal;
}

// ============================================================================
// Rectangle Implementation
// ============================================================================
Rectangle::Rectangle(const Vector3& c, const Vector3& edge1, const Vector3& edge2,
                    MaterialType mat, const Color& col)
    : Surface(mat, col), corner(c), u(edge1), v(edge2) {
    normal = u.cross(v).normalize();
}

bool Rectangle::intersect(const Ray& ray, HitRecord& record) const {
    double denom = normal.dot(ray.direction);
    if (std::fabs(denom) < RAY_OFFSET) return false;
    
    Vector3 p0l0 = corner - ray.origin;
    double t = p0l0.dot(normal) / denom;
    
    if (t <= RAY_OFFSET) return false;
    
    Vector3 p = ray.at(t);
    Vector3 d = p - corner;
    
    double ddotu = d.dot(u);
    double ddotv = d.dot(v);
    
    if (ddotu < 0 || ddotu > u.dot(u)) return false;
    if (ddotv < 0 || ddotv > v.dot(v)) return false;
    
    record.t = t;
    record.point = p;
    record.normal = normal;
    record.material = material;
    record.color = color;
    record.hit = true;
    
    return true;
}

Vector3 Rectangle::getNormal(const Vector3& point) const {
    return normal;
}

// ============================================================================
// ShapeFactory Implementation
// ============================================================================
namespace ShapeFactory {
    std::unique_ptr<Polygon> createTriangle(const Vector3& v0, const Vector3& v1, 
                                           const Vector3& v2, MaterialType mat, const Color& col) {
        return std::make_unique<Polygon>(std::vector<Vector3>{v0, v1, v2}, mat, col);
    }
    
    std::unique_ptr<Polygon> createQuad(const Vector3& v0, const Vector3& v1, 
                                       const Vector3& v2, const Vector3& v3, 
                                       MaterialType mat, const Color& col) {
        return std::make_unique<Polygon>(std::vector<Vector3>{v0, v1, v2, v3}, mat, col);
    }
    
    std::unique_ptr<Mesh> createCube(const Vector3& center, double size, 
                                    MaterialType mat, const Color& col) {
        auto cube = std::make_unique<Mesh>(mat, col);
        double half = size / 2.0;
        
        Vector3 v000 = center + Vector3(-half, -half, -half);
        Vector3 v001 = center + Vector3(-half, -half, half);
        Vector3 v010 = center + Vector3(-half, half, -half);
        Vector3 v011 = center + Vector3(-half, half, half);
        Vector3 v100 = center + Vector3(half, -half, -half);
        Vector3 v101 = center + Vector3(half, -half, half);
        Vector3 v110 = center + Vector3(half, half, -half);
        Vector3 v111 = center + Vector3(half, half, half);
        
        cube->addFace({ v000, v100, v110, v010 });
        cube->addFace({ v101, v001, v011, v111 });
        cube->addFace({ v001, v000, v010, v011 });
        cube->addFace({ v100, v101, v111, v110 });
        cube->addFace({ v010, v110, v111, v011 });
        cube->addFace({ v000, v001, v101, v100 });
        
        return cube;
    }
    
    std::unique_ptr<Mesh> createPyramid(const Vector3& base_center, double base_size, 
                                       double height, MaterialType mat, const Color& col) {
        auto pyramid = std::make_unique<Mesh>(mat, col);
        double half = base_size / 2.0;
        
        Vector3 v0 = base_center + Vector3(-half, -half, 0);
        Vector3 v1 = base_center + Vector3(half, -half, 0);
        Vector3 v2 = base_center + Vector3(half, half, 0);
        Vector3 v3 = base_center + Vector3(-half, half, 0);
        Vector3 apex = base_center + Vector3(0, 0, height);
        
        pyramid->addFace({ v0, v1, v2, v3 });
        pyramid->addFace({ v0, v1, apex });
        pyramid->addFace({ v1, v2, apex });
        pyramid->addFace({ v2, v3, apex });
        pyramid->addFace({ v3, v0, apex });
        
        return pyramid;
    }
    
    std::unique_ptr<Polygon> createRegularPolygon(const Vector3& center, const Vector3& normal,
                                                 double radius, int sides, 
                                                 MaterialType mat, const Color& col) {
        std::vector<Vector3> vertices;
        
        Vector3 w = normal.normalize();
        Vector3 u = ((std::fabs(w.x) > 0.1) ? Vector3(0, 1, 0) : Vector3(1, 0, 0)).cross(w).normalize();
        Vector3 v = w.cross(u);
        
        for (int i = 0; i < sides; i++) {
            double angle = 2.0 * M_PI * i / sides;
            double x = radius * std::cos(angle);
            double y = radius * std::sin(angle);
            vertices.push_back(center + u * x + v * y);
        }
        
        return std::make_unique<Polygon>(vertices, mat, col);
    }
}