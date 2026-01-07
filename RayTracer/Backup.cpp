#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <fstream>
#include <memory>
#include <algorithm>
#include <limits>      // For std::numeric_limits
#include <stdexcept>   // For std::invalid_argument

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif // !M_PI

constexpr double EPSILON = 1e-6;
constexpr double RAY_OFFSET = 0.001;

// Vector3 class for 3D operations
struct Vector3 {
    double x, y, z;

    Vector3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}

    Vector3 operator+(const Vector3& v) const { return Vector3(x + v.x, y + v.y, z + v.z); }
    Vector3 operator-(const Vector3& v) const { return Vector3(x - v.x, y - v.y, z - v.z); }
    Vector3 operator*(double t) const { return Vector3(x * t, y * t, z * t); }
    Vector3 operator/(double t) const { return Vector3(x / t, y / t, z / t); }

    // Array subscript operator for accessing components by index
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

    double length() const { return sqrt(x * x + y * y + z * z); }
    Vector3 normalize() const { double l = length(); return l > 0 ? *this / l : Vector3(); }
};

// Color class using double precision
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

// Ray class
struct Ray {
    Vector3 origin, direction;

    Ray(const Vector3& o, const Vector3& d) : origin(o), direction(d.normalize()) {}

    Vector3 at(double t) const { return origin + direction * t; }
};

// Material types
enum MaterialType { LAMBERTIAN, PERFECT_REFLECTOR, LIGHT_SOURCE };

// Hit record for ray-surface intersections
struct HitRecord {
    Vector3 point;
    Vector3 normal;
    double t;
    MaterialType material;
    Color color;
    bool hit;

    HitRecord() : t(0), material(LAMBERTIAN), hit(false) {}
};

// Abstract Surface class
class Surface {
public:
    MaterialType material;
    Color color;

    Surface(MaterialType mat, const Color& col) : material(mat), color(col) {}
    virtual ~Surface() = default;
    virtual bool intersect(const Ray& ray, HitRecord& record) const = 0;
    virtual Vector3 getNormal(const Vector3& point) const = 0;
};

// Sphere class (implicit object)
class Sphere : public Surface {
private:
    Vector3 center;
    double radius;

public:
    Sphere(const Vector3& c, double r, MaterialType mat, const Color& col)
        : Surface(mat, col), center(c), radius(r) {
    }

    bool intersect(const Ray& ray, HitRecord& record) const override {
        Vector3 oc = ray.origin - center;
        double a = ray.direction.dot(ray.direction);
        double b = 2.0 * oc.dot(ray.direction);
        double c = oc.dot(oc) - radius * radius;

        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0) return false;

        double sqrt_discriminant = sqrt(discriminant);
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

    Vector3 getNormal(const Vector3& point) const override {
        return (point - center).normalize();
    }
};

// General Polygon class for n-sided polygons
class Polygon : public Surface {
private:
    std::vector<Vector3> vertices;
    Vector3 normal;
    Vector3 centroid;

public:
    Polygon(const std::vector<Vector3>& verts, MaterialType mat, const Color& col)
        : Surface(mat, col), vertices(verts) {
        if (vertices.size() < 3) {
            throw std::invalid_argument("Polygon must have at least 3 vertices");
        }
        calculateNormal();
        calculateCentroid();
    }

private:
    void calculateNormal() {
        // Use Newell's method for robust normal calculation
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

    void calculateCentroid() {
        centroid = Vector3(0, 0, 0);
        for (const auto& vertex : vertices) {
            centroid = centroid + vertex;
        }
        centroid = centroid / vertices.size();
    }

public:
    bool intersect(const Ray& ray, HitRecord& record) const override {
        // First check if ray intersects the plane of the polygon
        double denom = normal.dot(ray.direction);
        if (std::fabs(denom) < EPSILON) return false; // Ray parallel to plane

        double t = (vertices[0] - ray.origin).dot(normal) / denom;
        if (t < RAY_OFFSET) return false; // Behind ray origin

        Vector3 hitPoint = ray.at(t);

        // Check if point is inside polygon using ray casting algorithm
        if (!pointInPolygon(hitPoint)) return false;

        record.t = t;
        record.point = hitPoint;

        // Make sure normal faces the ray origin
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

private:
    bool pointInPolygon(const Vector3& point) const {
        // Project polygon and point onto 2D plane for point-in-polygon test
        // Choose the coordinate plane that gives the largest projection
        int maxAxis = 0;
        if (std::fabs(normal.y) > std::fabs(normal.x)) maxAxis = 1;
        if (std::fabs(normal.z) > std::fabs(normal[maxAxis])) maxAxis = 2;

        // Create 2D coordinates
        std::vector<std::pair<double, double>> poly2D;
        std::pair<double, double> point2D;

        for (const auto& v : vertices) {
            if (maxAxis == 0) { // Project onto YZ plane
                poly2D.push_back({ v.y, v.z });
                point2D = { point.y, point.z };
            }
            else if (maxAxis == 1) { // Project onto XZ plane
                poly2D.push_back({ v.x, v.z });
                point2D = { point.x, point.z };
            }
            else { // Project onto XY plane
                poly2D.push_back({ v.x, v.y });
                point2D = { point.x, point.y };
            }
        }

        // Ray casting algorithm
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

public:
    Vector3 getNormal(const Vector3& point) const override {
        return normal;
    }

    // Helper method to get vertex count
    size_t getVertexCount() const { return vertices.size(); }
};

// Mesh class for complex 3D objects made of multiple polygons
class Mesh : public Surface {
private:
    std::vector<std::unique_ptr<Polygon>> faces;
    Vector3 boundingBoxMin, boundingBoxMax;

public:
    Mesh(MaterialType mat, const Color& col) : Surface(mat, col) {}

    void addFace(const std::vector<Vector3>& vertices) {
        faces.push_back(std::make_unique<Polygon>(vertices, material, color));
        updateBoundingBox(vertices);
    }

private:
    void updateBoundingBox(const std::vector<Vector3>& vertices) {
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

public:
    bool intersect(const Ray& ray, HitRecord& record) const override {
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

    Vector3 getNormal(const Vector3& point) const override {
        // This is a simplified approach - in practice, you'd want to find 
        // which face the point is on and return that face's normal
        return Vector3(0, 1, 0); // Default up normal
    }
};

// Helper functions to create common shapes
namespace ShapeFactory {

    std::unique_ptr<Polygon> createTriangle(const Vector3& v0, const Vector3& v1, const Vector3& v2,
        MaterialType mat, const Color& col) {
        return std::make_unique<Polygon>(std::vector<Vector3>{v0, v1, v2}, mat, col);
    }

    std::unique_ptr<Polygon> createQuad(const Vector3& v0, const Vector3& v1, const Vector3& v2, const Vector3& v3,
        MaterialType mat, const Color& col) {
        return std::make_unique<Polygon>(std::vector<Vector3>{v0, v1, v2, v3}, mat, col);
    }

    std::unique_ptr<Mesh> createCube(const Vector3& center, double size, MaterialType mat, const Color& col) {
        auto cube = std::make_unique<Mesh>(mat, col);
        double half = size / 2.0;

        // Define 8 vertices of cube
        Vector3 v000 = center + Vector3(-half, -half, -half);
        Vector3 v001 = center + Vector3(-half, -half, half);
        Vector3 v010 = center + Vector3(-half, half, -half);
        Vector3 v011 = center + Vector3(-half, half, half);
        Vector3 v100 = center + Vector3(half, -half, -half);
        Vector3 v101 = center + Vector3(half, -half, half);
        Vector3 v110 = center + Vector3(half, half, -half);
        Vector3 v111 = center + Vector3(half, half, half);

        // Add 6 faces (quads)
        cube->addFace({ v000, v100, v110, v010 }); // Front
        cube->addFace({ v101, v001, v011, v111 }); // Back
        cube->addFace({ v001, v000, v010, v011 }); // Left
        cube->addFace({ v100, v101, v111, v110 }); // Right
        cube->addFace({ v010, v110, v111, v011 }); // Top
        cube->addFace({ v000, v001, v101, v100 }); // Bottom

        return cube;
    }

    std::unique_ptr<Mesh> createPyramid(const Vector3& base_center, double base_size, double height,
        MaterialType mat, const Color& col) {
        auto pyramid = std::make_unique<Mesh>(mat, col);
        double half = base_size / 2.0;

        // Base vertices
        Vector3 v0 = base_center + Vector3(-half, -half, 0);
        Vector3 v1 = base_center + Vector3(half, -half, 0);
        Vector3 v2 = base_center + Vector3(half, half, 0);
        Vector3 v3 = base_center + Vector3(-half, half, 0);

        // Apex
        Vector3 apex = base_center + Vector3(0, 0, height);

        // Add faces
        pyramid->addFace({ v0, v1, v2, v3 }); // Base (quad)
        pyramid->addFace({ v0, v1, apex });   // Side 1 (triangle)
        pyramid->addFace({ v1, v2, apex });   // Side 2 (triangle)
        pyramid->addFace({ v2, v3, apex });   // Side 3 (triangle)
        pyramid->addFace({ v3, v0, apex });   // Side 4 (triangle)

        return pyramid;
    }

    std::unique_ptr<Polygon> createRegularPolygon(const Vector3& center, const Vector3& normal,
        double radius, int sides, MaterialType mat, const Color& col) {
        std::vector<Vector3> vertices;

        // Create local coordinate system
        Vector3 w = normal.normalize();
        Vector3 u = ((std::fabs(w.x) > 0.1) ? Vector3(0, 1, 0) : Vector3(1, 0, 0)).cross(w).normalize();
        Vector3 v = w.cross(u);

        // Generate vertices
        for (int i = 0; i < sides; i++) {
            double angle = 2.0 * M_PI * i / sides;
            double x = radius * cos(angle);
            double y = radius * sin(angle);
            vertices.push_back(center + u * x + v * y);
        }

        return std::make_unique<Polygon>(vertices, mat, col);
    }
}

// Alternative: Quad class that can be split into two triangles
class Quad : public Surface {
private:
    Vector3 v0, v1, v2, v3;
    Vector3 normal;

public:
    Quad(const Vector3& vertex0, const Vector3& vertex1, const Vector3& vertex2, const Vector3& vertex3,
        MaterialType mat, const Color& col)
        : Surface(mat, col), v0(vertex0), v1(vertex1), v2(vertex2), v3(vertex3) {
        // Calculate normal from first triangle
        normal = (v1 - v0).cross(v2 - v0).normalize();
    }

    bool intersect(const Ray& ray, HitRecord& record) const override {
        // Test intersection with both triangles that make up the quad

        // Triangle 1: v0, v1, v2
        if (intersectTriangle(ray, v0, v1, v2, record)) {
            return true;
        }

        // Triangle 2: v0, v2, v3
        if (intersectTriangle(ray, v0, v2, v3, record)) {
            return true;
        }

        return false;
    }

private:
    bool intersectTriangle(const Ray& ray, const Vector3& vert0, const Vector3& vert1, const Vector3& vert2, HitRecord& record) const {
        // Möller-Trumbore algorithm for individual triangle
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

public:
    Vector3 getNormal(const Vector3& point) const override {
        return normal;
    }
};

// Rectangle class (polygonal object)
class Rectangle : public Surface {
private:
    Vector3 corner;
    Vector3 u, v; // Two edges
    Vector3 normal;

public:
    Rectangle(const Vector3& c, const Vector3& edge1, const Vector3& edge2,
        MaterialType mat, const Color& col)
        : Surface(mat, col), corner(c), u(edge1), v(edge2) {
        normal = u.cross(v).normalize();
    }

    bool intersect(const Ray& ray, HitRecord& record) const override {
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

    Vector3 getNormal(const Vector3& point) const override {
        return normal;
    }
};

// Scene class
class Scene {
private:
    std::vector<std::unique_ptr<Surface>> surfaces;
    std::mt19937 rng;
    std::uniform_real_distribution<double> uniform;

public:
    Scene() : rng(std::random_device{}()), uniform(0.0, 1.0) {}

    // Add getter for RNG access
    std::mt19937& getRng() { return rng; }

    void addSurface(std::unique_ptr<Surface> surface) {
        surfaces.push_back(std::move(surface));
    }

    bool intersect(const Ray& ray, HitRecord& record) const {
        HitRecord tempRecord;
        bool hitAnything = false;
        double closestSoFar = std::numeric_limits<double>::max();

        for (const auto& surface : surfaces) {
            if (surface->intersect(ray, tempRecord) && tempRecord.t < closestSoFar) {
                hitAnything = true;
                closestSoFar = tempRecord.t;
                record = tempRecord;
            }
        }

        return hitAnything;
    }

    // Sample random direction in hemisphere with better distribution
    Vector3 sampleHemisphere(const Vector3& normal) {
        double u1 = uniform(rng);
        double u2 = uniform(rng);

        // Cosine-weighted hemisphere sampling for better convergence
        double theta = acos(sqrt(u1));
        double phi = 2 * M_PI * u2;

        double x = sin(theta) * cos(phi);
        double y = sin(theta) * sin(phi);
        double z = cos(theta);

        // Create local coordinate system
        Vector3 w = normal;
        Vector3 u = ((std::fabs(w.x) > 0.1) ? Vector3(0, 1, 0) : Vector3(1, 0, 0)).cross(w).normalize();
        Vector3 v = w.cross(u);

        return u * x + v * y + w * z;
    }

    // Pass Depth as parameter for Russian roulette
    bool russianRoulette(const Color& importance, double& pdf, int depth) {
        double maxComponent = std::max({ importance.r, importance.g, importance.b });

        // Start Russian roulette only after depth 3
        if (depth <= 3) {
            pdf = 1.0;
            return true;
        }

        double survivalProb = std::min(0.8, maxComponent);

        if (uniform(rng) < survivalProb) {
            pdf = survivalProb;
            return true;
        }
        return false;
    }

    // Direct lighting calculation
    Color calculateDirectLighting(const Vector3& point, const Vector3& normal, const Color& surfaceColor) {
        // Light source properties
        Vector3 lightPos(5, -1, 4.9);  // Center of light rectangle
        Vector3 lightSize(4, 2, 0);    // Size of light rectangle
        Color lightColor(3, 3, 3);     // Reduced intensity

        // Sample random point on light source for soft shadows
        double u = uniform(rng);
        double v = uniform(rng);
        Vector3 lightSamplePos = lightPos + Vector3(lightSize.x * (u - 0.5), lightSize.y * (v - 0.5), 0);

        Vector3 lightDir = (lightSamplePos - point).normalize();
        double distance = (lightSamplePos - point).length();

        // Check for shadows
        Ray shadowRay(point + normal * RAY_OFFSET, lightDir);
        HitRecord shadowRecord;
        if (intersect(shadowRay, shadowRecord) && shadowRecord.t < distance - RAY_OFFSET) {
            return Color(0, 0, 0); // In shadow
        }

        // Lambertian BRDF calculation
        double cosTheta = std::max(0.0, normal.dot(lightDir));
        double lightArea = lightSize.x * lightSize.y;
        double attenuation = 1.0 / (distance * distance);

        return surfaceColor * lightColor * cosTheta * lightArea * attenuation * (1 / M_PI);
    }

    // Monte Carlo ray tracing
    Color traceRay(const Ray& ray, int depth = 0) {
        if (depth > 10) return Color(0, 0, 0);

        HitRecord record;
        if (!intersect(ray, record)) {
            return Color(0.1, 0.1, 0.2); // Background color
        }

        if (record.material == LIGHT_SOURCE) {
            return record.color;
        }

        Color result(0, 0, 0);

        if (record.material == LAMBERTIAN) {
            // Direct lighting
            result = calculateDirectLighting(record.point, record.normal, record.color);

            // Indirect lighting (Monte Carlo)
            double pdf;
            if (russianRoulette(record.color, pdf, depth)) {
                Vector3 newDirection = sampleHemisphere(record.normal);
                Ray newRay(record.point + record.normal * RAY_OFFSET, newDirection);

                double cosTheta = record.normal.dot(newDirection);
                Color indirectLight = traceRay(newRay, depth + 1);

                // Correct BRDF calculation for Lambertian surface
                // BRDF = albedo/π, PDF = cosθ/π for cosine-weighted sampling
                result = result + record.color * indirectLight * (1.0 / pdf);
            }
        }
        else if (record.material == PERFECT_REFLECTOR) {
            // Perfect reflection
            Vector3 reflected = ray.direction - record.normal * 2 * ray.direction.dot(record.normal);
            Ray reflectedRay(record.point + record.normal * RAY_OFFSET, reflected);
            result = traceRay(reflectedRay, depth + 1);
        }

        return result;
    }
};

// Camera class
class Camera {
private:
    Vector3 eye;
    Vector3 corner1, corner2, corner3, corner4;

public:
    Camera() {
        // Camera setup from the provided specification
        eye = Vector3(-1, 0, 0);
        corner1 = Vector3(0, -1, -1);
        corner2 = Vector3(0, 1, -1);
        corner3 = Vector3(0, 1, 1);
        corner4 = Vector3(0, -1, 1);
    }

    Ray getRay(double u, double v) const {
        // Bilinear interpolation on camera plane
        Vector3 horizontal = corner2 - corner1;
        Vector3 vertical = corner4 - corner1;
        Vector3 pixelPos = corner1 + horizontal * u + vertical * v;

        Vector3 direction = (pixelPos - eye).normalize();
        return Ray(eye, direction);
    }
};

// Create hexagonal room with configurable colors
void createHexagonalRoom(Scene& scene, const std::vector<Color>& wallColors = {}) {
    // Default wall colors if none provided
    std::vector<Color> defaultColors = {
        Color(0.8, 0.3, 0.3),  // Red
        Color(0.3, 0.8, 0.3),  // Green
        Color(0.3, 0.3, 0.8),  // Blue
        Color(0.8, 0.8, 0.3),  // Yellow
        Color(0.8, 0.3, 0.8),  // Magenta
        Color(0.3, 0.8, 0.8)   // Cyan
    };

    std::vector<Color> colors = wallColors.empty() ? defaultColors : wallColors;

    // Hexagonal room vertices (x,y coordinates, z extends from -5 to 5)
    std::vector<Vector3> hexVertices = {
        Vector3(0, 6, 0),    // Top
        Vector3(10, 6, 0),   // Top-right
        Vector3(13, 0, 0),   // Right
        Vector3(10, -6, 0),  // Bottom-right
        Vector3(0, -6, 0),   // Bottom
        Vector3(-3, 0, 0)    // Left
    };

    // Create walls (rectangles)
    for (int i = 0; i < 6; i++) {
        int next = (i + 1) % 6;
        Vector3 v1 = hexVertices[i];
        Vector3 v2 = hexVertices[next];

        v1.z = -5; v2.z = -5;
        Vector3 v3 = v2; v3.z = 5;
        Vector3 v4 = v1; v4.z = 5;

        Vector3 edge1 = v2 - v1;
        Vector3 edge2 = v4 - v1;

        Color wallColor = (i < colors.size()) ? colors[i] : Color(0.8, 0.8, 0.8);
        scene.addSurface(std::make_unique<Rectangle>(v1, edge1, edge2, LAMBERTIAN, wallColor));
    }

    // Floor (z = -5)
    scene.addSurface(std::make_unique<Rectangle>(
        Vector3(-3, -6, -5), Vector3(16, 0, 0), Vector3(0, 12, 0),
        LAMBERTIAN, Color(0.6, 0.6, 0.8)));

    // Ceiling (z = 5)
    scene.addSurface(std::make_unique<Rectangle>(
        Vector3(-3, -6, 5), Vector3(16, 0, 0), Vector3(0, 12, 0),
        LAMBERTIAN, Color(0.8, 0.8, 0.2)));
}

int main() {
    const int width = 400;
    const int height = 300;
    const int samples = 10; // Increased samples for less noise

    Scene scene;
    Camera camera;

    // Define custom wall colors (optional)
    std::vector<Color> customWallColors = {
        Color(0.9, 0.6, 0.6),  // Light red
        Color(0.6, 0.9, 0.6),  // Light green  
        Color(0.6, 0.6, 0.9),  // Light blue
        Color(0.9, 0.9, 0.6),  // Light yellow
        Color(0.9, 0.6, 0.9),  // Light magenta
        Color(0.6, 0.9, 0.9)   // Light cyan
    };

    // Create hexagonal room with custom colors
    createHexagonalRoom(scene, customWallColors);

    // Add sphere (perfect reflector)
    scene.addSurface(std::make_unique<Sphere>(
        Vector3(5, 2, -2), 1.5, PERFECT_REFLECTOR, Color(0.9, 0.9, 0.9)));

    // Add various polygon shapes using the new Polygon and Mesh classes
    std::cout << "Creating polygon shapes..." << std::endl;

    //// Triangle using Polygon class
    //scene.addSurface(ShapeFactory::createTriangle(
    //    Vector3(6, -4, -3),     // v0 - bottom left
    //    Vector3(10, -4, -3),    // v1 - bottom right  
    //    Vector3(8, -1, -1),     // v2 - top apex
    //    LAMBERTIAN, Color(0.9, 0.1, 0.1))); // Bright red

    //// Add a cube
    //scene.addSurface(ShapeFactory::createCube(
    //    Vector3(3, 2, -3),      // center
    //    1.5,                    // size
    //    LAMBERTIAN, Color(0.2, 0.8, 0.2))); // Green

    // Add a pyramid
    scene.addSurface(ShapeFactory::createPyramid(
        Vector3(9, -2, -4),      // base center
        2.0,                    // base size
        3.0,                    // height
        LAMBERTIAN, Color(0.2, 0.2, 0.8))); // Blue

    //// Add a hexagon (6-sided polygon)
    //scene.addSurface(ShapeFactory::createRegularPolygon(
    //    Vector3(5, -2, 2),      // center
    //    Vector3(0, 0, 1),       // normal (pointing up)
    //    1.0,                    // radius
    //    6,                      // sides
    //    LAMBERTIAN, Color(0.8, 0.8, 0.2))); // Yellow

    //// Add an octagon (8-sided polygon) on the wall
    //scene.addSurface(ShapeFactory::createRegularPolygon(
    //    Vector3(11, 1, 0),      // center
    //    Vector3(-1, 0, 0),      // normal (pointing left)
    //    0.8,                    // radius
    //    8,                      // sides
    //    LAMBERTIAN, Color(0.8, 0.2, 0.8))); // Magenta

    // Add light source (rectangle on ceiling)
    scene.addSurface(std::make_unique<Rectangle>(
        Vector3(3, -1, 4.9), Vector3(4, 0, 0), Vector3(0, 2, 0),
        LIGHT_SOURCE, Color(4, 4, 4)));

    std::vector<Color> image(width * height);

    std::cout << "Rendering " << width << "x" << height << " image with " << samples << " samples per pixel..." << std::endl;

    for (int y = 0; y < height; y++) {

        std::cout << "Line: " << y << " out of " << height << std::endl;

        for (int x = 0; x < width; x++) {
            Color pixelColor(0, 0, 0);

            for (int s = 0; s < samples; s++) {
                // Add slight jittering for anti-aliasing
                double jitterX = (std::uniform_real_distribution<double>(-0.5, 0.5))(scene.getRng());
                double jitterY = (std::uniform_real_distribution<double>(-0.5, 0.5))(scene.getRng());

                double u = (x + 0.5 + jitterX) / width;
                double v = (y + 0.5 + jitterY) / height;

                Ray ray = camera.getRay(u, v);
                pixelColor = pixelColor + scene.traceRay(ray);
            }

            pixelColor = pixelColor * (1.0 / samples);
            image[y * width + x] = pixelColor.clamp();
        }
    }

    // Save to PPM file
    std::ofstream file("raytracer_output.ppm");
    file << "P3\n" << width << " " << height << "\n255\n";

    for (int y = height - 1; y >= 0; y--) {
        for (int x = 0; x < width; x++) {
            Color c = image[y * width + x];
            int r = int(255 * c.r);
            int g = int(255 * c.g);
            int b = int(255 * c.b);
            file << r << " " << g << " " << b << "\n";
        }
    }

    file.close();
    std::cout << "Rendering complete! Output saved to raytracer_output.ppm" << std::endl;

    return 0;
}


