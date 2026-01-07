 #pragma once
#include "Vector3.h"
#include "Ray.h"
#include "Material.h"
#include <vector>
#include <memory>

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

// Sphere class
class Sphere : public Surface {
private:
    Vector3 center;
    double radius;
    
public:
    Sphere(const Vector3& c, double r, MaterialType mat, const Color& col);
    bool intersect(const Ray& ray, HitRecord& record) const override;
    Vector3 getNormal(const Vector3& point) const override;
};

// Polygon class
class Polygon : public Surface {
private:
    std::vector<Vector3> vertices;
    Vector3 normal;
    Vector3 centroid;
    
    void calculateNormal();
    void calculateCentroid();
    bool pointInPolygon(const Vector3& point) const;
    
public:
    Polygon(const std::vector<Vector3>& verts, MaterialType mat, const Color& col);
    bool intersect(const Ray& ray, HitRecord& record) const override;
    Vector3 getNormal(const Vector3& point) const override;
    size_t getVertexCount() const { return vertices.size(); }
};

// Mesh class
class Mesh : public Surface {
private:
    std::vector<std::unique_ptr<Polygon>> faces;
    Vector3 boundingBoxMin, boundingBoxMax;
    
    void updateBoundingBox(const std::vector<Vector3>& vertices);
    
public:
    Mesh(MaterialType mat, const Color& col);
    void addFace(const std::vector<Vector3>& vertices);
    bool intersect(const Ray& ray, HitRecord& record) const override;
    Vector3 getNormal(const Vector3& point) const override;
};

// Quad class
class Quad : public Surface {
private:
    Vector3 v0, v1, v2, v3;
    Vector3 normal;
    
    bool intersectTriangle(const Ray& ray, const Vector3& vert0, const Vector3& vert1, 
                          const Vector3& vert2, HitRecord& record) const;
    
public:
    Quad(const Vector3& vertex0, const Vector3& vertex1, const Vector3& vertex2, 
         const Vector3& vertex3, MaterialType mat, const Color& col);
    bool intersect(const Ray& ray, HitRecord& record) const override;
    Vector3 getNormal(const Vector3& point) const override;
};

// Rectangle class
class Rectangle : public Surface {
private:
    Vector3 corner;
    Vector3 u, v;
    Vector3 normal;
    
public:
    Rectangle(const Vector3& c, const Vector3& edge1, const Vector3& edge2,
             MaterialType mat, const Color& col);
    bool intersect(const Ray& ray, HitRecord& record) const override;
    Vector3 getNormal(const Vector3& point) const override;
};

// ShapeFactory namespace
namespace ShapeFactory {
    std::unique_ptr<Polygon> createTriangle(const Vector3& v0, const Vector3& v1, 
                                           const Vector3& v2, MaterialType mat, const Color& col);
    std::unique_ptr<Polygon> createQuad(const Vector3& v0, const Vector3& v1, 
                                       const Vector3& v2, const Vector3& v3, 
                                       MaterialType mat, const Color& col);
    std::unique_ptr<Mesh> createCube(const Vector3& center, double size, 
                                    MaterialType mat, const Color& col);
    std::unique_ptr<Mesh> createPyramid(const Vector3& base_center, double base_size, 
                                       double height, MaterialType mat, const Color& col);
    std::unique_ptr<Polygon> createRegularPolygon(const Vector3& center, const Vector3& normal,
                                                 double radius, int sides, 
                                                 MaterialType mat, const Color& col);
}
