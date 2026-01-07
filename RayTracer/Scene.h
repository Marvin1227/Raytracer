#pragma once
#include "Vector3.h"
#include "Ray.h"
#include "Material.h"
#include "Surface.h"
#include <vector>
#include <memory>
#include <random>

class Scene {
private:
    std::vector<std::unique_ptr<Surface>> surfaces;
    // Legacy RNG members kept for compatibility, but thread-local RNG is used via getRng().
    mutable std::mt19937 rng;
    mutable std::uniform_real_distribution<double> uniform;
    
public:
    Scene();
    
    // Returns a thread-local RNG safe to use from multiple threads.
    std::mt19937& getRng() const;
    void addSurface(std::unique_ptr<Surface> surface);
    bool intersect(const Ray& ray, HitRecord& record) const;
    
    Vector3 sampleHemisphere(const Vector3& normal);
    bool russianRoulette(const Color& importance, double& pdf, int depth);
    Color calculateDirectLighting(const Vector3& point, const Vector3& normal, 
                                 const Color& surfaceColor);
	Color traceRay(const Ray& ray, int depth = 0);
};

void createHexagonalRoom(Scene& scene, const std::vector<Color>& wallColors = {});
