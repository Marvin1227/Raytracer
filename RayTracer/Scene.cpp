#include "Scene.h"
#include <limits>
#include <cmath>
#include <random>
#include <thread>
#include <functional>

Scene::Scene() : rng(std::random_device{}()), uniform(0.0, 1.0) {
}

std::mt19937& Scene::getRng() const {
    // Thread-local RNG seeded uniquely per thread
    thread_local std::mt19937 rng_local{
        static_cast<unsigned int>(std::random_device{}() ^
        static_cast<unsigned int>(std::hash<std::thread::id>()(std::this_thread::get_id())))};
    return rng_local;
}

void Scene::addSurface(std::unique_ptr<Surface> surface) {
    surfaces.push_back(std::move(surface));
}

bool Scene::intersect(const Ray& ray, HitRecord& record) const {
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

Vector3 Scene::sampleHemisphere(const Vector3& normal) {
    auto& rngRef = getRng();
    std::uniform_real_distribution<double> dist01(0.0, 1.0);

    double u1 = dist01(rngRef);
    double u2 = dist01(rngRef);
    
    double theta = std::acos(std::sqrt(u1));
    double phi = 2 * M_PI * u2;
    
    double x = std::sin(theta) * std::cos(phi);
    double y = std::sin(theta) * std::sin(phi);
    double z = std::cos(theta);
    
    Vector3 w = normal;
    Vector3 u = ((std::fabs(w.x) > 0.1) ? Vector3(0, 1, 0) : Vector3(1, 0, 0)).cross(w).normalize();
    Vector3 v = w.cross(u);
    
    return u * x + v * y + w * z; // cosine-weighted
}

bool Scene::russianRoulette(const Color& importance, double& pdf, int depth) {
    double maxComponent = std::max({ importance.r, importance.g, importance.b });
    
    if (depth <= 8) {
        pdf = 1.0;
        return true;
    }

    double survivalProb = std::min(0.9, maxComponent); // slightly higher max

    std::uniform_real_distribution<double> dist01(0.0, 1.0);
    if (dist01(getRng()) < survivalProb) {
        pdf = survivalProb;
        return true;
    }
    return false;
}

Color Scene::calculateDirectLighting(const Vector3& point, const Vector3& normal, 
                                     const Color& surfaceColor) {
    // Rect area light matching the scene light
    Vector3 Lcorner(3, -1, 4.9);
    Vector3 Lu(2, 0, 0);
    Vector3 Lv(0, 2, 0);
    Vector3 nL = Lu.cross(Lv).normalize();
    if (nL.z > 0.0) nL = nL * (-1.0); // emit downward into the room

    Color Le(8, 8, 8); // emission
    constexpr int lightSamples = 8;
    Color accumulated(0, 0, 0);

    auto& rngRef = getRng();
    std::uniform_real_distribution<double> dist01(0.0, 1.0);
    const double area = Lu.length() * Lv.length();

    // Start shadow rays from the offset point
    const Vector3 origin = point + normal * RAY_OFFSET;

    for (int i = 0; i < lightSamples; ++i) {
        double su = dist01(rngRef), sv = dist01(rngRef);
        Vector3 xL = Lcorner + Lu * su + Lv * sv;

        Vector3 toLight = xL - origin;
        double r2 = toLight.dot(toLight);
        if (r2 <= 0.0) continue;
        double r = std::sqrt(r2);
        Vector3 wi = toLight / r;

        // Occlusion: any blocker strictly closer than the light sample
        Ray shadow(origin, wi);
        HitRecord h;
        if (intersect(shadow, h) && h.t < r - 1e-4) {
            continue; // occluded
        }

        double cosTheta  = std::max(0.0, normal.dot(wi));
        double cosThetaL = std::max(0.0, nL.dot(-wi));
        if (cosTheta <= 0.0 || cosThetaL <= 0.0) continue;

        // Uniform area sampling: weight = (cosθ * cosθL / r²) * Area
        Color contrib = surfaceColor * Le * (cosTheta * cosThetaL * area / r2);
        accumulated = accumulated + contrib;
    }

    return accumulated * (1.0 / lightSamples);
}

Color Scene::traceRay(const Ray& ray, int depth) {
    if (depth > 16) return Color(0, 0, 0);
    
    HitRecord record;
    if (!intersect(ray, record)) {
        return Color(0.05, 0.05, 0.05);
    }
    
    if (record.material == LIGHT_SOURCE) {
        return record.color;
    }
    
    Color result(0, 0, 0);
    
    if (record.material == LAMBERTIAN) {
        // Direct lighting (classic Lambertian term)
        result = calculateDirectLighting(record.point, record.normal, record.color);
        
        double rrPdf;
        if (russianRoulette(record.color, rrPdf, depth)) {
            Vector3 newDirection = sampleHemisphere(record.normal);
            double cosTheta = std::max(0.0, record.normal.dot(newDirection));
            if (cosTheta > 0.0) {
                Ray newRay(record.point + record.normal * RAY_OFFSET, newDirection);
                Color indirectLight = traceRay(newRay, depth + 1);
                // Cosine-weighted hemisphere sampling cancels BRDF*cos/pdf, so multiply by albedo and RR weight only
                result = result + record.color * indirectLight * (1.0 / rrPdf);
            }
        }
    }
    else if (record.material == PERFECT_REFLECTOR) {
        Vector3 reflected = ray.direction - record.normal * 2 * ray.direction.dot(record.normal);
        Ray reflectedRay(record.point + record.normal * RAY_OFFSET, reflected);
        result = traceRay(reflectedRay, depth + 1);
    }
    
    return result;
}

void createHexagonalRoom(Scene& scene, const std::vector<Color>& wallColors) {
    std::vector<Color> defaultColors = {
        Color(0.8, 0.3, 0.3),
        Color(0.3, 0.8, 0.3),
        Color(0.3, 0.3, 0.8),
        Color(0.8, 0.8, 0.3),
        Color(0.8, 0.3, 0.8),
        Color(0.3, 0.8, 0.8)
    };

    std::vector<Color> colors = wallColors.empty() ? defaultColors : wallColors;

    // Hex in XY, extruded along Z [-5, 5]
    std::vector<Vector3> hexVertices = {
        Vector3(0, 6, 0),     // 0: Top
        Vector3(10, 6, 0),    // 1: Top-right
        Vector3(13, 0, 0),    // 2: Right (back-right)
        Vector3(10, -6, 0),   // 3: Bottom-right
        Vector3(0, -6, 0),    // 4: Bottom
        Vector3(-3, 0, 0)     // 5: Left (near-left)
    };

    for (int i = 0; i < 6; i++) {
        int next = (i + 1) % 6;
        Vector3 v1 = hexVertices[i];
        Vector3 v2 = hexVertices[next];

        // Extrude along Z
        v1.z = -5; v2.z = -5;
        Vector3 v3 = v2; v3.z = 5;
        Vector3 v4 = v1; v4.z = 5;

        Vector3 edge1 = v2 - v1;
        Vector3 edge2 = v4 - v1;

        // Default lambertian walls
        MaterialType wallMat = LAMBERTIAN;
        Color wallColor = (static_cast<size_t>(i) < colors.size()) ? colors[i] : Color(0.8, 0.8, 0.8);

        // Remove mirror to better observe diffuse bleeding
        if (i == 2){ 
            wallMat = PERFECT_REFLECTOR; wallColor = Color(0.95, 0.95, 0.95); 
        }

        scene.addSurface(std::make_unique<Rectangle>(v1, edge1, edge2, wallMat, wallColor));
    }

    // Floor
    scene.addSurface(std::make_unique<Rectangle>(
        Vector3(-3, -6, -5), Vector3(16, 0, 0), Vector3(0, 12, 0),
        LAMBERTIAN, Color(1.0, 1.0, 1.0)));

    // Ceiling
    scene.addSurface(std::make_unique<Rectangle>(
        Vector3(-3, -6, 5), Vector3(16, 0, 0), Vector3(0, 12, 0),
        LAMBERTIAN, Color(0.2, 0.2, 0.2)));
}