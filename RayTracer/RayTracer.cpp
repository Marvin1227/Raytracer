#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <thread>
#include <atomic>
#include <mutex>
#include <random>
#include "Vector3.h"
#include "Ray.h"
#include "Material.h"
#include "Surface.h"
#include "Scene.h"
#include "Camera.h"

int main() {
    const int width = 2560;
    const int height = 1440;
    const int samples = 256;

    Scene scene;
    Camera camera;
    
    std::vector<Color> customWallColors = {
        Color(0.9, 0.6, 0.6),
        Color(0.6, 0.9, 0.6),
        Color(0.6, 0.6, 0.9),
        Color(0.9, 0.9, 0.6),
        Color(0.9, 0.6, 0.9),
        Color(0.6, 0.9, 0.9)
    };
    
    createHexagonalRoom(scene, customWallColors);
    
    scene.addSurface(std::make_unique<Sphere>(
        Vector3(5, 2, -2), 1.5, PERFECT_REFLECTOR, Color(0.9, 0.9, 0.9)));
    
    scene.addSurface(ShapeFactory::createPyramid(
        Vector3(9, -2, -4), 2.0, 3.0, LAMBERTIAN, Color(0.2, 0.2, 0.8)));
    
    scene.addSurface(std::make_unique<Rectangle>(
        Vector3(3, -1, 4.9), Vector3(0, 2, 0), Vector3(2, 0, 0),
        LIGHT_SOURCE, Color(0.95,0.95,0.95)));
    
    std::vector<Color> image(width * height);
    
    std::cout << "Rendering " << width << "x" << height 
              << " image with " << samples << " samples per pixel..." << std::endl;

    std::atomic<int> nextRow{0};
    std::atomic<int> completedRows{0};
    const unsigned int threadCount = std::max(1u, std::thread::hardware_concurrency());

    std::mutex progressMutex;

    auto worker = [&](int threadIndex) {
        std::uniform_real_distribution<double> jitterDist(-0.5, 0.5);

        while (true) {
            int y = nextRow.fetch_add(1);
            if (y >= height) break;
            
            for (int x = 0; x < width; x++) {
                Color pixelColor(0, 0, 0);
                
                for (int s = 0; s < samples; s++) {
                    double jitterX = jitterDist(scene.getRng());
                    double jitterY = jitterDist(scene.getRng());
                    
                    double u = (x + 0.5 + jitterX) / width;
                    double v = (y + 0.5 + jitterY) / height;
                    
                    Ray ray = camera.getRay(u, v);
                    pixelColor = pixelColor + scene.traceRay(ray);
                }
                pixelColor = pixelColor * (1.0 / samples);
                image[y * width + x] = pixelColor.clamp();
            }
            int done = completedRows.fetch_add(1) + 1;
            if (done % 10 == 0 || done == height) {
                std::lock_guard<std::mutex> lock(progressMutex);
                double progress = 100.0 * done / height;
                std::cout << "\rProgress: " << std::fixed << std::setprecision(1) 
                          << progress << "%" << std::flush;
            }
        }
    };

    std::vector<std::thread> threads;
    threads.reserve(threadCount);
    for (unsigned int i = 0; i < threadCount; ++i) {
        threads.emplace_back(worker, i);
    }

    for (auto& t : threads) {
        t.join();
    }

    std::cout << "\rProgress: 100.0%" << std::endl;
    
    std::ofstream file("raytracer_output.ppm");
    if (!file.is_open()) {
        std::cerr << "Error: Could not open output file!" << std::endl;
        return 1;
    }
    
    file << "P3\n" << width << " " << height << "\n255\n";
    
    for (int y = height - 1; y >= 0; y--) {
        for (int x = 0; x < width; x++) {
            Color c = image[y * width + x];
            int r = static_cast<int>(255 * c.r);
            int g = static_cast<int>(255 * c.g);
            int b = static_cast<int>(255 * c.b);
            file << r << " " << g << " " << b << "\n";
        }
    }
    
    file.close();
    std::cout << "Rendering complete using " << threadCount << " threads! Output saved to raytracer_output.ppm" << std::endl;
    
    return 0;
}


