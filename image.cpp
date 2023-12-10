#include "image.h"

// printing overload for color
std::ostream& operator<<(std::ostream& out, const Color& c) { out << "(" << c.r << ", " << c.g << ", " << c.b << ')'; return out;}

// Image constructor
Image::Image(int w, int h, std::string n): width{w}, height{h}, name{n} {
    png.resize(height * width * 4);
    maxDim = std::max(width, height);
    eye = new Point;
    forward = new Vector(0, 0, -1);
    right = new Vector(1, 0, 0);
    up = new Vector(0, 1, 0);
    bvh = nullptr;
}

// Image deconstructor
Image::~Image() {
    delete eye;
    delete forward;
    delete right;
    delete up;
    delete bvh;
}

// Height getter
int Image::getHeight() {return height;}

// Width getter
int Image::getWidth() {return width;}

// Coor getter
Color Image::getColor(){return currentColor;}

// Name getter
std::string const &Image::getName() {return name;}

// Png getter
std::vector<unsigned char> const &Image::getPng() {return png;}

// Set the color
void Image::setColor(double r, double g, double b, double a) {
    currentColor.r = clamp(r, 0.0, 1.0);
    currentColor.g = clamp(g, 0.0, 1.0);
    currentColor.b = clamp(b, 0.0, 1.0);
    currentColor.alpha = clamp(a, 0.0, 1.0);
}

// add sphere to list of objects
void Image::addSphere(double x, double y, double z, double r) {
    spheres.push_back(Sphere{Point{x,y,z}, r, this->currentColor});
}

// Add a new sun
void Image::addSun(double x, double y, double z) {
    Sun newSun;
    Vector d(x,y,z);
    d.normalize();
    newSun.direction = d;
    newSun.c = currentColor;
    suns.push_back(newSun);
}

// // Print the current sun
// void Image::printSun() {
//     if (currentSun != nullptr) {
//         std::cout << "Color: " << currentSun->c << std::endl;
//         std::cout << "Direction: " << std::endl;
//         currentSun->direction.print();
//     }
// }

void Image::createBVH() { bvh = buildBVH(spheres.data(), 0, spheres.size()); }

// Cast rays and draw the scene
void Image::castRays() { 
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            double sx = (2.0 * x - width) / maxDim;
            double sy = (height - 2.0 * y) / maxDim;
            Vector direction = *forward + sx * *right + sy * *up;
            direction.normalize();
            Ray ray = Ray{*eye, direction};
            Intersection intersection;
            if (!intersectBVH(bvh, ray, intersection)) continue;
            Vector normal = computeSphereNormal(intersection.p, intersection.center);
            computeColor(normal, intersection.c, intersection.p);
            colorPixel(x, y, intersection.c);
            // if (intersection.found == true && intersection.t > 0.0) {
            //     Vector normal = computeSphereNormal(intersection.p, intersection.center);
            //     computeColor(normal, intersection.c, intersection.p);
            //     colorPixel(x, y, intersection.c);
            // }
        }
    }
}

// return the ray-sphere collision
Intersection Image::getSphereCollision(const Ray &ray) const {
    Intersection intersection;
    for (auto &object : spheres) {
        Vector diff(object.c - ray.origin);
        bool inside = std::pow(diff.mag(), 2.0) < std::pow(object.r, 2.0);
        double tc = dot(diff, ray.direction) / ray.direction.mag();
        if (!inside && tc < 0) continue;
        Point d = ray.origin + tc * ray.direction.getVectorPoint() - object.c;
        double d2 = std::pow(Vector(d).mag(), 2.0);
        if (!inside && std::pow(object.r, 2.0) < d2) continue;
        double tOffset = std::sqrt(std::pow(object.r, 2) - d2) / ray.direction.mag();
        double t = 0.0;
        intersection.found = true;
        t = inside ? tc + tOffset : tc - tOffset;
        if (t < intersection.t) {
            intersection.t = t;
            intersection.c = object.color;
            intersection.center = object.c;
        }
    }
    if (intersection.found == true) 
        intersection.p = intersection.t * ray.direction.getVectorPoint() + ray.origin;
    return intersection;
}


// helper function to convert color space
void Image::convertLinearTosRGB(Color &c) {
    const double factor = 0.0031308;
    c.r = clamp(c.r, 0.0, 1.0);
    c.g = clamp(c.g, 0.0, 1.0);
    c.b = clamp(c.b, 0.0, 1.0);
    c.r = c.r <= factor ? 12.92 * c.r : 1.055 * std::pow(c.r, (1.0/2.4)) - 0.055;
    c.g = c.g <= factor ? 12.92 * c.g : 1.055 * std::pow(c.g, (1.0/2.4)) - 0.055;
    c.b = c.b <= factor ? 12.92 * c.b: 1.055 * std::pow(c.b, (1.0/2.4)) - 0.055;
}

// compute color with lambert shading for multiple suns
void Image::computeColor(Vector& normal, Color& c, Point& p) {
    Color accumulatedColor{0.0, 0.0, 0.0};
    for (auto& sun : suns) {
        if (!isInShadow(p, sun)) {
            Vector eyeDir(p - *eye);
            eyeDir.normalize();
            if (dot(eyeDir, normal) > 0.0)
                normal = normal * -1;

            sun.direction.normalize();
            double lambert = std::max(dot(normal, sun.direction), 0.0);

            accumulatedColor.r += lambert * sun.c.r;
            accumulatedColor.g += lambert * sun.c.g;
            accumulatedColor.b += lambert * sun.c.b;
        }
    }

    c.r *= accumulatedColor.r;
    c.g *= accumulatedColor.g;
    c.b *= accumulatedColor.b;
}

// print the set of objects in the scene
// void Image::printObjects() {
//     for (auto &object : objects) {
//         std::cout << "Point: (";
//         std::cout << object.c.x << ", ";
//         std::cout << object.c.y << ", ";
//         std::cout << object.c.z << ")" << std::endl;
//         std::cout << "Radius: " << object.r << std::endl;
//         std::cout << "Color: (" << object.color.r << ", ";
//         std::cout << object.color.g << ", ";
//         std::cout << object.color.b << ")" << std::endl;
//     }
// }

// check if a point is in shadow for a given sun
bool Image::isInShadow(const Point &intersection, const Sun &sun) {
    constexpr double bias = 1e-6;
    Vector biasVector = sun.direction * bias;
    Ray shadowRay{intersection + biasVector.getVectorPoint(), sun.direction};
    // Intersection i = getSphereCollision(shadowRay);
    Intersection i;
    return (intersectBVH(bvh, shadowRay, i));
    // return i.found;
}

// color pixel at location
void Image::colorPixel(int x, int y, Color &c) {
    convertLinearTosRGB(c);
    png[((y * width) + x)*4 + 0] = static_cast<unsigned char>(c.r * 255.0);
    png[((y * width) + x)*4 + 1] = static_cast<unsigned char>(c.g * 255.0);
    png[((y * width) + x)*4 + 2] = static_cast<unsigned char>(c.b * 255.0);
    png[((y * width) + x)*4 + 3] = static_cast<unsigned char>(c.alpha * 255.0);
}

