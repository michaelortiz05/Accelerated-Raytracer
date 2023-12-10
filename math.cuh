#ifndef MATH_CUH
#define MATH_CUH
#ifdef __CUDACC__
#define CUDA_HOSTDEV __host__ __device__
#else
#define CUDA_HOSTDEV
#endif
#include <vector>
#include <algorithm>
#include <iterator>
#include <cmath>
#include <cfloat>
#include <limits>

// basic struct for point and point operations
CUDA_HOSTDEV struct Point {
    double x, y, z;

    // Add two points
    CUDA_HOSTDEV Point operator+(const Point &p) const {
        return Point{x + p.x, y + p.y, z + p.z};
    }

    // Subtract two points
    CUDA_HOSTDEV Point operator-(const Point &p) const {
        return Point{x - p.x, y - p.y, z - p.z};
    }

    // Scalar multiplication
    CUDA_HOSTDEV Point operator*(double scalar) const {
        return Point{x * scalar, y * scalar, z * scalar};
    }

    // Scalar division
    CUDA_HOSTDEV Point operator/(double scalar) const {
        if (scalar == 0.0) {
            printf("Warning: Division by zero"); 
            return Point();
        }
        return Point{x / scalar, y / scalar, z / scalar};
    }
};

// simple vector class
class Vector {
    private:
        Point p;
    public:
        CUDA_HOSTDEV Vector(double xPos = 0.0, double yPos = 0.0, double zPos = 0.0);
        CUDA_HOSTDEV Vector(const Point &p);
        CUDA_HOSTDEV double getX() const;
        CUDA_HOSTDEV double getY() const;
        CUDA_HOSTDEV double getZ() const;
        CUDA_HOSTDEV double mag() const;
        // CUDA_HOSTDEV void print() const;
        CUDA_HOSTDEV Point getVectorPoint() const;
        CUDA_HOSTDEV void normalize();

        // Friend operator overloads
        CUDA_HOSTDEV friend Vector operator*(const Vector &v1, const Vector &v2) {
            return Vector(v1.p.x * v2.p.x, v1.p.y * v2.p.y, v1.p.z * v2.p.z);
        }

        CUDA_HOSTDEV friend Vector operator/(const Vector &v1, const Vector &v2) {
            if (v2.p.x == 0.0 || v2.p.y == 0.0 || v2.p.z == 0.0) {
                printf("Division by 0 error");
                return Vector(); 
            }
            return Vector(v1.p.x / v2.p.x, v1.p.y / v2.p.y, v1.p.z / v2.p.z);
        }

        CUDA_HOSTDEV friend Vector operator+(const Vector &v1, const Vector &v2) {
            return Vector(v1.p.x + v2.p.x, v1.p.y + v2.p.y, v1.p.z + v2.p.z);
        }

        CUDA_HOSTDEV friend Vector operator+(const Vector &v, double c) {
            return Vector(v.p.x + c, v.p.y + c, v.p.z + c);
        }

        CUDA_HOSTDEV friend Vector operator-(const Vector &v1, const Vector &v2) {
            return Vector(v1.p.x - v2.p.x, v1.p.y - v2.p.y, v1.p.z - v2.p.z);
        }

        CUDA_HOSTDEV friend Vector operator+(double c, const Vector &v) {
            return Vector(c + v.p.x, c + v.p.y, c + v.p.z);
        }

        CUDA_HOSTDEV friend Vector operator-(double c, const Vector &v) {
            return Vector(c - v.p.x, c - v.p.y, c - v.p.z);
        }

        CUDA_HOSTDEV friend Vector operator-(const Vector &v, double c) {
            return Vector(v.p.x - c, v.p.y - c, v.p.z - c);
        }

        CUDA_HOSTDEV friend Vector operator*(const Vector &v, double c) {
            return Vector(v.p.x * c, v.p.y * c, v.p.z * c);
        }

        CUDA_HOSTDEV friend Vector operator*(double c, const Vector &v) {
            return Vector(c * v.p.x, c * v.p.y, c * v.p.z);
        }

        CUDA_HOSTDEV friend Vector operator/(const Vector &v, double c) {
            return Vector(v.p.x / c, v.p.y / c, v.p.z / c);
        }

        CUDA_HOSTDEV friend Vector operator/(double c, const Vector &v) {
            return Vector(c / v.p.x, c / v.p.y, c / v.p.z);
        }
};

// Struct representing a ray 
struct CUDA_HOSTDEV Ray {
    Point origin;
    Vector direction;
};

// color struct
struct CUDA_HOSTDEV Color {
    double r, g, b, alpha;
};

// Sphere with center c, radius r, and a color
struct CUDA_HOSTDEV Sphere {
    Point c;
    double r;
    Color color;
};

// Sun light source
struct CUDA_HOSTDEV Sun {
    Vector direction;
    Color c;
};

// Intersection computations
struct CUDA_HOSTDEV Intersection {
    Point p;
    double t = DBL_MAX;
    bool found = false;
    Color c;
    Point center;
};

// Simple bounding box for bvh nodes
struct CUDA_HOSTDEV BoundingBox {
    Point minCorner, maxCorner;

    BoundingBox() 
        : minCorner{std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()}, 
          maxCorner{-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max()} {}

    void expandToInclude(const Sphere& sphere) {
        minCorner.x = std::min(minCorner.x, sphere.c.x - sphere.r);
        minCorner.y = std::min(minCorner.y, sphere.c.y - sphere.r);
        minCorner.z = std::min(minCorner.z, sphere.c.z - sphere.r);

        maxCorner.x = std::max(maxCorner.x, sphere.c.x + sphere.r);
        maxCorner.y = std::max(maxCorner.y, sphere.c.y + sphere.r);
        maxCorner.z = std::max(maxCorner.z, sphere.c.z + sphere.r);
    }

    bool intersect(const Ray& ray, double& tMin, double& tMax) const {
        for (int i = 0; i < 3; ++i) {
            double invD = 1.0 / (i == 0 ? ray.direction.getX() : (i == 1 ? ray.direction.getY() : ray.direction.getZ()));
            double t0 = ((i == 0 ? minCorner.x : (i == 1 ? minCorner.y : minCorner.z)) - (i == 0 ? ray.origin.x : (i == 1 ? ray.origin.y : ray.origin.z))) * invD;
            double t1 = ((i == 0 ? maxCorner.x : (i == 1 ? maxCorner.y : maxCorner.z)) - (i == 0 ? ray.origin.x : (i == 1 ? ray.origin.y : ray.origin.z))) * invD;
            if (invD < 0.0) std::swap(t0, t1);
            tMin = (t0 > tMin) ? t0 : tMin;
            tMax = (t1 < tMax) ? t1 : tMax;
            if (tMax <= tMin) return false;
        }
        return true;
    }
};

// BVH tree node
struct CUDA_HOSTDEV BVHNode {
    BoundingBox bbox;
    BVHNode* left;
    BVHNode* right;
    Sphere* spheres; 
    int sphereCount; 

    BVHNode() : left(nullptr), right(nullptr), spheres(nullptr), sphereCount(0) {}
    ~BVHNode() {
        delete left;
        delete right;
    }

    bool isLeaf() const { return sphereCount > 0; }
};

BVHNode* CUDA_HOSTDEV buildBVH(Sphere* spheres, int start, int end, int depth = 0);
double CUDA_HOSTDEV dot(const Vector& v1, const Vector& v2);
Vector CUDA_HOSTDEV computeSphereNormal(const Point &p1, const Point &c);
double CUDA_HOSTDEV clamp(double value, double min_value, double max_value);
bool CUDA_HOSTDEV intersectSphere(Ray& ray, Sphere& sphere, Intersection& intersection);
bool CUDA_HOSTDEV intersectBVH(const BVHNode* node, Ray& ray, Intersection& closestIntersection);
inline Point CUDA_HOSTDEV  operator*(double scalar, const Point &p) {return Point{p.x * scalar, p.y * scalar, p.z * scalar};}

#endif