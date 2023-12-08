#ifndef MATH_CUH
#define MATH_CUH
#include <vector>
#include <algorithm>
#include <iterator>
#include <cmath>
#include <cfloat>
#include <limits>

// basic struct for point and point operations
struct __host __device__ Point {
    double x, y, z;

    // Add two points
    __host__ __device__ Point operator+(const Point &p) const {
        return Point{x + p.x, y + p.y, z + p.z};
    }

    // Subtract two points
    __host__ __device__ Point operator-(const Point &p) const {
        return Point{x - p.x, y - p.y, z - p.z};
    }

    // Scalar multiplication
    __host__ __device__ Point operator*(double scalar) const {
        return Point{x * scalar, y * scalar, z * scalar};
    }

    // Scalar division
    __host__ __device__ Point operator/(double scalar) const {
        if (scalar == 0.0) {
            printf("Warning: Division by zero"); 
            return Point();
        }
        return Point{x / scalar, y / scalar, z / scalar};
    }
};

class Vector {
    private:
        Point p;
    public:
        __host__ __device__ Vector(double xPos = 0.0, double yPos = 0.0, double zPos = 0.0);
        __host__ __device__ Vector(const Point &p);
        __host__ __device__ double mag() const;
        __host__ __device__ void print() const;
        __host__ __device__ void normalize();

        // Friend operator overloads
        __host__ __device__ friend Vector operator*(const Vector &v1, const Vector &v2) {
            return Vector(v1.x * v2.x, v1.y * v2.y, v1.z * v2.z);
        }

        __host__ __device__ friend Vector operator/(const Vector &v1, const Vector &v2) {
            if (v2.x == 0.0 || v2.y == 0.0 || v2.z == 0.0) {
                printf("Division by 0 error");
                return Vector(); 
            }
            return Vector(v1.x / v2.x, v1.y / v2.y, v1.z / v2.z);
        }

        __host__ __device__ friend Vector operator+(const Vector &v1, const Vector &v2) {
            return Vector(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
        }

        __host__ __device__ friend Vector operator+(const Vector &v, double c) {
            return Vector(v.x + c, v.y + c, v.z + c);
        }

        __host__ __device__ friend Vector operator-(const Vector &v1, const Vector &v2) {
            return Vector(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
        }

        __host__ __device__ friend Vector operator+(double c, const Vector &v) {
            return Vector(c + v.x, c + v.y, c + v.z);
        }

        __host__ __device__ friend Vector operator-(double c, const Vector &v) {
            return Vector(c - v.x, c - v.y, c - v.z);
        }

        __host__ __device__ friend Vector operator-(const Vector &v, double c) {
            return Vector(v.x - c, v.y - c, v.z - c);
        }

        __host__ __device__ friend Vector operator*(const Vector &v, double c) {
            return Vector(v.x * c, v.y * c, v.z * c);
        }

        __host__ __device__ friend Vector operator*(double c, const Vector &v) {
            return Vector(c * v.x, c * v.y, c * v.z);
        }

        __host__ __device__ friend Vector operator/(const Vector &v, double c) {
            return Vector(v.x / c, v.y / c, v.z / c);
        }

        __host__ __device__ friend Vector operator/(double c, const Vector &v) {
            return Vector(c / v.x, c / v.y, c / v.z);
        }
};

// Struct representing a ray 
struct __host__ __device__ Ray {
    Point origin;
    Vector direction;
};

// color struct
struct __host__ __device__ Color {
    double r, g, b, alpha;
};

// Sphere with center c, radius r, and a color
struct __host__ __device__ Sphere {
    Point center;
    double r;
    Color color;
};

// Sun light source
struct __host__ __device__ Sun {
    Vector direction;
    Color c;
};

struct __host__ __device__ Intersection {
    Point p;
    double t = DBL_MAX;
    bool found = false;
    Color c;
    Point center;
};

// finish this
double dot(const Vector& v1, const Vector& v2);
Vector computeSphereNormal(const Point &p1, const Point &c);
double clamp(double value, double min_value, double max_value);

#endif