#include "math.cuh"

CUDA_HOSTDEV Vector::Vector(double xPos, double yPos, double zPos) : p{xPos, yPos, zPos} {}

CUDA_HOSTDEV Vector::Vector(const Point &point) : p{point} {}

double CUDA_HOSTDEV Vector::getX() const {
    return p.x;
}

double CUDA_HOSTDEV Vector::getY() const {
    return p.y;
}

double CUDA_HOSTDEV Vector::getZ() const {
    return p.z;
}

double CUDA_HOSTDEV dot(const Vector& v1, const Vector& v2) {
    return v1.getX() * v2.getX() + v1.getY() * v2.getY() + v1.getZ() * v2.getZ();
}

Vector CUDA_HOSTDEV computeSphereNormal(const Point &p1, const Point &c) {
    Vector normal(p1 - c);
    normal.normalize();
    return normal;
}

double CUDA_HOSTDEV clamp(double value, double min_value, double max_value) {
    return max(min_value, std::min(value, max_value));
}

Point CUDA_HOSTDEV Vector::getVectorPoint() const {return p;}

double CUDA_HOSTDEV Vector::mag() const {
    return sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));
}
void CUDA_HOSTDEV Vector::normalize() {
    double mag = this->mag();
    if (mag == 0.0) {
        printf("Warning: Cannot normalize a zero vector\n");
    }
    p.x /= mag;
    p.y /= mag;
    p.z /= mag;
}

CUDA_HOSTDEV BVHNode* buildBVH(Sphere* spheres, int start, int end, int depth) {
    BVHNode* node = new BVHNode();

    // Compute bounding box for current node
    for (int i = start; i < end; ++i) {
        node->bbox.expandToInclude(spheres[i]);
    }

    int numSpheres = end - start;
    if (numSpheres == 1) {
        // Leaf node
        node->spheres = &spheres[start];
        node->sphereCount = numSpheres;
    } else {
        int axis = depth % 3;
        std::nth_element(&spheres[start], &spheres[start + numSpheres / 2], 
                         &spheres[end], [axis](const Sphere& a, const Sphere& b) {
                             if (axis == 0)
                                 return a.c.x < b.c.x;
                             else if (axis == 1)
                                 return a.c.y < b.c.y;
                             else
                                 return a.c.z < b.c.z;
                         });

        int mid = start + numSpheres / 2;
        node->left = buildBVH(spheres, start, mid, depth + 1);
        node->right = buildBVH(spheres, mid, end, depth + 1);
    }

    return node;
}

bool CUDA_HOSTDEV intersectSphere(Ray& ray, Sphere& sphere, Intersection& intersection) {
    Vector diff(sphere.c - ray.origin);
    bool inside = pow(diff.mag(), 2.0) < pow(sphere.r, 2.0);
    double tc = dot(diff, ray.direction) / ray.direction.mag();
    if (!inside && tc < 0) false;
    Point d = ray.origin + tc * ray.direction.getVectorPoint() - sphere.c;
    double d2 = pow(Vector(d).mag(), 2.0);
    if (!inside && pow(sphere.r, 2.0) < d2) return false;
    double tOffset = sqrt(pow(sphere.r, 2) - d2) / ray.direction.mag();
    double t = 0.0;
    intersection.found = true;
    t = inside ? tc + tOffset : tc - tOffset;
    if (t < intersection.t) {
        intersection.t = t;
        intersection.c = sphere.color;
        intersection.center = sphere.c;
    }
    if (intersection.found == true) 
        intersection.p = intersection.t * ray.direction.getVectorPoint() + ray.origin;
    return intersection.found == true && intersection.t > 0.0;
}

CUDA_HOSTDEV bool intersectBVH(const BVHNode* node, Ray& ray, Intersection& closestIntersection) {
    if (node == nullptr) return false;

    double tMin = 0.0, tMax = DBL_MAX;
    if (!node->bbox.intersect(ray, tMin, tMax)) {
        return false; 
    }

    if (node->isLeaf()) {
        bool hit = false;
        for (int i = 0; i < node->sphereCount; ++i) {
            if (intersectSphere(ray, node->spheres[i], closestIntersection)) {
                hit = true;
            }
        }
        return hit;
    } else {
        bool hitLeft = intersectBVH(node->left, ray, closestIntersection);
        bool hitRight = intersectBVH(node->right, ray, closestIntersection);
        return hitLeft || hitRight;
    }
}

