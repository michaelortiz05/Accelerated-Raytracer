#ifndef IMAGE_H
#define IMAGE_H
#include <iostream>
#include "math.cuh"
std::ostream& operator<<(std::ostream& out, const Color& c);

// Image class for rendering the scene
class Image {
    private:
        int width;
        int height;
        double maxDim;
        std::string name;
        Color currentColor;
        std::vector<unsigned char> png;
        std::vector<Sphere> spheres;
        std::vector<Sun> suns;
        Point* eye;
        Vector* forward;
        Vector* right;
        Vector* up;
        BVHNode* bvh;
        Intersection getSphereCollision(const Ray &ray) const;
        void colorPixel(int x, int y, Color &c);
        void computeColor(Vector& normal, Color& c, Point& p);
        bool isInShadow(const Point &intersection, const Sun &sun);
    public:
        Image(int w = 0, int h = 0, std::string n = ""); 
        ~Image();
        int getHeight();
        Color getColor();
        int getWidth();
        std::vector<unsigned char> const &getPng();
        std::string const &getName();
        void setColor(double r, double g, double b, double a = 1.0);
        void addSphere(double x, double y, double z, double r);
        void addSun(double x, double y, double z);
        void createBVH();
        // void printSun();
        void castRays();
        void printObjects();
        static void convertLinearTosRGB(Color &c);
};
#endif
