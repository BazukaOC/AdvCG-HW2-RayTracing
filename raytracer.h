#ifndef RAYTRACER_H_INCLUDED
#define RAYTRACER_H_INCLUDED

#include <iostream> /// for testing
#include <vector>
#include <math.h>
#include "algebra3.h"

#define PI 3.1415926535898
#define Tan(th) tan(PI/180*(th))
#define Cos(th) cos(PI/180*(th))

typedef float Position;

class Data {
public:
    Pixel color;
    float ditance;
    bool  isIntersected;
    bool  isIntersectedSphere;
    bool  isIntersectedTriangle;
    vec3  hitpoint;
    vec3  normal;
    vec3  reflect;
    vec3  refract;
};

class cTriangle {
public:
    Position x1, y1, z1, x2, y2, z2, x3, y3, z3;
    Pixel color;
};

class cSphere {
public:
    Position x, y, z, r;
    Pixel color;
};

class cLight {
public:
    Position x, y, z;
    Pixel color;
};

class Ray {
public:
    Ray(vec3 istart, vec3 idirect){
        start  = istart;
        direct = idirect;
    }
    vec3 start;
    vec3 direct;
};

class RayTracer {
public:
    RayTracer(vec3 start, vec3 direct, vec3 vertical, int angle, int resolutionW, int resolutionH,
              std::vector<cTriangle> triangle,
              std::vector<cSphere>   sphere,
              std::vector<cLight>    light ) {

        start_       = start;
        direct_      = direct;
        vertical_    = vertical;
        angle_       = angle;
        resolutionW_ = resolutionW;
        resolutionH_ = resolutionH;

        horizon_     = vertical_ ^ direct_;
        direct_      =   direct_.normalize();
        vertical_    = vertical_.normalize();
        horizon_     =  horizon_.normalize();

        horizonU_    = horizon_  * ( direct_.length() * Tan(angle_/2) / (resolutionW_/2) );
        verticalU_   = vertical_ * ( direct_.length() * Tan(angle_/2) / (resolutionH_/2) );
        directU_     = direct_ - resolutionW_/2 * horizonU_ + resolutionH_/2 * verticalU_;

        triangle_ = triangle;
        sphere_   = sphere;
        light_    = light;

        ka_ = vec3(0.100, 0.100, 0.100);
        kd_ = vec3(0.700, 0.600, 0.300);
        ks_ = vec3(1.000, 1.000, 1.000);
    }
    RayTracer(vec3 start, vec3 direct, int count) {
        start_  = start;
        direct_ = direct;
        count_  = count;
    }

    bool isIntersectedSphere(vec4&, Ray&);      /// not done, must return distance to check the first impact object
    bool isIntersectedTriangle(std::vector<vec3>&, Ray&);    /// not done, must return distance to check the first impact object
    bool isIntersectedAllSphere(Ray&);
    bool isIntersectedAllTriangle(Ray&);
    Data isIntersected(Ray&);

    vec3 firstRayGenerator(int, int);
    vec3 raytracing(vec3, vec3, int);
private:
    int   count_;
    int   angle_, resolutionW_, resolutionH_;
    vec3  start_, direct_, horizon_, vertical_, half_;
    vec3  directU_, horizonU_, verticalU_;

    std::vector<cTriangle> triangle_;
    std::vector<cSphere>   sphere_;
    std::vector<cLight>    light_;

    vec3  ka_, kd_, ks_, intansity_;
    float ia_, id_, is_, ii_, distance_, exponential_;
    vec3  collectColor_;

/// intansity = Ka*Ia + Kd*Id + Ks*Is
/// Ka = {r, g, b}, Ia = Ka
/// Kd = {r, g, b}, Id = normal * point_to_light
/// Ks = {r, g, b}, Is = pow(normal * half, exponential)
/// half = (eye_to_point + point_to_light) / 2
};

bool RayTracer::isIntersectedSphere(vec4 &sphere, Ray &ray) {

    float A = ray.direct[0]*ray.direct[0]+
              ray.direct[1]*ray.direct[1]+
              ray.direct[2]*ray.direct[2];

    float B = 2*(ray.direct[0]*(ray.start[0]-sphere[0])+
                 ray.direct[1]*(ray.start[1]-sphere[1])+
                 ray.direct[2]*(ray.start[2]-sphere[2]));

    float C = (ray.start[0]-sphere[0])*(ray.start[0]-sphere[0])+
              (ray.start[1]-sphere[1])*(ray.start[1]-sphere[1])+
              (ray.start[2]-sphere[2])*(ray.start[2]-sphere[2])-sphere[3]*sphere[3];


    if ( ( B*B - 4*A*C ) >= 0 ) {
        return true;
    }
    return false;
}

bool RayTracer::isIntersectedTriangle(std::vector<vec3> &triangle, Ray &ray) {
    vec3 S1 = triangle[1] - triangle[0];
    vec3 S2 = triangle[2] - triangle[0];
    vec3 normal  = S1 ^ S2;

    if (normal * ray.direct > 0) { normal = normal * -1; }

    normal = normal.normalize();

    /// there is no triangle
    if( normal == vec3(0, 0, 0) ) { return false; }

    /// ray is not in the triangle plane
    if( ray.direct * normal == 0 ) { return false; }

    /// ray is away from the triangle, ax + by + cz = d

    float d = normal[0]*triangle[0][0]+
              normal[1]*triangle[0][1]+
              normal[2]*triangle[0][2];

    float t = ( d - ( normal[0]*ray.start[0]  + normal[1]*ray.start[1]  + normal[2]*ray.start[2]  ) ) /
                    ( normal[0]*ray.direct[0] + normal[1]*ray.direct[1] + normal[2]*ray.direct[2] );

    if ( t < 0.0 ) { return false; }

    /// inside test
    float inaccuracy = 0.0001;
    vec3 checkingPoint = vec3(ray.start[0] + t*ray.direct[0], ray.start[1] + t*ray.direct[1], ray.start[2] + t*ray.direct[2]);

    vec3 S3 = checkingPoint - triangle[0];
    vec3 S4 = triangle[1] - checkingPoint;
    vec3 S5 = triangle[2] - checkingPoint;

    vec3 v31 = S1 ^ S3; float tri31 = v31.length() / 2;
    vec3 v32 = S2 ^ S3; float tri32 = v32.length() / 2;
    vec3 v12 = S1 ^ S2; float tri12 = v12.length() / 2;
    vec3 v45 = S4 ^ S5; float tri45 = v45.length() / 2;

    if ( (tri31 + tri32 + tri45 - tri12) > inaccuracy ) { return false; }
    return true;
}

bool RayTracer::isIntersectedAllSphere(Ray &ray) {
    for(std::size_t i = 0; i < sphere_.size(); i += 1) {
        vec4 temp = vec4(sphere_[i].x, sphere_[i].y, sphere_[i].z, sphere_[i].r);
        if(isIntersectedSphere(temp, ray)) {
            return true;
        }

    }

    return false;
}

bool RayTracer::isIntersectedAllTriangle(Ray &ray) {
    for(std::size_t i = 0; i < triangle_.size(); i += 1) {
        std::vector<vec3> temp;
        vec3 p1  = vec3(triangle_[i].x1, triangle_[i].y1, triangle_[i].z1); temp.push_back(p1);
        vec3 p2  = vec3(triangle_[i].x2, triangle_[i].y2, triangle_[i].z2); temp.push_back(p2);
        vec3 p3  = vec3(triangle_[i].x3, triangle_[i].y3, triangle_[i].z3); temp.push_back(p3);
        if(isIntersectedTriangle(temp, ray)) {
            return true;
        }
    }
    return false;
}

Data RayTracer::isIntersected(Ray &ray) {
    Data data;
    if(isIntersectedAllSphere(ray)) {
        data.isIntersected         = true;
        data.isIntersectedSphere   = true;
        data.isIntersectedTriangle = false;
        data.color = {160, 140, 120};
    } else if(isIntersectedAllTriangle(ray)) {
        data.isIntersected         = true;
        data.isIntersectedSphere   = false;
        data.isIntersectedTriangle = true;
        data.color = {255, 255, 255};
    } else {
        data.isIntersected         = false;
        data.isIntersectedSphere   = false;
        data.isIntersectedTriangle = false;
        data.color = {0, 0, 0};
    }
    return data;
}

vec3 RayTracer::firstRayGenerator(int x, int y) {
    vec3 firstRay = directU_ + x * horizonU_ - y * verticalU_;
    firstRay = firstRay.normalize();
    return firstRay;
}

vec3 RayTracer::raytracing(vec3 start, vec3 direct, int count) {
    if(count == 0) {
        return vec3(0, 0, 0);
    } else {
        return raytracing(start, direct, count - 1);
    }
}

#endif // RAYTRACER_H_INCLUDED
