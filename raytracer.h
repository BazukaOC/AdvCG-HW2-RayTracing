#ifndef RAYTRACER_H_INCLUDED
#define RAYTRACER_H_INCLUDED

#include <iostream> /// for testing
#include <vector>
#include <math.h>
#include <algorithm>
#include "algebra3.h"

#define PI 3.1415926535898
#define Tan(th) tan(PI/180*(th))
#define Cos(th) cos(PI/180*(th))

typedef float Position;

class Data {
public:
    bool  isIntersected = false;
    bool  isIntersectedSphere = false;
    bool  isIntersectedTriangle = false;

    Pixel color = {255, 255, 255};
    float distance = -1.0;

    vec3  hitpoint;
    vec3  startToHitpoint;
    vec3  hitpointToLight;
    vec3  normal;
    vec3  half;
//    vec3  reflect;
//    vec3  refract;
};

class cTriangle {
public:
    Position x1, y1, z1, x2, y2, z2, x3, y3, z3;
    int r, g, b;
    Pixel color;
};

class cSphere {
public:
    Position x, y, z, r;
    int red, g, b;
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
    RayTracer(vec3 start,
              vec3 direct,
              vec3 vertical,
              int angle,
              int resolutionW, int resolutionH,
              std::vector<cTriangle> triangle,
              std::vector<cSphere> sphere,
              std::vector<cLight> light,
              float exponential) {

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

        horizonU_    = horizon_  * (direct_.length() * Tan(angle_/2) / (resolutionW_/2));
        verticalU_   = vertical_ * (direct_.length() * Tan(angle_/2) / (resolutionH_/2));
        directU_     = direct_ - resolutionW_/2 * horizonU_ + resolutionH_/2 * verticalU_;

        triangle_    = triangle;
        sphere_      = sphere;
        light_       = light;

        ka_ = vec3(0.350, 0.300, 0.150);
        kd_ = vec3(0.600, 0.600, 0.600);
        ks_ = vec3(1.000, 1.000, 1.000);
        exponential_ = exponential;
    }
    RayTracer(vec3 start, vec3 direct, int count) {
        start_  = start;
        direct_ = direct;
        count_  = count;
    }
    Pixel phongModelColor(Data&);
    Data  isIntersectedSphere(vec4&, Ray&);
    Data  isIntersectedTriangle(cTriangle&, Ray&);
    Data  isIntersected(Ray&);

    vec3  firstRayGenerator(int, int);
    vec3  raytracing(vec3, vec3, int);

private:
    int   count_;
    int   angle_, resolutionW_, resolutionH_;
    vec3  start_, direct_, horizon_, vertical_, half_;
    vec3  directU_, horizonU_, verticalU_;

    std::vector<cTriangle> triangle_;
    std::vector<cSphere>   sphere_;
    std::vector<cLight>    light_;

    vec3  ka_, kd_, ks_;
    float exponential_;
};

Pixel RayTracer::phongModelColor(Data &data) {
    float Id = data.normal * data.hitpointToLight, Is = pow(data.normal * data.half, exponential_);
    float Ia = 2.2;
    float intansity = 0.45;
    vec3 color = ka_ * Ia + (kd_ * Id + ks_ * Is) * intansity;
    int r = color[0] * 255, g = color[1] * 255, b = color[2] * 255;
    r = r > 255 ? 255 : r; r = r < 0 ? 0 : r;
    g = g > 255 ? 255 : g; g = g < 0 ? 0 : g;
    b = b > 255 ? 255 : b; b = b < 0 ? 0 : b;
    Pixel colorP = {r, g, b};
    return colorP;
}

Data RayTracer::isIntersectedSphere(vec4 &sphere, Ray &ray) {
    Data data;
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
        float t = ( -B - sqrt(B*B - 4*A*C) ) / 2;

        data.isIntersected         = true;
        data.isIntersectedSphere   = true;
        data.isIntersectedTriangle = false;

        data.hitpoint              = vec3(ray.start[0] + t*ray.direct[0], ray.start[1] + t*ray.direct[1], ray.start[2] + t*ray.direct[2]);
        data.startToHitpoint       = data.hitpoint - ray.start;
        data.hitpointToLight       = vec3(light_[0].x, light_[0].y, light_[0].z) - data.hitpoint;
        data.distance              = data.startToHitpoint.length();
        data.normal                = data.hitpoint - vec3(sphere[0], sphere[1], sphere[2]); data.normal = data.normal.normalize();
        data.half                  = (data.hitpointToLight - ray.direct) / 2;
        data.color                 = phongModelColor(data);
        return data;
    }
    return data;
}

Data RayTracer::isIntersectedTriangle(cTriangle &triangle, Ray &ray) {
    Data data;
    vec3 S1 = vec3(triangle.x2 - triangle.x1, triangle.y2 - triangle.y1, triangle.z2 - triangle.z1);
    vec3 S2 = vec3(triangle.x3 - triangle.x1, triangle.y3 - triangle.y1, triangle.z3 - triangle.z1);
    vec3 normal  = S1 ^ S2;

    if (normal * ray.direct > 0) { normal = normal * -1; }

    normal = normal.normalize();

    /// there is no triangle
    if( normal == vec3(0, 0, 0) ) { return data; }

    /// ray is not in the triangle plane
    if( ray.direct * normal == 0 ) { return data; }

    /// ray is away from the triangle, ax + by + cz = d

    float d = normal[0]*triangle.x1+
              normal[1]*triangle.y1+
              normal[2]*triangle.z1;

    float t = ( d - ( normal[0]*ray.start[0]  + normal[1]*ray.start[1]  + normal[2]*ray.start[2]  ) ) /
                    ( normal[0]*ray.direct[0] + normal[1]*ray.direct[1] + normal[2]*ray.direct[2] );

    if ( t < 0.0 ) { return data; }

    /// inside test
    float inaccuracy = 0.0001;
    vec3 checkingPoint = vec3(ray.start[0] + t*ray.direct[0], ray.start[1] + t*ray.direct[1], ray.start[2] + t*ray.direct[2]);

    vec3 S3 = checkingPoint - vec3(triangle.x1, triangle.y1, triangle.z1);
    vec3 S4 = vec3(triangle.x2, triangle.y2, triangle.z2) - checkingPoint;
    vec3 S5 = vec3(triangle.x3, triangle.y3, triangle.z3) - checkingPoint;

    vec3 v31 = S1 ^ S3; float tri31 = v31.length() / 2;
    vec3 v32 = S2 ^ S3; float tri32 = v32.length() / 2;
    vec3 v12 = S1 ^ S2; float tri12 = v12.length() / 2;
    vec3 v45 = S4 ^ S5; float tri45 = v45.length() / 2;

    if ( (tri31 + tri32 + tri45 - tri12) > inaccuracy ) { return data; }

    data.isIntersected         = true;
    data.isIntersectedSphere   = false;
    data.isIntersectedTriangle = true;

    data.hitpoint              = checkingPoint;
    data.startToHitpoint       = data.hitpoint - ray.start;
    data.hitpointToLight       = vec3(light_[0].x, light_[0].y, light_[0].z) - data.hitpoint;
    data.distance              = data.startToHitpoint.length();
    data.normal                = normal;
    data.half                  = (data.hitpointToLight - ray.direct) / 2;
//    data.color                 = phongModelColor(data);
    data.color                 = {triangle.r, triangle.g, triangle.b};
    return data;
}

Data RayTracer::isIntersected(Ray &ray) {
    Data best;
    best.distance = 100;
    for(std::size_t i = 0; i < sphere_.size(); i += 1) {
        vec4 tempV = vec4(sphere_[i].x, sphere_[i].y, sphere_[i].z, sphere_[i].r);
        Data tempD = isIntersectedSphere(tempV, ray);
        if(tempD.distance < best.distance && tempD.isIntersected) {
            best = tempD;
        }
    }
    for(std::size_t i = 0; i < triangle_.size(); i += 1) {
        Data tempD = isIntersectedTriangle(triangle_[i], ray);
        if(tempD.distance < best.distance && tempD.isIntersected) {
            best = tempD;
        }
    }
    return best;
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
