#ifndef RAYTRACER_H_INCLUDED
#define RAYTRACER_H_INCLUDED

#include <vector>
#include <math.h>
#include "algebra3.h"

#define PI 3.1415926535898
#define Tan(th) tan(PI/180*(th))
#define Cos(th) cos(PI/180*(th))

typedef vec3 ambient;
typedef vec3 diffuse;
typedef vec3 speculr;

class RayTracer {
public:
    RayTracer(vec3 start, vec3 direct, vec3 vertical, int angle, int resolutionW, int resolutionH) {
        start_       = start;
        direct_      = direct;
        vertical_    = vertical;
        angle_       = angle;
        resolutionW_ = resolutionW;
        resolutionH_ = resolutionH;

        horizon_     = vertical_ ^ direct_;
        direct_      = direct_.normalize();
        vertical_    = vertical_.normalize();
        horizon_     = horizon_.normalize();

        horizonU_    = horizon_  * ( direct_.length() * Tan(angle_/2) / (resolutionW_/2) );
        verticalU_   = vertical_ * ( direct_.length() * Tan(angle_/2) / (resolutionH_/2) );
        directU_     = direct_ - resolutionW_/2 * horizon_ + resolutionH_/2 * vertical_;
    }
    RayTracer(vec3 start, vec3 direct, int count) {
        start_  = start;
        direct_ = direct;
        count_  = count;
    }
    bool isIntersectedSphere(std::vector<vec4>&);      /// not done, must return distance to check the first impact object
    bool isIntersectedTriangle(std::vector<vec3>&);    /// not done, must return distance to check the first impact object
    vec3 firstRayGenerator(int, int);
    vec3 raytracing(vec3, vec3, int);
private:
    int  count_;
    int  angle_, resolutionW_, resolutionH_;
    vec3 start_, direct_, horizon_, vertical_;
    vec3 directU_, horizonU_, verticalU_;
    vec3 ka_, kd_, ks_, intanse_;
    double distance_, exponential_;
    vec3 collectColor_;
};

bool RayTracer::isIntersectedSphere(std::vector<vec4> &sphere) {
    for(std::size_t i = 0; i < sphere.size(); i += 1) {
        double A = direct_[0]*direct_[0]+direct_[1]*direct_[1]+direct_[2]*direct_[2];
        double B = 2*(direct_[0]*(start_[0]-sphere[i][0])+direct_[1]*(start_[1]-sphere[i][1])+direct_[2]*(start_[2]-sphere[i][2]));
        double C = (start_[0]-sphere[i][0])*(start_[0]-sphere[i][0])+(start_[1]-sphere[i][1])*(start_[1]-sphere[i][1])+(start_[2]-sphere[i][2])*(start_[2]-sphere[i][2])-sphere[i][3]*sphere[i][3];

        if ( ( B*B - 4*A*C ) >= 0 ) {
            return true;
        }
    }
    return false;
}

bool RayTracer::isIntersectedTriangle(std::vector<vec3> &triangle) {
    for(std::size_t i = 0; i < triangle.size()/3; i += 1) {
        vec3 S1 = triangle[3*i+1] - triangle[3*i];
        vec3 S2 = triangle[3*i+2] - triangle[3*i];
        vec3 normal  = S1 ^ S2;

        if (normal * direct_ > 0) { normal = normal * -1; }

        normal = normal.normalize();

        /// there is no triangle
        if( normal == vec3(0, 0, 0) ) { return false; }

        /// ray is not in the triangle plane
        if( direct_ * normal == 0 ) { return false; }

        /// ray is away from the triangle, ax + by + cz = d
        double d = normal[0]*triangle[3*i][0] + normal[1]*triangle[3*i][1] + normal[2]*triangle[3*i][2];
        double t = ( d - ( normal[0]*start_[0] + normal[1]*start_[1] + normal[2]*start_[2] ) ) / ( normal[0]*direct_[0] + normal[1]*direct_[1] + normal[2]*direct_[2] );
        if ( t < 0.0 ) { return false; }

        /// inside test
        double inaccuracy = 0.0001;
        vec3 checkingPoint = vec3(start_[0] + t*direct_[0], start_[1] + t*direct_[1], start_[2] + t*direct_[2]);

        vec3 S3 = checkingPoint - triangle[3*i];
        vec3 S4 = triangle[3*i+1] - checkingPoint;
        vec3 S5 = triangle[3*i+2] - checkingPoint;

        vec3 v31 = S1 ^ S3; double tri31 = v31.length() / 2;
        vec3 v32 = S2 ^ S3; double tri32 = v32.length() / 2;
        vec3 v12 = S1 ^ S2; double tri12 = v12.length() / 2;
        vec3 v45 = S4 ^ S5; double tri45 = v45.length() / 2;

        if ( (tri31 + tri32 + tri45 - tri12) > inaccuracy ) { return false; }
        return true;
    }
    return false;
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

class Ray {
public:
    Ray(vec3 start, vec3 direct) {
        start_  = start;
        direct_ = direct;
    }
    vec3 start()  {return start_;}
    vec3 direct() {return direct_;}
private:
    vec3 start_;
    vec3 direct_;
};


#endif // RAYTRACER_H_INCLUDED
