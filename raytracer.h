#ifndef RAYTRACER_H_INCLUDED
#define RAYTRACER_H_INCLUDED

#include <math.h>
#include "algebra3.h"

typedef vec3 ambient;
typedef vec3 diffuse;
typedef vec3 speculr;

class raytracer {
public:
    raytracer(vec3 start, vec3 direct, vec3 vertical, int angle, int resolutionW, int resolutionH) {
        start_       = start;
        direct_      = direct;
        vertical_    = vertical;
        angle_       = angle;
        resolutionW_ = resolutionW;
        resolutionH_ = resolutionH;

        horizon_  = vertical_ ^ direct_;
        direct_   = direct_.normalize();
        vertical_ = vertical_.normalize();
        horizon_  = horizon_.normalize();
    }
    raytracer(vec3 start, vec3 direct, int count) {
        start_  = start;
        direct_ = direct;
        count_  = count;
    }
    bool isIntersectedSphere(vec4);      /// not done, must return distance to check the first impact object
    bool isIntersectedTriangle(vec3[3]); /// not done, must return distance to check the first impact object
    vec3 firstRayGenerator(int, int);
    vec3 raytracing(vec3, vec3, int);
private:
    int  count_, color_[3];
    int  angle_, resolutionW_, resolutionH_;
    vec3 start_, direct_, horizon_, vertical_;
    vec3 ka_, kd_, ks_, intanse_;
};

bool raytracer::isIntersectedSphere(vec4 sphere) {
    ///  AT^2 + BT + C = 0
    double A = direct_[0]*direct_[0]+direct_[1]*direct_[1]+direct_[2]*direct_[2];
    double B = 2*(direct_[0]*(start_[0]-sphere[0])+direct_[1]*(start_[1]-sphere[1])+direct_[2]*(start_[2]-sphere[2]));
    double C = (start_[0]-sphere[0])*(start_[0]-sphere[0])+(start_[1]-sphere[1])*(start_[1]-sphere[1])+(start_[2]-sphere[2])*(start_[2]-sphere[2])-sphere[3]*sphere[3];

    if ( ( B*B - 4*A*C ) >= 0 ) {
        ///  T = ( -B +- (B^2 - 4*C)^1/2 ) / 2
//        float t = ( -B - sqrt(B*B - 4*A*C) ) / 2;
//        checkingPoint = vec3(start_[0] + t*direct_[0], start_[1] + t*direct_[1], start_[2] + t*direct_[2]);
//        normal = checkingPoint - vec3(sphere[0], sphere[1], sphere[2]);
//        normal = normal.normalize();
        return true;
    }
    return false;
}

bool raytracer::isIntersectedTriangle(vec3 tri[3]) {

    vec3 S1 = tri[1] - tri[0];
    vec3 S2 = tri[2] - tri[0];
    vec3 normal  = S1 ^ S2;

    if (normal * direct_ > 0) { normal = normal * -1; }

    normal = normal.normalize();

    /// there is no triangle
    if( normal == vec3(0, 0, 0) ) { return false; }

    /// ray is not in the triangle plane
    if( direct_ * normal == 0 ) { return false; }

    /// ray is away from the triangle, ax + by + cz = d
    double d = normal[0]*tri[0][0] + normal[1]*tri[0][1] + normal[2]*tri[0][2];
    double t = ( d - ( normal[0]*start_[0] + normal[1]*start_[1] + normal[2]*start_[2] ) ) / ( normal[0]*direct_[0] + normal[1]*direct_[1] + normal[2]*direct_[2] );
    if ( t < 0.0 ) { return false; }

    /// inside test
    double inaccuracy = 0.0001;
    vec3 checkingPoint = vec3(start_[0] + t*direct_[0], start_[1] + t*direct_[1], start_[2] + t*direct_[2]);

    vec3 S3 = checkingPoint - tri[0];
    vec3 S4 = tri[1] - checkingPoint;
    vec3 S5 = tri[2] - checkingPoint;

    vec3 v31 = S1 ^ S3; double tri31 = v31.length() / 2;
    vec3 v32 = S2 ^ S3; double tri32 = v32.length() / 2;
    vec3 v12 = S1 ^ S2; double tri12 = v12.length() / 2;
    vec3 v45 = S4 ^ S5; double tri45 = v45.length() / 2;

    if ( (tri31 + tri32 + tri45 - tri12) > inaccuracy ) { return false; }

    return true;
}

vec3 raytracer::firstRayGenerator(int x, int y) {
    /// return the Color
    return vec3(0, 0, 0);
}

vec3 raytracer::raytracing(vec3 start, vec3 direct, int count) {
    if(count == 0) {
        return vec3(0, 0, 0);
    } else {
        return raytracing(start, direct, count - 1);
    }
}

#endif // RAYTRACER_H_INCLUDED
