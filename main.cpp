#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include "algebra3.h"
#include "imageIO.h"
#include "raytracer.h"

#define PI 3.1415926535898
#define Tan(th) tan(PI/180*(th))
#define Cos(th) cos(PI/180*(th))

using namespace std;

vec3 eye, dir, vert, hori, light;
float Fangle;
int Rw, Rh;

float SOx, SOy, SOz, Sr;
float Ax, Ay, Az, Bx, By, Bz, Cx, Cy, Cz;

vector<vec4> Sphere;
vector<vec3> Triangle;

vec3 intansity, checkingPoint, normal, half, reflect, refract;

float Ia, Id, Is, Ii, Ke = 1000.0, Nr = 2.0;
vec3 Ka = vec3(0.100, 0.100, 0.100);
vec3 Kd = vec3(0.700, 0.600, 0.300);
vec3 Ks = vec3(1.000, 1.000, 1.000);

void readFile() {
    ifstream ifile("hw1_input.txt");
    string test;
    while( ifile >> test ) {
        switch (test[0]) {
            case 'E':
                ifile >> eye[0] >> eye[1] >> eye[2];
                break;
            case 'V':
                ifile >> dir[0] >> dir[1] >> dir[2];
                break;
            case 'F':
                ifile >> Fangle;
                break;
            case 'R':
                ifile >> Rw >> Rh;
                break;
            case 'S':
                ifile >> SOx >> SOy >> SOz >> Sr;
                Sphere.push_back(vec4(SOx, SOy, SOz, Sr));
                break;
            case 'T':
                ifile >> Ax >> Ay >> Az >> Bx >> By >> Bz >> Cx >> Cy >> Cz;
                Triangle.push_back(vec3(Ax, Ay, Az));
                Triangle.push_back(vec3(Bx, By, Bz));
                Triangle.push_back(vec3(Cx, Cy, Cz));
                break;
            case 'L':
                ifile >> light[0] >> light[1] >> light[2];
        }
    }
}

void setRay() {
    vert = vec3(0, 1, 0);
    hori = vert ^ dir;

    dir  =  dir.normalize();
    vert = vert.normalize();
    hori = hori.normalize();

    hori = hori * ( dir.length() * Tan(Fangle/2) / (Rw/2) );
    vert = vert * ( dir.length() * Tan(Fangle/2) / (Rh/2) );
    dir  =  dir - Rw/2 * hori + Rh/2 * vert;
    ///cout << dir[0] << ", " << dir[1] << ", " << dir[2] << endl;
}

bool isIntersectedSph(vec3 &ray, vec4 &sph, vec3 &seye) {

    ///  AT^2 + BT + C = 0
    float A = ray[0]*ray[0]+ray[1]*ray[1]+ray[2]*ray[2];
    float B = 2*(ray[0]*(seye[0]-sph[0])+ray[1]*(seye[1]-sph[1])+ray[2]*(seye[2]-sph[2]));
    float C = (seye[0]-sph[0])*(seye[0]-sph[0])+(seye[1]-sph[1])*(seye[1]-sph[1])+(seye[2]-sph[2])*(seye[2]-sph[2])-sph[3]*sph[3];

    if ( ( B*B - 4*A*C ) >= 0 ) {
        ///  T = ( -B +- (B^2 - 4*C)^1/2 ) / 2
        float t = ( -B - sqrt(B*B - 4*A*C) ) / 2;
        checkingPoint = vec3(seye[0] + t*ray[0], seye[1] + t*ray[1], seye[2] + t*ray[2]);
        normal = checkingPoint - vec3(sph[0], sph[1], sph[2]);
        normal = normal.normalize();
        return true;
    }
    return false;
}

bool isIntersectedTri(vec3 &ray, vec3 &a, vec3 &b, vec3 &c, vec3 &seye) {

    vec3 S1 = b - a;
    vec3 S2 = c - a;
    normal  = S1 ^ S2;

    if (normal * ray > 0) { normal = normal * -1; }

    normal = normal.normalize();

    /// there is no triangle
    if( normal == vec3(0, 0, 0) ) { return false; }

    /// ray is not in the triangle plane
    if( ray * normal == 0 ) { return false; }

    /// ray is away from the triangle, ax + by + cz = d
    float d = normal[0]*a[0] + normal[1]*a[1] + normal[2]*a[2];
    float t = ( d - ( normal[0]*seye[0] + normal[1]*seye[1] + normal[2]*seye[2] ) ) / ( normal[0]*ray[0] + normal[1]*ray[1] + normal[2]*ray[2] );
    if ( t < 0.0 ) { return false; }

    /// inside test
    float inaccuracy = 0.0001;
    checkingPoint = vec3(seye[0] + t*ray[0], seye[1] + t*ray[1], seye[2] + t*ray[2]);

    vec3 S3 = checkingPoint - a;
    vec3 S4 = b - checkingPoint;
    vec3 S5 = c - checkingPoint;

    vec3 v31 = S1 ^ S3; float tri31 = v31.length() / 2;
    vec3 v32 = S2 ^ S3; float tri32 = v32.length() / 2;
    vec3 v12 = S1 ^ S2; float tri12 = v12.length() / 2;
    vec3 v45 = S4 ^ S5; float tri45 = v45.length() / 2;

    //if ( tri31 + tri32 + tri45 - tri12 <= inaccuracy ) { return true;}

    //if ( (tri31 + tri32) > tri12 ) { return false; }
    if ( (tri31 + tri32 + tri45 - tri12) > inaccuracy ) { return false; }

    return true;
}

int main()
{
    readFile();
    ColorImage image;
    image.init(Rw, Rh);
    Pixel white = {255, 255, 255};
    for(int x = 0; x < Rw; x += 1) {
        for(int y = 0; y < Rh; y += 1) {
            image.writePixel(x, y, white);
        }
    }
    char outputname[15] = "output.ppm";
    image.outputPPM(outputname);
    return 0;
}
