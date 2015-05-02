#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

#include "algebra3.h"
#include "imageIO.h"
#include "RayTracer.h"

#define PI 3.1415926535898
#define Tan(th) tan(PI/180*(th))
#define Cos(th) cos(PI/180*(th))

using namespace std;

vec3 eye, direct, vertical = vec3(0, 1, 0), light;
float angle;
int resolusionW, resolusionH;

float SOx, SOy, SOz, Sr;
float Ax, Ay, Az, Bx, By, Bz, Cx, Cy, Cz;

vector<vec4> Sphere;
vector<vec3> Triangle;

float Ia, Id, Is, Ii, Ke = 1000.0, Nr = 2.0;
vec3 Ka = vec3(0.100, 0.100, 0.100);
vec3 Kd = vec3(0.700, 0.600, 0.300);
vec3 Ks = vec3(1.000, 1.000, 1.000);

void readFile() {
    ifstream ifile("input.txt");
    string test;
    while( ifile >> test ) {
        switch (test[0]) {
            case 'E':
                ifile >> eye[0] >> eye[1] >> eye[2];
                break;
            case 'V':
                ifile >> direct[0] >> direct[1] >> direct[2];
                break;
            case 'F':
                ifile >> angle;
                break;
            case 'R':
                ifile >> resolusionW >> resolusionH;
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
                break;
        }
    }
}

int main()
{
    readFile();
    RayTracer raytracer(eye, direct, vertical, angle, resolusionW, resolusionH);
    ColorImage image;
    image.init(resolusionW, resolusionH);
    for(int x = 0; x < resolusionW; x += 1) {
        for(int y = 0; y < resolusionH; y += 1) {

            Pixel color;
            image.writePixel(x, y, color);
        }
    }
    char outputname[11] = "output.ppm";
    image.outputPPM(outputname);
    return 0;
}
