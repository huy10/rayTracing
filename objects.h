#include "stdafx.h"
#include "math.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

class _point
{
public:
	float x,y,z;
	 _point(float xx, float yy, float zz){x = xx; y= yy; z = zz;}
	 _point(){x=0;y=0;z=0;}
	 bool operator == (const _point& p1) const{return ((int)x==(int)p1.x && (int)y==(int)p1.y && (int)z==(int)p1.z)?1:0;}
	 _point operator +(_point &v1)const{return _point(x+v1.x,	y+v1.y,		z+v1.z);}
	 _point operator -(_point &v1)const{return _point(x-v1.x,	y-v1.y,		z-v1.z);}
	 _point operator *(float C)const{return _point(C*x,C*y,C*z);}
	 _point &operator = (const _point & c1){x = c1.x; y = c1.y; z = c1.z; return *this;}
};

class _vector
{
public:
	_vector(){}
	~_vector(){}
	float x,y,z;
	_vector(float xx, float yy,float zz){x = xx; y= yy; z = zz;}
	void normalize(){ float temp =sqrt(x*x+y*y+z*z); x /= temp; y /= temp; z /=temp;}
	_vector operator +(_vector &v1)const{return _vector(x+v1.x,y+v1.y,z+v1.z);}
	_vector operator -(_vector &v1)const{return _vector(x-v1.x,y-v1.y,z-v1.z);}
	_vector operator *(float C)const{return _vector(C*x,C*y,C*z);}
	_vector &operator =(const _vector &v1){x = v1.x;y=v1.y;z=v1.z; return *this;}
	 bool operator == (const _vector& p1) const{return (x==p1.x && y==p1.y && z==p1.z)?1:0;}

};

class colorScalar{
public:
	float b,g,r;
	colorScalar(float bb, float gg, float rr){r = rr; g = gg; b = bb;}
	colorScalar & operator = (const colorScalar& c1){b = c1.b; g = c1.g; r = c1.r; return *this;}
	colorScalar(){b = 0; g = 0; r = 0;}
	colorScalar operator + (const colorScalar& c1) const{return colorScalar(c1.b+b,c1.g+g,c1.r+r);}
	colorScalar operator /(float C)const{return colorScalar(b/C,g/C,r/C);}
	colorScalar operator *(float C)const{return colorScalar((b*C>255)?255:b*C,(g*C>255)?255:g*C,(r*C>255)?255:r*C);}
	 bool operator == (const colorScalar& p1) const{return (b==p1.b && g==p1.g && r==p1.r)?1:0;}

};

class sphere
{
public:
	_point center;
	float radius;
	sphere(float x, float y, float z, float rad,float bb, float gg, float rr) {center = _point(x,y,z); radius = rad;lightIntense = colorScalar(bb,gg,rr);}
	sphere(float x, float y, float z, float rad, colorScalar intense) {center = _point(x,y,z); radius = rad;lightIntense = intense;}
	sphere(float x, float y, float z, float rad, colorScalar intense, float ka, float ks, float kds, float kt, float kdt, float kss, float ktt)
	{
		center = _point(x,y,z); radius = rad;lightIntense = intense;
		Ka = ka;
		Ks = ks;
		Kds = kds;
		Kt = kt;
		Kdt = kdt;
		Kss = kss;
		Ktt = ktt;
	}

	colorScalar lightIntense;
	float Ka, Kds, Ks,Kdt,Kt;          // Hall model parameters
	float Kss, Ktt;         // ray tracing parameters
	
};
