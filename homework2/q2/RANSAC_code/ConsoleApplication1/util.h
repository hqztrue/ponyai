#ifndef _UTIL_H_
#define _UTIL_H_
#include "common.h"

const double eps = 1e-6;
struct point3D{
	double x, y, z;
	point3D(double _x = 0, double _y = 0, double _z = 0) :x(_x), y(_y), z(_z){}
	friend point3D operator +(const point3D &x, const point3D &y){ return point3D(x.x + y.x, x.y + y.y, x.z + y.z); }
	friend point3D operator -(const point3D &x, const point3D &y){ return point3D(x.x - y.x, x.y - y.y, x.z - y.z); }
	double operator *(const point3D &p)const{ return x*p.x + y*p.y + z*p.z; }
	friend point3D operator ^(const point3D &x, const point3D &y){ return point3D(x.y*y.z - y.y*x.z, x.z*y.x - y.z*x.x, x.x*y.y - y.x*x.y); }
	point3D operator *(double d)const{ return point3D(x*d, y*d, z*d); }
	point3D operator /(double d)const{ return point3D(x / d, y / d, z / d); }
	point3D operator -()const{ return point3D(-x, -y, -z); }
	bool operator <(const point3D &p)const{ return x + eps<p.x || fabs(x - p.x)<eps&&y<p.y || fabs(x - p.x)<eps&&fabs(y - p.y)<eps&&z<p.z; }
	bool operator ==(const point3D &p)const{ return fabs(x - p.x)<eps&&fabs(y - p.y)<eps&&fabs(z - p.z)<eps; }
	bool operator !=(const point3D &p)const{ return !(*this == p); }
	bool operator >(const point3D &p)const{ return !(*this == p || *this<p); }
	point3D& operator +=(const point3D &p){ x += p.x; y += p.y; z += p.z; return *this; }
	point3D& operator -=(const point3D &p){ x -= p.x; y -= p.y; z -= p.z; return *this; }
	point3D& operator *=(double d){ x *= d; y *= d; z *= d; return *this; }
	point3D& operator /=(double d){ x /= d; y /= d; z /= d; return *this; }
	inline friend double dot(const point3D &x, const point3D &y){ return x*y; }
	inline friend point3D cross(const point3D &x, const point3D &y){ return x^y; }
	double len()const{ return sqrt(x*x + y*y + z*z); }
	double dist(const point3D &p)const{ return (*this - p).len(); }
	inline friend double dist(const point3D &x, const point3D &y){ return (x - y).len(); }
	point3D unit()const{ return *this / len(); }
	double abs2()const{ return x*x + y*y + z*z; }
	double dist2(const point3D &p){ return (*this - p).abs2(); }
	inline friend double angle(const point3D &x, const point3D &y){ return acos(x*y / x.len() / y.len()); }
	friend istream& operator >>(istream &in, point3D &p){ return in >> p.x >> p.y >> p.z; }
	friend ostream& operator <<(ostream &out, const point3D &p){ return out << "(" << p.x << "," << p.y << "," << p.z << ")"; }
	inline friend double volume(const point3D &a, const point3D &b, const point3D &c, const point3D &d){ return ((b - a) ^ (c - a))*(d - a); }
	inline friend double area(const point3D &a, const point3D &b, const point3D &c){ return ((b - a) ^ (c - a)).len(); }
};


#endif



