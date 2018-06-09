#pragma once

#include<algorithm>
#include<assert.h>
#include<bitset>
#include<complex>
#include<ctype.h>
#include<deque>
#include<fstream>
#include<functional>
#include<iomanip>
#include<iostream>
#include<iterator>
#include<limits>
#include<list>
#include<map>
#include<math.h>
#include<numeric>
#include<queue>
#include<set>
#include<sstream>
#include<stack>
#include<stdarg.h>
#include<stddef.h>
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<string>
#include<time.h>
#include<typeinfo>
#include<utility>
#include<vector>
//#include<memory.h>
//#include<rope.h>
//#include<ext/rope>
//#include <hash_map>
//#include <hash_set>
//#include<windows.h>
using namespace std;
//using namespace __gnu_cxx;

namespace geometry{
#define link Link
#define rep(i,n) for (int i=1;i<=n;++i)
#define For(i,l,r) for(int i=l;i<=r;++i)
#define foreach(i,x) for(typeof((x).begin()) i=(x).begin();i!=(x).end();++i)
#define DO(n) while(n--)
#define DO_C(n) int n_=n;while(n_--)
#define INS(A,P,B) A.insert(A.begin()+P,B)
#define ERS(A,P) A.erase(A.begin()+P)
#define CTN(A,x) (A.find(x)!=A.end())
#define SORT(x) sort(all(x))
#define CLEAR(x) memset(x,0,sizeof(x))
#define FILL(x,c) memset(x,c,sizeof(x))
#define CPY(A,B) memcpy(A,B,sizeof(B))
#define all(x) x.begin(),x.end()
#define SZ(x) ((int)(x.size()))
#define DREP(x) sort(all(x));x.erase(unique(x.begin(),x.end()),x.end())
#define INDEX(arr,index) (lower_bound(arr.begin(),arr.end(),index)-arr.begin())
#define MP(A,B) make_pair(A,B)
#define Debug(x) cout<<#x<<"="<<x<<endl
#define debug(args...) {dbg,args;cerr<<endl;}
#define Dbg(x) cout<<"debug: "<<__FUNCTION__<<"() @ "<<__TIMESTAMP__<<"\n"\
	<<__FILE__<<" L"<<__LINE__<<"\n"<<#x" = "<<(x)<<endl
#define dline cerr<<endl
#define mp make_pair
#define pb push_back
#define RT return
#define FF first
#define SS second
#define tri(a,b,c) make_pair((a),make_pair((b),(c)))
#define XX first
#define YY second.first
#define ZZ second.second
#define Scan(x) scanf("%d",&x)
#ifdef _WIN32
	#define Scanl(x) scanf("%I64d",x)
#else
	#define Scanl(x) scanf("%lld",x)
#endif
#define Scanf(x) scanf("%lf",&x)
#define Scans(x) scanf("%s",x)
#define eprintf(...) {fprintf(stderr,__VA_ARGS__),fflush(stderr);}
#define maX(a,b) ((a)>(b)?(a):(b))
#define miN(a,b) ((a)<(b)?(a):(b))
#define abS(x) ((x)<0?-(x):(x))
#define Display(A,n,m){ \
	rep(i,n){ \
        rep(j,m)cout<<A[i][j]<<" "; \
		cout<<endl; \
	} \
}
#define likely(x) __builtin_expect(!!(x),1)
#define unlikely(x) __builtin_expect(!!(x),0)
//#define if(_) if(!(_))
//#pragma comment(linker,"/STACK:36777216")
//#pragma GCC optimize ("O2")
#define Ruby system("ruby main.rb")
#define Haskell system("runghc main.hs")
#define Python system("py main.py")
#define Pascal system("fpc main.pas")
const int inf=~0U>>1;
const int MOD=1000000007;
const long long linf=(long long)1e18;
const double eps=1e-5,PI=atan(1.0)*4,E=2.71828182845904523;
typedef long long ll;
typedef long long LL;
typedef unsigned long long ull;
typedef unsigned int uint;
typedef double DB;
typedef long double ld;
typedef pair<int,int> PII;
typedef pair<int,bool> PIB;
typedef pair<int,pair<int,int> > tri;
typedef vector<int> VI;
typedef vector<char> VC;
typedef vector<string> VS;
typedef vector<ll> VL;
typedef vector<double> VD;
typedef vector<PII> VII;
typedef vector<VI> VVI;
typedef vector<VII> VVII;
typedef vector<int>::iterator vit;
typedef set<int> SI;
typedef set<string> SS;
typedef set<ll> SL;
typedef set<double> SD;
typedef set<PII> SII;
typedef set<int>::iterator sit;
typedef map<int,int> MII;
typedef map<string,int> MSI;
typedef map<ll,int> MLI;
typedef map<double,int> MDI;
typedef map<int,bool> MIB;
typedef map<string,bool> MSB;
typedef map<ll,bool> MLB;
typedef map<double,bool> MDB;
typedef map<int,int>::iterator mit;
typedef map<PII,int> MPIII;
typedef map<PII,bool> MPIIB;
typedef istringstream ISS;
typedef ostringstream OSS;

template<class T> inline T sqr(const T &a){return a*a;}
inline int sgn(double x){return fabs(x)<eps?0:(x<0?-1:1);}
inline int sign(double x){return fabs(x)<eps?0:(x<0?-1:1);}
inline double cross(double x1,double y1,double x2,double y2){return x1*y2-x2*y1;}
inline double dot(double x1,double y1,double x2,double y2){return x1*x2+y1*y2;}
inline double cha(double x1,double y1,double x2,double y2){return x1*y2-x2*y1;}
struct point{
	double x,y;
	point(double _x=0,double _y=0):x(_x),y(_y){}
	point operator +(const point &p)const{return point(x+p.x,y+p.y);}
	point operator -(const point &p)const{return point(x-p.x,y-p.y);}
	double operator *(const point &p)const{return x*p.x+y*p.y;}  //dot
	double operator ^(const point &p)const{return x*p.y-p.x*y;}  //cha
	point operator /(const point &p)const{return point((x*p.x+y*p.y)/p.len(),(y*p.x-x*p.y)/p.len());}
	point operator *(double d)const{return point(x*d,y*d);}
	friend point operator *(double d,const point &x){return point(x.x*d,x.y*d);}
	point operator /(double d)const{return point(x/d,y/d);}
	friend point operator /(double d,const point &x){return point(x.x/d,x.y/d);}
	point operator -()const{return point(-x,-y);}
	bool operator <(const point &p)const{return x+eps<p.x||fabs(x-p.x)<eps&&y<p.y;}
	bool operator ==(const point &p)const{return fabs(x-p.x)<eps&&fabs(y-p.y)<eps;}
	bool operator !=(const point &p)const{return !(*this==p);}
	bool operator >(const point &p)const{return !(*this==p||*this<p);}
	point& operator +=(const point &p){x+=p.x;y+=p.y;return *this;}
	point& operator -=(const point &p){x-=p.x;y-=p.y;return *this;}
	point& operator *=(double d){x*=d;y*=d;return *this;}
	point& operator /=(double d){x/=d;y/=d;return *this;}
	inline friend double dot(const point &x,const point &y){return x*y;}
	inline friend double cross(const point &x,const point &y){return x^y;}
	double len()const{return sqrt(x*x+y*y);}
	double dist(const point &p)const{return (*this-p).len();}
	inline friend double dist(const point &x,const point &y){return (x-y).len();}
	point unit()const{return *this/len();}
	double abs2()const{return x*x+y*y;}
	double dist2(const point &p){return (*this-p).abs2();}
	double atan()const{return atan2(y,x);}
	point normal_vector(const point &x){double l=x.len();return point(-x.y/l,x.x/l);}
	point rotate(double angle)const{
		return point(x*cos(angle)-y*sin(angle),y*cos(angle)+x*sin(angle));
	}
	inline friend point rotate(const point &x,double angle){
		return point(x.x*cos(angle)-x.y*sin(angle),x.y*cos(angle)+x.x*sin(angle));
	}
	inline friend double angle(const point &x,const point &y){return acos(x*y/x.len()/y.len());}
	inline friend point exp(const point &x){
		return point(exp(x.x)*cos(x.y),exp(x.x)*sin(x.y));
	}
	friend istream& operator >>(istream &in,point &p){return in>>p.x>>p.y;}
	friend ostream& operator <<(ostream &out,const point &p){return out<<"("<<p.x<<","<<p.y<<")";}
	void print()const{printf("(%.5lf,%.5lf)\n",x,y);}
};
struct line{
	point x,y;
	line(){}
	line(const point &_x,const point &_y):x(_x),y(_y){}
	point at(double rate)const{return x+(y-x)*rate;}
	point atx(double _x)const{return x+(y-x)*((_x-x.x)/(y.x-x.x));}
	double slope()const{return (y.y-x.y)/(y.x-x.x);}
	double tan()const{return (y-x).atan();}
	bool opposite(const point &a,const point &b)const{
		return sgn((y-x)^(a-x))*sgn((y-x)^(b-x))==-1;
	}
	inline friend int on_line(const point &p,const line &l){return sgn(((p-l.x)^(l.y-l.x))/(p-l.x).len()/(l.y-l.x).len())==0;}  //return sgn((p-l.x)^(l.y-l.x))==0;
	inline friend int on_segment(const point &p,const line &l){
		if (on_line(p,l)&&min(l.x.x,l.y.x)-eps<=p.x&&max(l.x.x,l.y.x)+eps>=p.x&&
		min(l.x.y,l.y.y)-eps<=p.y&&max(l.x.y,l.y.y)+eps>=p.y)return 1;
		return 0;
	}
	inline friend point cross(const line &a,const line &b){
		double s1=cha(b.y.x-b.x.x,b.y.y-b.x.y,a.x.x-b.x.x,a.x.y-b.x.y),
		s2=cha(b.y.x-b.x.x,b.y.y-b.x.y,a.y.x-b.x.x,a.y.y-b.x.y);
		if (fabs(s1-s2)<eps)return point(1e100,1e100);
		return point((a.x.x*s2-a.y.x*s1)/(s2-s1),(a.x.y*s2-a.y.y*s1)/(s2-s1));
	}
	inline friend bool in_line(const point &p,const line &l){  //in left plane
		return cha(l.y.x-l.x.x,l.y.y-l.x.y,p.x-l.x.x,p.y-l.x.y)>-eps;
	}
};
struct line_v{
	point x,v;
	line_v(){}
	line_v(const point &_x,const point &_v):x(_x),v(_v.unit()){}
	point at(double rate)const{return x+v*rate;}
	double tan()const{return v.atan();}
	bool opposite(const point &a,const point &b)const{return sign(v^(a-x))*sign(v^(b-x))==-1;}
};
struct point3D{
	double x,y,z;
	point3D(double _x=0,double _y=0,double _z=0):x(_x),y(_y),z(_z){}
	friend point3D operator +(const point3D &x,const point3D &y){return point3D(x.x+y.x,x.y+y.y,x.z+y.z);}
	friend point3D operator -(const point3D &x,const point3D &y){return point3D(x.x-y.x,x.y-y.y,x.z-y.z);}
	friend point3D mul(const point3D &x,const point3D &y){return point3D(x.x*y.x,x.y*y.y,x.z*y.z);}
	double operator *(const point3D &p)const{return x*p.x+y*p.y+z*p.z;}
	friend point3D operator ^(const point3D &x,const point3D &y){return point3D(x.y*y.z-y.y*x.z,x.z*y.x-y.z*x.x,x.x*y.y-y.x*x.y);}
	point3D operator *(double d)const{return point3D(x*d,y*d,z*d);}
	point3D operator /(double d)const{return point3D(x/d,y/d,z/d);}
	point3D operator -()const{return point3D(-x,-y,-z);}
	bool operator <(const point3D &p)const{return x+eps<p.x||fabs(x-p.x)<eps&&y<p.y||fabs(x-p.x)<eps&&fabs(y-p.y)<eps&&z<p.z;}
	bool operator ==(const point3D &p)const{return fabs(x-p.x)<eps&&fabs(y-p.y)<eps&&fabs(z-p.z)<eps;}
	bool operator !=(const point3D &p)const{return !(*this==p);}
	bool operator >(const point3D &p)const{return !(*this==p||*this<p);}
	point3D& operator +=(const point3D &p){x+=p.x;y+=p.y;z+=p.z;return *this;}
	point3D& operator -=(const point3D &p){x-=p.x;y-=p.y;z-=p.z;return *this;}
	point3D& operator *=(double d){x*=d;y*=d;z*=d;return *this;}
	point3D& operator /=(double d){x/=d;y/=d;z/=d;return *this;}
	inline friend point3D operator *(double d,const point3D &p){return p*d;}
	inline friend double dot(const point3D &x,const point3D &y){return x*y;}
	inline friend point3D cross(const point3D &x,const point3D &y){return x^y;}
	inline point3D perp(){return ((fabs(x)>.1?point3D(0,1,0):point3D(1,0,0))^(*this)).unit();}  //perpendicular
	friend point3D min(const point3D &x,const point3D &y){return point3D(min(x.x,y.x),min(x.y,y.y),min(x.z,y.z));}
	friend point3D max(const point3D &x,const point3D &y){return point3D(max(x.x,y.x),max(x.y,y.y),max(x.z,y.z));}
	double len()const{return sqrt(x*x+y*y+z*z);}
	double dist(const point3D &p)const{return (*this-p).len();}
	inline friend double dist(const point3D &x,const point3D &y){return (x-y).len();}
	point3D unit()const{return *this/len();}
	void normalize(){double l=len();x/=l;y/=l;z/=l;}
	double abs2()const{return x*x+y*y+z*z;}
	double dist2(const point3D &p){return (*this-p).abs2();}
	inline friend double angle(const point3D &x,const point3D &y){return acos(x*y/x.len()/y.len());}
	friend istream& operator >>(istream &in,point3D &p){return in>>p.x>>p.y>>p.z;}
	friend ostream& operator <<(ostream &out,const point3D &p){return out<<"("<<p.x<<","<<p.y<<","<<p.z<<")";}
	void print()const{printf("(%.5lf,%.5lf,%.5lf)\n",x,y,z);}
	inline friend double volume(const point3D &a,const point3D &b,const point3D &c,const point3D &d){return ((b-a)^(c-a))*(d-a);}
	inline friend double area(const point3D &a,const point3D &b,const point3D &c){return ((b-a)^(c-a)).len();}
	inline friend double det(const point3D &a,const point3D &b,const point3D &c){
		return a.x*(b.y*c.z-b.z*c.y)-a.y*(b.x*c.z-b.z*c.x)+a.z*(b.x*c.y-b.y*c.x);
	}
};
struct line3D{
	point3D x,y;
	line3D(){}
	line3D(const point3D &_x,const point3D &_y):x(_x),y(_y){}
	inline point3D at(double rate)const{return x+(y-x)*rate;}
};
struct ray3D{
	point3D o,d;
	ray3D(){}
	ray3D(const point3D &_o,const point3D &_d):o(_o),d(_d.unit()){}
	inline void normalize(){d.normalize();}
	inline point3D at(double rate)const{return o+d*rate;}
};
struct object3D{
	virtual double intersect(const ray3D &l)const=0;
	virtual point3D norm(const point3D &p)const=0;
	virtual pair<point3D,point3D> bbox()const=0;
};
struct plane3D:object3D{
	point3D p,v;
	plane3D(){}
	plane3D(const point3D &_p,const point3D &_v):p(_p),v(_v.unit()){}
	inline friend double DistToPlane(const point3D &x,const plane3D &p){return fabs((x-p.p)*p.v);}
	inline friend point3D GetPlaneProjection(const point3D &x,const plane3D &p){return x-p.v*dot(x-p.p,p.v);}
	inline friend point3D LinePlaneIntersection(const line3D &l,const plane3D &p){
		double t=dot(p.v,p.p-l.x)/dot(p.v,l.y-l.x);return l.at(t);
	}
	virtual double intersect(const ray3D &l)const{
		double d=dot(v,l.d);
		if (fabs(d)<eps)return -1;
		double t=dot(v,p-l.o)/d;
		if (t>eps)return t;
		else return -1;
	}
	point3D norm(const point3D &p)const{return v;}
	pair<point3D,point3D> bbox()const{
		return make_pair(point3D(-inf,-inf,-inf),point3D(inf,inf,inf));
	}
};
struct sphere:object3D{
	point3D o;
	double r;
	sphere(const point3D &o_=point3D(0,0,0),double r_=0):o(o_),r(r_){}
	virtual double intersect(const ray3D &l)const{
		point3D v=o-l.o;
		double v2=v*v,r2=r*r,tp=v*l.d;
		if (v2*(1+eps)>=r2&&tp<=0)return -1;
		double d2=v2-tp*tp,t;
		if (d2>r2)return -1;
		if (v2<=r2*(1+eps))t=tp+sqrt(r2-d2);
		else t=tp-sqrt(r2-d2);
		if (t<=eps)t=tp+sqrt(r2-d2);
		if (t>eps)return t;
		else return -1;
	}
	point3D norm(const point3D &p)const{
		return (p-o).unit();
	}
	pair<point3D,point3D> bbox()const{
		return make_pair(point3D(o.x-r,o.y-r,o.z-r),point3D(o.x+r,o.y+r,o.z+r));
	}
};
struct triangle3D:object3D{
	point3D p[3];
	triangle3D(){}
	triangle3D(const point3D &a,const point3D &b,const point3D &c){p[0]=a;p[1]=b;p[2]=c;}
	point3D norm()const{return ((p[1]-p[0])^(p[2]-p[1])).unit();}
	virtual double intersect(const ray3D &l)const{
		double d=det(l.d,p[0]-p[1],p[0]-p[2]),t=det(p[0]-l.o,p[0]-p[1],p[0]-p[2])/d,
			beta=det(l.d,p[0]-l.o,p[0]-p[2])/d,gamma=det(l.d,p[0]-p[1],p[0]-l.o)/d;
		if ((t>eps)and(beta>=0)and(gamma>=0)and(beta+gamma<=1))return t;
		else return -1;
	}
	point3D norm(const point3D &p)const{return norm();}
	pair<point3D,point3D> bbox()const{
		point3D p0=p[0],p1=p[0];
		for (int i=1;i<3;++i)p0=min(p0,p[i]),p1=max(p1,p[i]);
		return make_pair(p0,p1);
	}
};
inline double angle(const point3D &o,const point3D &p1,const point3D &p2){
	point3D v1=p1-o,v2=p2-o;
	double len=v1.len()*v2.len();
	if (fabs(len)<eps)return 0;
	double ang=v1*v2/len;
	if (ang>=1.0)return 0; if (ang<=-1.0)return -PI;
	ang=acos(ang);
	if ((v1^v2).len()>0)return ang;
	else return -ang;
}
struct polygon3D:object3D{
	vector<point3D> p;
	typedef vector<point3D>::iterator vit;
	point3D norm()const{return ((p[1]-p[0])^(p[2]-p[1])).unit();}
	inline void clear(){p.clear();}
	inline void add(const point3D &pt){p.push_back(pt);}
	bool inside(const point3D &pt)const{
		int n=p.size();
		double ang=0;
		for (int i=0,j;i<n;++i){
			if (i==n-1)j=0;
			else j=i+1;
			ang+=angle(pt,p[i],p[j]);
		}
		return fabs(ang-2*PI)<1e-1;
	}
	virtual double intersect(const ray3D &l)const{
		point3D v=norm();
		double t=dot(v,p[0]-l.o)/dot(v,l.d);
		if (t<eps)return -1;
		point3D pt=l.at(t);
		if (inside(pt))return t;
		else return -1;
	}
	point3D norm(const point3D &p)const{return norm();}
	pair<point3D,point3D> bbox()const{
		point3D p0=p[0],p1=p[0];
		for (int i=1;i<p.size();++i)p0=min(p0,p[i]),p1=max(p1,p[i]);
		return make_pair(p0,p1);
	}
};
struct box3D:object3D{
	point3D p0,p1;
	box3D(){}
	box3D(const point3D &p0_,const point3D &p1_):p0(p0_),p1(p1_){}
	virtual double intersect(const ray3D &l)const{
		double tmin=-inf,tmax=inf;
		if (fabs(l.d.x)>eps){
			double t0=(p0.x-l.o.x)/l.d.x,t1=(p1.x-l.o.x)/l.d.x;
			if (t0>t1)swap(t0,t1);
			tmin=max(tmin,t0);
			tmax=min(tmax,t1);
		}
		if (fabs(l.d.y)>eps){
			double t0=(p0.y-l.o.y)/l.d.y,t1=(p1.y-l.o.y)/l.d.y;
			if (t0>t1)swap(t0,t1);
			tmin=max(tmin,t0);
			tmax=min(tmax,t1);
		}
		if (fabs(l.d.z)>eps){
			double t0=(p0.z-l.o.z)/l.d.z,t1=(p1.z-l.o.z)/l.d.z;
			if (t0>t1)swap(t0,t1);
			tmin=max(tmin,t0);
			tmax=min(tmax,t1);
		}
		if (tmin<tmax){
			if (tmin>eps)return tmin;
			if (tmax>eps)return tmax;
		}
		return -1;
	}
	point3D norm(const point3D &p)const{
		return point3D();  //todo
	}
	pair<point3D,point3D> bbox()const{
		return make_pair(p0,p1);
	}
};
struct circle{
	double x,y,r;
	circle(double _x=0,double _y=0,double _r=0):x(_x),y(_y),r(_r){}
	point o()const{return point(x,y);}
	point get_point(double ang)const{return point(x+cos(ang)*r,y+sin(ang)*r);}
	double get_ang(const point &p)const{return (p-point(x,y)).atan();}
	inline friend vector<point> GetLineCircleIntersection(const line &l,const circle &C){
		vector<point> res;double a=l.y.x-l.x.x,b=l.x.x-C.x,c=l.y.y-l.x.y,d=l.x.y-C.y,
			e=a*a+c*c,f=2*(a*b+c*d),g=b*b+d*d-C.r*C.r,delta=f*f-4*e*g,t;
		if (sign(delta)<0)return res;
		if (sign(delta)==0){t=-f/(2*e);res.push_back(l.x+(l.y-l.x)*t);return res;}
		t=(-f-sqrt(delta))/(2*e);res.push_back(l.x+(l.y-l.x)*t);
		t=(-f+sqrt(delta))/(2*e);res.push_back(l.x+(l.y-l.x)*t);
		return res;
	}
	inline friend vector<point> cut_circle(const circle &C,const point &x){
		vector<point> res;point u=C.o()-x;double d1=u.len();if (sign(C.r-d1)<0)return res;
		if (sign(C.r-d1)==0){res.push_back(x);return res;}
		double d2=sqrt(d1*d1-C.r*C.r),angle=asin(C.r/d1);
		res.push_back(x+rotate(u,-angle)*(d2/d1));res.push_back(x+rotate(u,angle)*(d2/d1));
		return res;
	}
	inline friend vector<point> cross_circle(const circle &x,const circle &y){
		vector<point> res;double d=point(x.x-y.x,x.y-y.y).len();if (sign(d-x.r-y.r)<0)return res;
		double a=point(y.x-x.x,y.y-x.y).atan(),da=acos((x.r*x.r+d*d-y.r*y.r)/(2*x.r*d));
		point x1=x.get_point(a-da),x2=x.get_point(a+da);
		if (sign(d-x.r-y.r)==0){res.push_back(x1);return res;}
		res.push_back(x1);res.push_back(x2);return res;
	}
	inline friend pair<int,vector<line> > GetTangents(circle A,circle B){
		vector<line> res;if (A.r<B.r)swap(A,B);
		double d2=(A.x-B.x)*(A.x-B.x)+(A.y-B.y)*(A.y-B.y),rdiff=A.r-B.r,rsum=A.r+B.r;
		if (d2==0&&A.r==B.r)return make_pair(-1,res);
		if (d2<rdiff*rdiff)return make_pair(0,res);
		double base=atan2(B.y-A.y,B.x-A.x),ang=acos(rdiff/sqrt(d2));
		if (d2==rdiff*rdiff){res.push_back(line(A.get_point(base),B.get_point(base)));return make_pair(1,res);}
		res.push_back(line(A.get_point(base+ang),B.get_point(base+ang)));
		res.push_back(line(A.get_point(base-ang),B.get_point(base-ang)));
		if (sign(d2-rsum*rsum)==0)res.push_back(line(A.get_point(base),B.get_point(base+PI)));
		else if (sign(d2-rsum*rsum)>0){
			ang=acos(rsum/sqrt(d2));
			res.push_back(line(A.get_point(base+ang),B.get_point(PI+base+ang)));
			res.push_back(line(A.get_point(base-ang),B.get_point(PI+base-ang)));
		}
		return make_pair(res.size(),res);
	}
	inline friend bool in_circle(const point &x,const circle &O){return (O.x-x.x)*(O.x-x.x)+(O.y-x.y)*(O.y-x.y)<=O.r*O.r*(1+eps);}
	inline friend bool in_circle(const circle &x,const circle &O){return (O.x-x.x)*(O.x-x.x)+(O.y-x.y)*(O.y-x.y)<=(O.r+x.r)*(O.r+x.r)*(1+eps);}
	inline friend circle get_circle(const point &a,const point &b,const point &c){
		circle res;line l1=line((a+b)/2,(a+b)/2+(b-a).rotate(PI/2)),l2=line((a+c)/2,(a+c)/2+(c-a).rotate(PI/2));
		point o=cross(l1,l2);res.x=o.x;res.y=o.y;res.r=a.dist(o);return res;
	}
	void print(){printf("(%.5lf,%.5lf),%.5lf ",x,y,r);}
};
struct plane{  //Ax+By+C>=0
	double A,B,C,angle;
	plane(double _A=0,double _B=0,double _C=0,double _angle=0):A(_A),B(_B),C(_C),angle(_angle){}
	void rev(){A=-A;B=-B;C=-C;}
	inline friend bool operator <(const plane &x,const plane &y){return x.angle+eps<y.angle;}
	inline friend point3D cross(const plane &x,const plane &y){
		point3D p;p.x=x.B*y.C-x.C*y.B; p.y=x.C*y.A-x.A*y.C; p.z=x.A*y.B-x.B*y.A;
		if (fabs(p.z)>eps)p.x/=p.z,p.y/=p.z,p.z=1;return p;
	}
	inline friend bool stayout(const point3D &p,const plane &l){
		return l.A*p.x+l.B*p.y+l.C*p.z<-eps?1:0;
	}
	double dist(const point &x)const{return fabs(A*x.x+B*x.y+C)/sqrt(A*A+B*B);}
};
inline plane make_line(double x1,double y1,double x2,double y2){  //left plane
	plane l;double z1=1,z2=1;
	l.A=y1*z2-y2*z1;l.B=z1*x2-z2*x1;l.C=x1*y2-x2*y1;l.angle=atan2(y2-y1,x2-x1);
	if (stayout(point3D(x1+y1-y2,y1+x2-x1,1),l))l.rev();return l;
}
inline plane make_line(const point &x,const point &y){return make_line(x.x,x.y,y.x,y.y);}
struct triangle{
	point a,b,c;
	triangle(){}
	triangle(const point &_a,const point &_b,const point &_c):a(_a),b(_b),c(_c){}
};
inline point GetLineIntersection(const point &p,const point &v,const point &q,const point &w){
	double t=(w^(p-q))/(v^w);return p+v*t;
}
inline double DistanceToLine(const point &p,const point &a,const point &b){
	point v1=b-a,v2=p-a;return fabs((v1^v2)/v1.len());
}
inline double DistanceToLine(const point3D &p,const point3D &a,const point3D &b){
	point3D v1=b-a,v2=p-a;return ((v1^v2)/v1.len()).len();
}
inline point GetLineProjection(const point &p,const point &a,const point &b){
	point v=b-a;return a+v*((v*(p-a))/(v*v));
}
inline point SymmetryPoint(const point &p,const line &l){
	point p1=GetLineProjection(p,l.x,l.y);return p1*2-p;
}
inline double DistanceToSegment(const point &p,const point &a,const point &b){
	if (a==b)return dist(p,a);point v1=b-a,v2=p-a,v3=p-b;
	if (sign(v1*v2)<0)return v2.len();if (sign(v1*v3)>0)return v3.len();
	return fabs((v1^v2)/v1.len());
}
inline double DistanceToSegment(const point3D &p,const point3D &a,const point3D &b){
	if (a==b)return dist(p,a);point3D v1=b-a,v2=p-a,v3=p-b;
	if (sign(v1*v2)<0)return v2.len();if (sign(v1*v3)>0)return v3.len();
	return (v1^v2).len()/v1.len();
}
inline double to_rad(double deg){return deg/180*PI;}
inline point3D get_coord(double lng,double lat,double r){
	lng=to_rad(lng);lat=to_rad(lat);return point3D(cos(lat)*cos(lng),cos(lat)*sin(lng),sin(lat))*r;
}
inline point3D LinePlaneIntersection(const line3D &l,double A,double B,double C,double D){
	double t=(A*l.x.x+B*l.x.y+C*l.x.z)/(A*(l.x.x-l.y.x)+B*(l.x.y-l.y.y)+C*(l.x.z-l.y.z));return l.at(t);
}
inline double area(double a,double b,double c){double p=(a+b+c)/2;return sqrt(p*(p-a)*(p-b)*(p-c));}
inline double angle(const point &o,const point &p1,const point &p2){
	double sx=p1.x-o.x,sy=p1.y-o.y,ex=p2.x-o.x,ey=p2.y-o.y;
	double fi,cosfi=sx*ex+sy*ey,norm=(sx*sx+ey*ey)*(ex*ex+ey*ey); cosfi/=sqrt(norm);
	if (cosfi>=1.0)return 0; if (cosfi<=-1.0)return -PI; fi=acos(cosfi);
	if (cha(sx,sy,ex,ey)>0)return fi; else return -fi;
}
inline double area2(const point &a,const point &b,const point &c){return (b-a)^(c-a);}
inline double area2(const point3D &a,const point3D &b,const point3D &c){return ((b-a)^(c-a)).len();}
inline bool PointInTri(const point3D &p,const point3D &p0,const point3D &p1,const point3D &p2){
	double a1=area2(p,p0,p1),a2=area2(p,p1,p2),a3=area2(p,p2,p0);
	return sign(a1+a2+a3-area2(p0,p1,p2))==0;
}
inline pair<bool,point3D> TriSegIntersection(const point3D &p0,const point3D &p1,const point3D &p2,const point3D &a,const point3D &b){
	point3D v=(p1-p0)^(p2-p0);if (sign(v*(b-a))==0)return make_pair(0,point3D());
	double t=(v*(p0-a))/(v*(b-a));if (sign(t)<0||sign(t-1)>0)return make_pair(0,point3D());
	point3D p=a+(b-a)*t;return make_pair(PointInTri(p,p0,p1,p2),p);
}
inline int direction(const point &p1,const point &p2,const point &p3){return sgn((p3-p1)^(p2-p1));}
inline bool iscross(const line &a,const line &b){
	const point &p1=a.x,&p2=a.y,&p3=b.x,&p4=b.y;
	int d1=direction(p3,p4,p1),
		d2=direction(p3,p4,p2),
		d3=direction(p1,p2,p3),
		d4=direction(p1,p2,p4);
	if (d1*d2==-1&&d3*d4==-1)return 1;
	else if (d1==0&&on_segment(p1,b))return 1;
	else if (d2==0&&on_segment(p2,b))return 1;
	else if (d3==0&&on_segment(p3,a))return 1;
	else if (d4==0&&on_segment(p4,a))return 1;
	else return 0;
}
inline double new_angle(double angle){
	angle+=2*PI;return angle-floor(angle/(2.0*PI))*2.0*PI;
}
inline double angle(const circle &O,const point &x){
	double angle=asin(x.y-O.y);if (x.x-O.x<0)angle=PI-angle;return angle;
}
struct polygon{
	vector<point> a;
	typedef vector<point>::iterator vit;
	void clear(){a.clear();}
	double area(){
		double res=0;
		for (vit i=a.begin(),next;i!=a.end();++i){
			next=i;++next; if (next==a.end())next=a.begin(); res+=(*i)^(*next);
		}
		return fabs(res/2);
	}
	bool inside(const point &p){
		bool in=0;
		for (vit i=a.begin(),next;i!=a.end();++i){
			next=i;++next; if (next==a.end())next=a.begin();
			if (iscross(line(*i,*next),line(p,point(123456,1e10))))in^=1;
		}
		return in;
	}
	point center(){  //center of gravity
		double sx=0,sy=0,area=0,tmp;
		for (vit i=a.begin(),next;i!=a.end();++i){
			next=i;++next; if (next==a.end())next=a.begin();tmp=(*i)^(*next);
			sx+=(i->x+next->x)*tmp;sy+=(i->y+next->y)*tmp;area+=tmp;
		}
		return point(sx/(3*area),sy/(3*area));
	} 
	void add(const point &p){a.push_back(p);}
	vector<point> ConvexHull(){
		vector<point> res,A=a;int *c=new int[a.size()*2+1],top=0;
		sort(A.begin(),A.end());
		for (int i=0;i<A.size();++i){
			while (top>1&&cha(A[i].x-A[c[top]].x,A[i].y-A[c[top]].y,A[c[top]].x-A[c[top-1]].x,A[c[top]].y-A[c[top-1]].y)>eps)--top;
			c[++top]=i;
		}
		for (int i=A.size()-2;i>=0;--i){
			while (top>1&&cha(A[i].x-A[c[top]].x,A[i].y-A[c[top]].y,A[c[top]].x-A[c[top-1]].x,A[c[top]].y-A[c[top-1]].y)>eps)--top;
			c[++top]=i;
		}
		for (int i=1;i<top;++i)res.push_back(A[c[i]]);
		delete[] c;return res;
	}
};
inline double area(const circle &c,const triangle &t){  //todo
	double res=0;return res;
}
inline double area(const circle &c,const polygon &p){
	double res=0;int n=p.a.size();
	for (int i=0;i<n;++i)res+=area(c,triangle(p.a[i],p.a[(i+1)%n],p.a[(i+2)%n]));
	return res;
}
inline bool in_poly(const circle &c,const polygon &p){return sign(area(c,p))==1;}

}


