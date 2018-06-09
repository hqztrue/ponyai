// Copyright @2018 Pony AI Inc. All rights reserved.

#include "perception/perception.h"

const double max_depth = 30;
void DrawPointCloudOnCameraImage(const PointCloud& pointcloud,
                                 const Eigen::VectorXd& intrinsic,
                                 const Eigen::Affine3d& extrinsic,
                                 cv::Mat* image) {
  const auto pixel_info = ProjectPointCloudToImage(pointcloud, intrinsic, extrinsic, 1920, 1080);
  for (const auto& pixel : pixel_info) {
    const cv::Point2d point(pixel.uv.u, pixel.uv.v);
    const double depth = pixel.position_in_camera_coordinate.norm();
    //const double hue = (math::Clamp(depth, 1.0, 150.0) - 1.0) / (150.0 - 1.0);
	const double hue = (math::Clamp(depth, 1.0, max_depth) - 1.0) / (max_depth - 1.0);
    const Eigen::Vector3d rgb = utils::display::HsvToRgb(Eigen::Vector3d(hue, 1.0, 1.0)) * 255;
    cv::circle(*image, point, 0, cv::Scalar(rgb.z(), rgb.y(), rgb.x()), 2);
  }
}

const double eps=1e-6;
struct point{
	double x,y,z;
	point(double _x=0,double _y=0,double _z=0):x(_x),y(_y),z(_z){}
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
	//friend istream& operator >>(istream &in,point &p){return in>>p.x>>p.y;}
	//friend ostream& operator <<(ostream &out,const point &p){return out<<"("<<p.x<<","<<p.y<<")";}
	void print()const{printf("(%.5lf,%.5lf)\n",x,y);}
};
inline double cha(double x1,double y1,double x2,double y2){return x1*y2-x2*y1;}
point max(const point &x,const point &y){
	return point(std::max(x.x,y.x), std::max(x.y,y.y), std::max(x.z,y.z));
}
point min(const point &x,const point &y){
	return point(std::min(x.x,y.x), std::min(x.y,y.y), std::min(x.z,y.z));
}
struct polygon{
	std::vector<point> a;
	typedef std::vector<point>::iterator vit;
	void clear(){a.clear();}
	/*double area(){
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
	}*/
	void add(const point &p){a.push_back(p);}
	std::vector<point> ConvexHull(){
		std::vector<point> res,A=a;int *c=new int[a.size()*2+1],top=0;
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

struct detection{
	double x,y,w,h,score;
	std::string label;
	detection(){}
	void print(){
		printf("det %lf %lf %lf %lf %lf %s\n",x,y,w,h,score,label.c_str());
	}
};

interface::perception::PerceptionObstacles Perception::RunPerception(
    const PointCloud &pointcloud, const utils::Optional<cv::Mat> &pimage, const Eigen::VectorXd &intrinsic, const Eigen::Affine3d &extrinsic, const char video_name[], int frameID)
{
    static int num_objects = 0;
    printf("video_name=%s, frameID=%d\n", video_name, frameID);
    interface::perception::PerceptionObstacles perception_result;

    assert(pimage);
    cv::Mat image = (*pimage).clone();
    auto pixel_info = ProjectPointCloudToImage(pointcloud, intrinsic, extrinsic, 1920, 1080);
    //{
    //draw
    DrawPointCloudOnCameraImage(pointcloud, intrinsic, extrinsic, &image);
    //cv::putText(image, image_path, cv::Point(10, 30),
    //            cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0));
    char fusion_output_path[205];
    sprintf(fusion_output_path, "/unsullied/sharefs/hqz/shared/tmp/fusion_output_/%d.png", frameID);
    //cv::imwrite(fusion_output_path, image);
    //puts("exit");exit(0);
    //cv::imshow("fusion demo", image);
    //cv::waitKey(0);
    //}

    //read detection
    char det_path[205];
    const char det_root_path[305] = "/unsullied/sharefs/hqz/shared/hw/pony_data/det";
    sprintf(det_path, "%s/%s/%d_det.txt", det_root_path, video_name, frameID);
    FILE *fin = fopen(det_path, "r");
    char label[205];
    detection d;
    std::vector<detection> dets;
    while (std::fscanf(fin, "%lf%lf%lf%lf%lf %s", &d.x, &d.y, &d.w, &d.h, &d.score, label) != EOF)
    {
        d.label = label;
        double rx = 1920. / 1280, ry = 1080. / 736;
        d.x *= rx;
        d.w *= rx;
        d.y *= ry;
        d.h *= ry;
        d.print();
        dets.push_back(d);
    }
    fclose(fin);
    for (int i = 0; i < dets.size(); ++i)
    {
        detection d = dets[i];
        std::vector<PixelInfo> det_pixels;
		for (auto & pixel : pixel_info)
        {
            int u = pixel.uv.u, v = pixel.uv.v;
            double depth = pixel.position_in_camera_coordinate.norm();
            if (depth <= max_depth)
            {
                if (u >= d.x && u < d.x + d.w && v >= d.y && v < d.y + d.h)
                {
                    cv::circle(image, cv::Point2d(pixel.uv.u, pixel.uv.v), 0, cv::Scalar(255, 0, 255), 2);
                    det_pixels.push_back(pixel);
                }
            }
        }
        d.print();
        cv::rectangle(image, cv::Point2d(d.x, d.y), cv::Point2d(d.x + d.w, d.y + d.h), cv::Scalar(255, 0, 0), 1, 1, 0);
        polygon poly;
		vector<cv2::Point> points;
		for (auto & pixel : det_pixels)
        {
            poly.add(point(pixel.position.x(), pixel.position.y(), pixel.position.z()));
			points,push_back(cv2::Point(pixel.position.x(), pixel.position.y()));
        }
        std::vector<point> pts = poly.ConvexHull();
        printf("#pts: %d #hull pts: %d\n", (int)det_pixels.size(), (int)pts.size());
		cv2::RotatedRect rectPoint = cv2::minAreaRect(points); 
		Point2f fourPoint2f[4];
		rectPoint.points(fourPoint2f);
		
		
        if (pts.size() > 0)
        {
            auto *obstacle = perception_result.add_obstacle();
            if (d.label == "person")obstacle->set_type(interface::perception::ObjectType::PEDESTRIAN);
            else obstacle->set_type(interface::perception::ObjectType::CAR);
            obstacle->set_id("o" + std::to_string(++num_objects));
            obstacle->set_height(2.69);

            /*for (auto &p : pts){
              auto* polygon_point = obstacle->add_polygon_point();
              polygon_point->set_x(p.x);
              polygon_point->set_y(p.y);
              polygon_point->set_z(p.z);
            }*/
            const double inf = 1e100;
            point mi(inf, inf, inf), ma(-inf, -inf, -inf);
			for (auto & p : pts)
            {
                mi = min(mi, p);
                ma = max(ma, p);
            }
            auto *polygon_point = obstacle->add_polygon_point();
            polygon_point->set_x(mi.x);
            polygon_point->set_y(mi.y);
            polygon_point->set_z(1);

            polygon_point = obstacle->add_polygon_point();
            polygon_point->set_x(ma.x);
            polygon_point->set_y(mi.y);
            polygon_point->set_z(1);

            polygon_point = obstacle->add_polygon_point();
            polygon_point->set_x(ma.x);
            polygon_point->set_y(ma.y);
            polygon_point->set_z(1);

            polygon_point = obstacle->add_polygon_point();
            polygon_point->set_x(mi.x);
            polygon_point->set_y(ma.y);
            polygon_point->set_z(1);
            printf("rect %lf %lf %lf %lf\n", mi.x, ma.x, mi.y, ma.y);
        }
    }
    cv::imwrite(fusion_output_path, image);


    /*if (image) {
      // Remove me if you don't want to pause the program every time.
      cv::namedWindow("camera");
      imshow("camera", *image);
      cv::waitKey(0);
    }*/


    LOG(INFO) << "Perception done.";
    return perception_result;
}

