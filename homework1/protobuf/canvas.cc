// Copyright @2018 Pony AI Inc. All rights reserved.

#include "homework1/protobuf/canvas.h"

#include <iostream>
#include<cmath>
#include <glog/logging.h>

namespace homework1 {

using homework1::geometry::Point3D;

double dist(const Point3D& p, const Point3D& q){
    return sqrt((p.x()-q.x())*(p.x()-q.x())+(p.y()-q.y())*(p.y()-q.y())+(p.z()-q.z())*(p.z()-q.z()));
}

void Canvas::Draw() const {
  for (const auto& p : polygon_.point()) {
    std::cout << "Point:" << p.x() << ", " << p.y() << ", " << p.z() << std::endl;
  }
}

void Canvas::AddPoint(double x, double y, double z) {
  Point3D point;
  point.set_x(x);
  point.set_y(y);
  point.set_z(z);
  AddPoint(point);
}

void Canvas::AddPoint(const Point3D& p) {
  auto* point = polygon_.add_point();
  point->CopyFrom(p);
}

void Canvas::AddPoint_(double x, double y, double z) {
  Point3D point;
  point.set_x(x);
  point.set_y(y);
  point.set_z(z);
  AddPoint_(point);
}

void Canvas::AddPoint_(const Point3D& p) {
  auto* point = polyline_.add_point();
  point->CopyFrom(p);
}

double Canvas::length()const{
    int n = polyline_.point_size();
    double ans=0;
    for (int i=0;i<n-1;++i)
        ans += dist(polyline_.point(i), polyline_.point(i+1));
    return ans;
}

const Point3D& Canvas::GetPoint(int index) const {
  return polygon_.point(index);
}

void Canvas::ParseFromString(const std::string& serialzation) {
  polygon_.ParseFromString(serialzation);
}

const std::string Canvas::SerializeToString() const {
  std::string serialzation;
  CHECK(polygon_.SerializeToString(&serialzation)) << "Canvas serialization failed.";
  return serialzation;
}

void Canvas::ParseFromString_(const std::string& serialzation) {
  polyline_.ParseFromString(serialzation);
}

const std::string Canvas::SerializeToString_() const {
  std::string serialzation;
  CHECK(polyline_.SerializeToString(&serialzation)) << "Canvas serialization failed.";
  return serialzation;
}
}  // namespace homework1

