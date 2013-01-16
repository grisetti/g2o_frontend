#ifndef _POINT_UTILS_HH_
#define _POINT_UTILS_HH_
#include <functional>
#include <ext/functional> // for 'compose1' and 'compose2'
#include <algorithm>
#include "vector_n.h"
#include "transformation.h"

using __gnu_cxx::compose1;      // note namespace!
using __gnu_cxx::compose2;


using namespace std;

template <typename Point>
struct isInsidePlane: public std::unary_function<Point, bool>{
  isInsidePlane (const Point& normal, const Point& origin): _origin(origin), _normal(normal) {}
  isInsidePlane (const Point& normal): _normal(_normal) { _origin=_origin*Point::BaseType(0.);}
  bool operator() (const Point& p) {
    return ( (p-_origin)*_normal > Point::BaseType(0.) );
  }
protected:
  Point& _origin;
  Point& _normal;
};

template <typename Point>
struct isInsideSphere: public std::unary_function<Point, bool>{
 isInsideSphere(typename Point::BaseType sqRadius, const Point& origin): _sqRadius(sqRadius),  _origin(origin) {}
 isInsideSphere(typename Point::BaseType sqRadius): _sqRadius(sqRadius) { _origin=_origin*Point::BaseType(0.);}
  bool operator() (const Point& p) {
    return ( (p-_origin)*(p-_origin)<_sqRadius );
  }
protected:
  typename Point::BaseType _sqRadius;
  Point& _origin;
};

template <typename Point>
struct PointLexCompare : public std::binary_function<Point, Point, bool> {
  bool operator () (const Point& p1, const Point& p2) const {
    //cerr << "p1=" << p1 << " p2=" << p2;
    for (int i=0; i<p1.size(); i++){
      if (p1[i]<p2[i]) {
 	//cerr <<"T" << endl;
	return true;
      } 
      if (p1[i]>p2[i]) {
 	//cerr <<"F" << endl;
	return false;
      }
    }
    //cerr << "E" << endl;
    return false;
  }
};

template <typename Point>
struct discretizeCartesian: public std::unary_function<Point, _Vector<Point::TemplateSize, int> >{
  typedef _Vector<Point::TemplateSize, int> DiscretizedType;
  discretizeCartesian() {_inverseResolution=1.;}
  discretizeCartesian(const typename Point::BaseType& res){
    _inverseResolution=1./res;
  }
  const typename Point::BaseType& resolution() const {return 1./_inverseResolution;}

  _Vector<Point::TemplateSize, int> operator() (const Point& p) const {
    _Vector<Point::TemplateSize, int> ip(p.size());
    for (int i=0; i<p.size(); i++)
      ip[i]=(int)(p[i]*_inverseResolution);
    return ip;
  }
  typename Point::BaseType _inverseResolution;
};

template <typename Transform, typename Compare> 
  struct TransformCompare : public std::binary_function< typename Transform::argument_type, typename Transform::argument_type, bool > {
    TransformCompare(Transform t, Compare c): _transform(t), _compare(c) {}
    bool operator () (const typename Transform::argument_type& a1, const typename Transform::argument_type& a2) const {
      return _compare(_transform(a1), _transform(a2));
    }
    Transform _transform;
    Compare _compare;
  };

template <typename PointContainer>
void sparsifyCartesian(PointContainer& container, const typename PointContainer::value_type::BaseType resolution){
  discretizeCartesian< typename PointContainer::value_type> disc(resolution);
  PointLexCompare< typename discretizeCartesian<typename PointContainer::value_type>::result_type > comp;
  TransformCompare< discretizeCartesian< typename PointContainer::value_type>,  PointLexCompare< typename discretizeCartesian<typename PointContainer::value_type>::result_type > > pred(disc,comp);
  std::sort(container.begin(), container.end(), pred );
  typename PointContainer::iterator it=unique(container.begin(), container.end(), not2(pred));
  container.erase(it,container.end());
};

template <typename PointContainer>
void radialClip(PointContainer& container, typename PointContainer::value_type::BaseType sqRadius, const typename PointContainer::value_type& origin, bool inside=true){
  isInsideSphere<typename PointContainer::value_type> pred(sqRadius, origin);
  if (inside)
    std::remove_if(container.begin(), container.end(), pred);
  else
    std::remove_if(container.begin(), container.end(), not1(pred) );
}

template <typename PointContainer>
void planarClip(PointContainer& container, const typename PointContainer::value_type& normal, const typename PointContainer::value_type& origin, bool inside=true){
  isInsidePlane<typename PointContainer::value_type> pred(normal, origin);
  if (inside)
    std::remove_if(container.begin(), container.end(), pred);
  else
    std::remove_if(container.begin(), container.end(), not1(pred));
}

template <typename Rotation, typename Translation>
  Translation apply(const _Transformation<Rotation>& tr, const Translation& p){
  return tr*p;
}

template <typename Rotation>
  _Transformation<Rotation> apply(const _Transformation<Rotation>& t0, const _Transformation<Rotation>& t1){
  return t0*t1;
}

#endif
