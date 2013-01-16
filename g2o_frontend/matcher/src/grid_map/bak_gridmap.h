#ifndef GRIDMAP_HH
#define GRIDMAP_HH

#include "stuff/array_allocator.h"
#include "math/transformation.h"
#include <iostream>

namespace AISNavigation{

/**Generic grid map class.
Supports copy constuction, assignment, resize, and indicidual cell access.*/
template <class T>
struct _GridMap{
  typedef T CellType;

  /**Mapping function between world coordinates and map cells.
   @param wp: the world point
   @returns the indices of the cell corresponding to wp in the map
  */
  inline Vector2i world2map(const Vector2& wp) const {
    return Vector2i(lrint((wp.x()-_offset.x())/_resolution),
		    lrint((wp.y()-_offset.y())/_resolution));
  }

  /**Mapping function between map cells and world coordinates.
   @param mp: the map cell
   @returns the coordinates of the cell mp in world coordinates
  */
  Vector2  map2world(const Vector2i& mp) const{
    return Vector2((double) _offset.x()+(_resolution*(double)mp.x()),
		    (double) _offset.y()+(_resolution*(double)mp.y()));
  }


  /**Boudary check
     @param mp: the cell to be checked
     @returns true if a cell is in the map, false otherwise
  */
  bool isInside(const Vector2i& mp) const {
    return mp.x()>=0 && mp.y()>=0 && mp.x()<size().x() && mp.y()<size().y();
  }


  /**Cell accessor method. Returns the cell located at the indices passed as argument.
     @param p: the cell indices
     @returns the cell at indices p in the map
  */
  inline T& cell(const Vector2i& p){
    return _allocator[p.x()][p.y()];
  }
  
  /**Const cell accessor method. Returns the cell located at the indices passed as argument.
     @param p: the cell indices
     @returns the cell at indices p in the map
  */
  inline const T& cell(const Vector2i& p) const{
    return _allocator[p.x()][p.y()];
  }


  inline double resolution() const {return _resolution;}
  inline Vector2i size()     const { Vector2i v; v[0]=_allocator.rows(), v[1]=_allocator.cols(); return v;}
  inline const Vector2& offset()    const {return _offset;}
  inline Vector2& offset()   {return _offset;}

  /**Constructs a n empty gridmap.*/
  _GridMap();

  /**Constructs a map of a  given size, offset and resolution. The mapped is filled with unknown cells.
  @param  size: the size in cells
  @param resolution: the resolution of the map
  @param offset: the location of the cell 0,0 in world coordinates
  @param unknownCell: the cell value to be used for filling the map
  */
  _GridMap(const Vector2i& size, const double& resolution, const Vector2& offset, const T& unknownCell=T());

  /**Resize operator
     It resizes the map so that the minimum represented world value will be in min and the maximum in max.
     Uninitialized cells will be padded with unknownval.
     @param min: the lower left corner in world coordinates
     @param max: the upper right corner in world coordinates
     @param unknownCell: the value to be used for padding new cells
  */
  _GridMap<T> resize(const Vector2 min, Vector2 max, T& unknownCell);

  bool load(std::istream& is);
  void save(std::ostream& os) const;

protected:
  /**the resolution of the grid map*/
  double _resolution;
  /**the size of the grid map in cells*/
  Vector2 _offset;
  _Array2DAllocator<0,0,CellType> _allocator;
};


template <class T>
  _GridMap<T>::_GridMap(): _allocator(0,0){
}

template <class T>
  _GridMap<T>::_GridMap(const Vector2i& s, const double& res, const Vector2& off, const T& unknownCell): _allocator(s.x(), s.y()){
  _allocator=_Array2DAllocator<0,0,CellType>(s.x(), s.y());
  _resolution=res;
  _offset=off;
  for (int i=0; i<size().x(); i++){
    for (int j=0; j<size().y(); j++){
      _allocator[i][j]=unknownCell;
    }
  }
}

template <class T>
_GridMap<T> _GridMap<T>::resize(const Vector2 min, Vector2 max, T& unknownCell){
  Vector2i newSize((int)((max.x()-min.x())/resolution()), (int) ((max.y()-min.y())/resolution()));
  _GridMap<T> newMap(newSize, resolution(), min, unknownCell);
  for (int i=0; i<newSize.x(); i++)
    for (int j=0; j<newSize.y(); j++){
      Vector2i c=world2map(newMap.map2world(Vector2i(i,j)));
      if (isInside(c)){
	newMap._allocator[i][j]=_allocator[c.x()][c.y()];
      }
    }
  return newMap;
}


template <class T>
bool _GridMap<T>::load(std::istream& is){
  std::string tag;
  while (is && tag!="#GRIDMAP")
    is >> tag;
  if (tag!="#GRIDMAP")
    return false;
  is >> tag;
  if (tag!="#SIZE")
    return false;
  Vector2i s;
  is >> s.x() >> s.y();
  is >> tag;
  if (tag!="#RESOLUTION")
    return false;
  double res;
  is >> res;
  is >> tag;
  if (tag!="#OFFSET")
    return false;
  Vector2 off;
  is >> off.x() >> off.y();
  T empty=T();
  *this=_GridMap<T>(s,res,off,empty);
  is >> tag;
  if (tag!="#CELLDATA_START")
    return false;
  for (int i=size().y()-1; i>=0; i--){
    for (int j=0; j<size().x(); j++){
      is >> cell(Vector2i(j,i));
    }
  }
  is >> tag;
  if (tag!="#CELLDATA_END")
    return false;
  return true;
}

template <class T>
void _GridMap<T>::save(std::ostream& os) const {
  os << "#GRIDMAP" << std::endl;
  os << "#SIZE " << size().x() << " " << size().y() << std::endl;
  os << "#RESOLUTION " << resolution() << std::endl;
  os << "#OFFSET " << offset().x() << " " << offset().y() << std::endl;
  os << "#CELLDATA_START" << std::endl;
  for (int i=size().y()-1; i>=0; i--){
    for (int j=0; j<size().x(); j++){
      os << cell(Vector2i(j,i)) << " ";
    }
    os << std::endl;
  }
  os << "#CELLDATA_END" << std::endl;
}
  
}; //namespace AISNavigation

#endif
