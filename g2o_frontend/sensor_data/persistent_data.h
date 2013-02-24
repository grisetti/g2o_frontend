#ifndef _PERSISTENT_DATA_H_
#define _PERSISTENT_DATA_H_

#include <srtring>

/**
a persistent data is something that lives on the disk, and can be loaded and modifiedin memory.
During its existance, it is associated to a file, and the memory and disk states should be consistent.

*/
class PersistentData {
public:
  enum State={Invalid=0x0, // no data set or no fileset
	      Synced=0x1,  // data and the 
	      Modified=0x2};
  PersistentData(const std::string& filename_, size_t& offset);
  virtual ~PersistentData();
  virtual bool retrieve();
  virtual bool flush();
  inline const std::string& filename() const {return _filename;}
  inline size_t fileOffset fileOffset() const {return _fileOffset;}
protected:
  std::string _filename;
  size_t _fileOffset;
};

#endif
