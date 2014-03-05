#ifndef _PWN_MERGER2_H
#define _PWN_MERGER2_H

#include "g2o_frontend/boss_map/eigen_boss_plugin.h" 
#include "g2o_frontend/boss/object_data.h"
#include "g2o_frontend/boss/identifiable.h"

#include "g2o_frontend/pwn_core/pinholepointprojector.h"
#include "g2o_frontend/pwn_core/depthimageconverterintegralimage.h"

#include "pwn_matcher_base.h"
#include "pwn_cloud_cache.h"

using namespace std;
  using namespace pwn;

namespace pwn_tracker {
	using namespace pwn_tracker;

  class Merger2 : public boss::Identifiable{
  public:  
    Merger2(int id =-1, boss::IdContext* context=0);
    virtual ~Merger2();

    inline void setDepthImageConverter(DepthImageConverter *depthImageConverter_) { _depthImageConverter = depthImageConverter_; }
	inline void setScale(float sc){_scale=sc;}

    inline DepthImageConverter* depthImageConverter() const { return _depthImageConverter; }

	void clear();
	void clearCloud();

	void merge(Eigen::Isometry3f& transform, Eigen::Isometry3f& offset, CloudWithImageSize* cloud);
	void mergeDepthImage(DepthImage& out,const DepthImage& scaledImage_current);

	void matchWithPartition(DepthImage& currentPartitionImage, Eigen::Isometry3f& offset, DepthImage& partitionMerged);

	void init();

	static void scale(DepthImage& dest, const DepthImage& src, int step);

    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);

	pwn::Cloud* _cloud_tot;
	pwn::Cloud* _bigCloud;
	pwn::Cloud* _currentBigCloud;
	DepthImage _depth_tot;
	DepthImage _image_pesi;
	int _image_points_count;
	int _image_overlapping_points_count;
	PwnMatcherBase* _matcher;
	PwnMatcherBase::MatcherResult _result;
	int _r;
	int _c;
    
  protected:

    DepthImageConverter *_depthImageConverter;

	std::vector<float> _pesi_tot;

	float _scale;

	Stats* sts;

	 private:
    pwn_boss::DepthImageConverter* _bconverter;
 
  };


}

#endif
