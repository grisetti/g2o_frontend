#include "pwn_tracker_frame.h"

namespace pwn_tracker {
  PwnTrackerFrame::PwnTrackerFrame(MapManager* manager, int id, IdContext* context):
    MapNode ( manager, id, context) {
    sensorOffset.setIdentity();
    scale=1;
    cloud=0;
  }
  
  void PwnTrackerFrame::serialize(ObjectData& data, IdContext& context) {
    MapNode::serialize(data,context);
    sensorOffset.matrix().toBOSS(data, "sensorOffset");
    cameraMatrix.toBOSS(data, "cameraMatrix");
    data.setInt("imageRows", imageRows);
    data.setInt("imageCols", imageCols);

    ObjectData* depthImageData=new ObjectData();
    data.setField("depthImage", depthImageData);
    depthImage.serialize(*depthImageData,context);

    ObjectData* depthThumbnailData=new ObjectData();
    data.setField("depthThumbnail", depthThumbnailData);
    depthThumbnail.serialize(*depthThumbnailData,context);


    ObjectData* normalThumbnailData=new ObjectData();
    data.setField("normalThumbnail", normalThumbnailData);
    normalThumbnail.serialize(*normalThumbnailData,context);

    // ObjectData* cloudData=new ObjectData();
    // data.setField("cloud", cloudData);
    // cloud.serialize(*cloudData,context);
  }

  
  void PwnTrackerFrame::deserialize(ObjectData& data, IdContext& context) {
    MapNode::deserialize(data,context);
    sensorOffset.matrix().fromBOSS(data,"sensorOffset");
    cameraMatrix.fromBOSS(data, "cameraMatrix");
    cameraMatrix(2,2)=1;
    imageRows = data.getInt("imageRows");
    imageCols = data.getInt("imageCols");

    ObjectData* depthImageData = static_cast<ObjectData*>(data.getField("depthImage"));
    depthImage.deserialize(*depthImageData, context);

    
    ObjectData* normalThumbnailData = static_cast<ObjectData*>(data.getField("normalThumbnail"));
    normalThumbnail.deserialize(*normalThumbnailData,context);

    ObjectData* depthThumbnailData = static_cast<ObjectData*>(data.getField("depthThumbnail"));
    depthThumbnail.deserialize(*depthThumbnailData,context);

    // ObjectData* cloudData = static_cast<ObjectData*>(data.getField("cloud"));
    // cloud.deserialize(*cloudData,context);
  }

}
