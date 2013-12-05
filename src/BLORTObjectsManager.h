#ifndef _H_BLORTOBJECTSMANAGER_H_
#define _H_BLORTOBJECTSMANAGER_H_

#include <vector>

#include <ros/ros.h>
#include <blort_ros/TrackerResults.h>

#include <blort/TomGine/tgPose.h>

class BLORTObject;

class BLORTObjectsManager
{
    friend class BLORTObject;
public:
    BLORTObjectsManager(ros::NodeHandle & nh);

    TomGine::tgPose GetObjectPosition(const std::string & obj_name);
protected:
    void AddObject(BLORTObject * object);

    void RemoveObject(BLORTObject * object);
private:
    void resultCallback(const blort_ros::TrackerResults::ConstPtr & trackerResult);

    std::map<std::string, TomGine::tgPose> positions;
    std::vector<BLORTObject *> objects;
    ros::Subscriber sub;
};

#endif
