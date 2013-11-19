#ifndef _H_BLORTOBJECSTMANAGER_H_
#define _H_BLORTOBJECTSMANAGER_H_

#include "BLORTObject.h"
#include <vector>

#include <ros/ros.h>
#include <blort_ros/TrackerResults.h>

class BLORTObjectsManager
{
public:
    BLORTObjectsManager(ros::NodeHandle & nh);

    void AddObject(BLORTObject * object);

    void ClearObjects();
private:
    void resultCallback(const blort_ros::TrackerResults::ConstPtr & trackerResult);

    std::vector<BLORTObject *> objects;
    ros::Subscriber sub;
};

#endif
