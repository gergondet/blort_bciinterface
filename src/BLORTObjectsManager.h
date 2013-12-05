#ifndef _H_BLORTOBJECTSMANAGER_H_
#define _H_BLORTOBJECTSMANAGER_H_

#include <vector>

#include <ros/ros.h>
#include <blort_ros/TrackerResults.h>

class BLORTObject;

class BLORTObjectsManager
{
    friend class BLORTObject;
public:
    BLORTObjectsManager(ros::NodeHandle & nh);

protected:
    void AddObject(BLORTObject * object);

    void RemoveObject(BLORTObject * object);
private:
    void resultCallback(const blort_ros::TrackerResults::ConstPtr & trackerResult);

    std::vector<BLORTObject *> objects;
    ros::Subscriber sub;
};

#endif
