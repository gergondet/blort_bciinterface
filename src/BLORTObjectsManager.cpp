#include "BLORTObjectsManager.h"

BLORTObjectsManager::BLORTObjectsManager(ros::NodeHandle & nh) : objects(0)
{
    sub = nh.subscribe("/blort_tracker/detection_result", 100, &BLORTObjectsManager::resultCallback, this);
}

void BLORTObjectsManager::AddObject(BLORTObject * object)
{
    objects.push_back(object);
}

void BLORTObjectsManager::ClearObjects()
{
    objects.clear();
}

void BLORTObjectsManager::resultCallback(const blort_ros::TrackerResults::ConstPtr & trackerResult)
{
    for(size_t i = 0; i < objects.size(); ++i)
    {
        if( objects[i]->getName() == (trackerResult->obj_name.data) )
        {
            objects[i]->Update(trackerResult);
            break;
        }
    }
}
