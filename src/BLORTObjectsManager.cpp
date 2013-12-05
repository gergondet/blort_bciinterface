#include "BLORTObjectsManager.h"

#include "BLORTObject.h"

BLORTObjectsManager::BLORTObjectsManager(ros::NodeHandle & nh) : objects(0)
{
    sub = nh.subscribe("/blort_tracker/detection_result", 100, &BLORTObjectsManager::resultCallback, this);
}

void BLORTObjectsManager::AddObject(BLORTObject * object)
{
    objects.push_back(object);
}

void BLORTObjectsManager::RemoveObject(BLORTObject * object)
{
    for(std::vector<BLORTObject*>::iterator it = objects.begin(); it != objects.end(); ++it)
    {
        if(*it == object)
        {
            objects.erase(it);
            break;
        }
    }
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
