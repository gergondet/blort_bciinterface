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
    positions[trackerResult->obj_name.data] = pal_blort::rosPose2TgPose(trackerResult->poseInCamera);
    positions[trackerResult->obj_name.data].q.x = -positions[trackerResult->obj_name.data].q.x;
    positions[trackerResult->obj_name.data].q.y = -positions[trackerResult->obj_name.data].q.y;
    positions[trackerResult->obj_name.data].q.z = -positions[trackerResult->obj_name.data].q.z;
    for(size_t i = 0; i < objects.size(); ++i)
    {
        if( objects[i]->getName() == (trackerResult->obj_name.data) )
        {
            objects[i]->Update(positions[trackerResult->obj_name.data]);
            break;
        }
    }
}
