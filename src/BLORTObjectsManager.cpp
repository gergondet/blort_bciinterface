#include "BLORTObjectsManager.h"

#include "BLORTObject.h"

BLORTObjectsManager::BLORTObjectsManager(ros::NodeHandle & nh, bool ignore_blort) : ignore_blort(ignore_blort), objects(0)
{
    if(!ignore_blort)
    {
        sub = nh.subscribe("/blort_tracker/detection_result", 100, &BLORTObjectsManager::resultCallback, this);
    }
    else
    {
        ignoreBLORTth = boost::thread(boost::bind(&BLORTObjectsManager::ignoreBLORTCallback, this));
    }
}

TomGine::tgPose BLORTObjectsManager::GetObjectPosition(const std::string & obj_name)
{
    if(positions.count(obj_name))
    {
        return positions[obj_name];
    }
    std::cerr << "Did not receive information about " << obj_name << ", giving default position" << std::endl;
    TomGine::tgPose ret;
    return ret;
}

void BLORTObjectsManager::AddObject(BLORTObject * object)
{
    objects.push_back(object);
    if(ignore_blort)
    {
        TomGine::tgPose defPose;
        defPose.t.x = 0.0359699837616; defPose.t.y = 0.0921241587674; defPose.t.z = 0.563517731061;
        defPose.q.x = 0.526820703494; defPose.q.y = -0.489232684931; defPose.q.z = 0.492633599549; defPose.q.w = 0.490329953155;
        positions[object->getName()] = defPose;
    }
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
    positions[trackerResult->obj_name.data] = pal_blort::rosPose2TgPose(trackerResult->pose.pose);
    for(size_t i = 0; i < objects.size(); ++i)
    {
        if( objects[i]->getName() == (trackerResult->obj_name.data) )
        {
            TomGine::tgPose pInCam = pal_blort::rosPose2TgPose(trackerResult->pose.pose);
            pInCam.q.x = -pInCam.q.x;
            pInCam.q.y = -pInCam.q.y;
            pInCam.q.z = -pInCam.q.z;
            objects[i]->Update(pInCam);
            break;
        }
    }
}

void BLORTObjectsManager::ignoreBLORTCallback()
{
    ros::Rate rt(10);
    while(ros::ok())
    {
        for(size_t i = 0; i < objects.size(); ++i)
        {
            TomGine::tgPose pInCam;
            pInCam.t.x = 0.0917806476355; pInCam.t.y = 0.137198194861; pInCam.t.z = -0.0531721040606;
            pInCam.q.x = 0.125980451703; pInCam.q.y = -0.127102732658; pInCam.q.z = -0.838982820511; pInCam.q.w = 0.513888657093;
            objects[i]->Update(pInCam);
        }
        rt.sleep();
    }
}
