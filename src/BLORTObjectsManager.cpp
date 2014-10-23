#include <blort/Tracker/Resources.h>
#include "bci-interface/DisplayObject/BLORTObjectsManager.h"
#include "bci-interface/DisplayObject/BLORTObject.h"

#ifndef WIN32
BLORTObjectsManager::BLORTObjectsManager(ros::NodeHandle & nh, const std::string & shader_path, bool ignore_blort) : ignore_blort(ignore_blort), objects(0)
#else
BLORTObjectsManager::BLORTObjectsManager(const std::string & shader_path, bool ignore_blort) : ignore_blort(ignore_blort), objects(0)
#endif
{
    g_Resources->SetShaderPath(shader_path.c_str());
    if(!ignore_blort)
    {
        #ifndef WIN32
        sub = nh.subscribe("/blort_tracker/detection_result", 100, &BLORTObjectsManager::resultCallback, this);
        #endif
    }
    else
    {
        ignoreBLORTth = boost::thread(boost::bind(&BLORTObjectsManager::ignoreBLORTCallback, this));
    }
    /*FIXME Get from ROS topic */
    camPar.width = 640;
    camPar.height = 480;
    camPar.fx = 538.04704900000002;
    camPar.fy = 538.22603900000001;
    camPar.cx = 319.50987400000002;
    camPar.cy = 239.33272500000001;
    camPar.k1 = 0.031828000000000002;
    camPar.k2 = -0.106567;
    camPar.k3 = 0;
    camPar.p1 = -0.0011670000000000001;
    camPar.p2 = 0.0026020000000000001;
    camPar.pos.x = 0;
    camPar.pos.y = 0;
    camPar.pos.z = 0;
    vec3 rotv; rotv.x = 0; rotv.y = 0; rotv.z = 0;
    camPar.rot.fromRotVector(rotv);
    camPar.zNear = 0.1f;
    camPar.zFar = 5.0f;
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

BLORTSubRect BLORTObjectsManager::GetSubRect(int object_size)
{
    BLORTSubRect res;
    int minX = camPar.width; int maxX = 0;
    int minY = camPar.height; int maxY = 0;
    for(std::map<std::string, TomGine::tgPose>::iterator it = positions.begin(); it != positions.end(); ++it)
    {
        int u = 0; int v = 0;
        ProjectPoint(it->second, u, v);
        if(u < minX) { minX = u; }
        if(u > maxX) { maxX = u; }
        if(v < minY) { minY = v; }
        if(v > maxY) { maxY = v; }
    }
    minX = minX > object_size/2 ? minX - object_size/2 : 0;
    maxX = maxX + object_size/2 > camPar.height - 1 ? camPar.height - 1 : maxX + object_size/2;
    minY = minY > object_size/2 ? minY - object_size/2 : 0;
    maxY = maxY + object_size/2 > camPar.height - 1 ? camPar.height - 1 : maxY + object_size/2;
    res.left = minX;
    res.top = minY;
    res.width = maxX - minX;
    res.height = maxY - minY;
    /* Preserve aspect ratio */
    if(res.width > res.height)
    {
        int heightDiff = (double)camPar.height/(double)camPar.width * res.width - res.height;
        res.height += heightDiff;
        res.top -= heightDiff/2;
        res.top = res.top < 0 ? 0 : res.top;
    }
    else
    {
        int widthDiff = (double)camPar.width/(double)camPar.height * res.height - res.width;
        res.width += widthDiff;
        res.left -= widthDiff/2;
        res.left = res.left < 0 ? 0 : res.left;
    }
    return res;
}

void BLORTObjectsManager::AddObject(BLORTObject * object)
{
    objects.push_back(object);
//    if(ignore_blort)
//    {
//        TomGine::tgPose defPose;
//        defPose.t.x = 0.134408188062; defPose.t.y = -0.00268787337344; defPose.t.z = 0.417594547483;
//        defPose.q.x = -0.565049071946; defPose.q.y = -0.370892131447; defPose.q.z = 0.440585349627; defPose.q.w = 0.590798715992;
//        positions[object->getName()] = defPose;
//    }
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

void BLORTObjectsManager::ProjectPoint(const TomGine::tgPose & pose, int & u, int & v)
{
    if(pose.t.z == 0) { u = -1; v = -1; return; }
    double xp = pose.t.x/pose.t.z;
    double yp = pose.t.y/pose.t.z;
    double r2 = xp*xp + yp*yp;
    double xpp = xp*(1 + r2*(camPar.k1 + r2*(camPar.k2 + r2*camPar.k3))) + 2*camPar.p1*xp*yp + camPar.p2*(r2 + 2*xp*xp);
    double ypp = yp*(1 + r2*(camPar.k1 + r2*(camPar.k2 + r2*camPar.k3))) + camPar.p1*(r2 + 2*yp*yp) + 2*camPar.p2*xp*yp;
    u = camPar.fx*xpp + camPar.cx;
    v = camPar.fy*ypp + camPar.cy;
}

#ifndef WIN32
void BLORTObjectsManager::resultCallback(const blort_msgs::TrackerResults::ConstPtr & trackerResult)
{
    TomGine::tgPose pInCam;
    pInCam.t.x = trackerResult->pose.pose.position.x;
    pInCam.t.y = trackerResult->pose.pose.position.y;
    pInCam.t.z = trackerResult->pose.pose.position.z;
    pInCam.q.x = trackerResult->pose.pose.orientation.x;
    pInCam.q.y = trackerResult->pose.pose.orientation.y;
    pInCam.q.z = trackerResult->pose.pose.orientation.z;
    pInCam.q.w = trackerResult->pose.pose.orientation.w;
    positions[trackerResult->obj_name.data] = pInCam;
    for(size_t i = 0; i < objects.size(); ++i)
    {
        if( objects[i]->getName() == (trackerResult->obj_name.data) )
        {
            pInCam.q.x = -pInCam.q.x;
            pInCam.q.y = -pInCam.q.y;
            pInCam.q.z = -pInCam.q.z;
            objects[i]->Update(pInCam);
            break;
        }
    }
}
#endif

void BLORTObjectsManager::ignoreBLORTCallback()
{
    #ifndef WIN32
    ros::Rate rt(10);
    while(ros::ok())
    #else
    while(1)
    #endif
    {
        for(size_t i = 0; i < objects.size(); ++i)
        {
            TomGine::tgPose pInCam;
            pInCam.t.x = 0.134408188062; pInCam.t.y = -0.00268787337344; pInCam.t.z = 0.417594547483;
            pInCam.q.x = -0.565049071946; pInCam.q.y = -0.370892131447; pInCam.q.z = 0.440585349627; pInCam.q.w = 0.590798715992;
            objects[i]->Update(pInCam);
        }
        #ifndef WIN32
        rt.sleep();
        #else
        Sleep(100);
        #endif
    }
}
