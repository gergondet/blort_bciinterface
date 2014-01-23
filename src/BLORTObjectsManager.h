#ifndef _H_BLORTOBJECTSMANAGER_H_
#define _H_BLORTOBJECTSMANAGER_H_

#include <vector>

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <blort_ros/TrackerResults.h>

#include <blort/TomGine/tgCamera.h>
#include <blort/TomGine/tgPose.h>
#include <blort/blort/pal_util.h>

class BLORTObject;

struct BLORTSubRect
{
    int left;
    int top;
    int width;
    int height;
};

class BLORTObjectsManager
{
    friend class BLORTObject;
public:
    BLORTObjectsManager(ros::NodeHandle & nh, const std::string & shader_path, bool ignore_blort = false);

    TomGine::tgPose GetObjectPosition(const std::string & obj_name);

    const TomGine::tgCamera::Parameter & GetCameraParameter() { return camPar; }

    /* Return a subrect view where all objects are located, the argument is an
     * estimation of the object bouding box in pixels */
    BLORTSubRect GetSubRect(int object_width);

protected:
    void AddObject(BLORTObject * object);

    void RemoveObject(BLORTObject * object);

private:
    void ProjectPoint(const TomGine::tgPose & pose, int & u, int & v);

    void resultCallback(const blort_ros::TrackerResults::ConstPtr & trackerResult);

    void ignoreBLORTCallback();
    boost::thread ignoreBLORTth;

    bool ignore_blort;
    TomGine::tgCamera::Parameter camPar;
    std::map<std::string, TomGine::tgPose> positions;
    std::vector<BLORTObject *> objects;
    ros::Subscriber sub;
};

#endif
