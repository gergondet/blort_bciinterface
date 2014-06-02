#ifndef _H_BLORTOBJECTSMANAGER_H_
#define _H_BLORTOBJECTSMANAGER_H_

#include <vector>

#include <boost/thread.hpp>

#ifndef WIN32
#include <ros/ros.h>
#include <blort_msgs/TrackerResults.h>
#define BLORT_API
#else
#include <blort/api.h>
#endif

#include <blort/TomGine/tgCamera.h>
#include <blort/TomGine/tgPose.h>

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
    #ifndef WIN32
    BLORTObjectsManager(ros::NodeHandle & nh, const std::string & shader_path, bool ignore_blort = false);
    #else
    BLORT_API BLORTObjectsManager(const std::string & shader_path, bool ignore_blort = true);
    #endif

    BLORT_API TomGine::tgPose GetObjectPosition(const std::string & obj_name);

    BLORT_API const TomGine::tgCamera::Parameter & GetCameraParameter() { return camPar; }

    /* Return a subrect view where all objects are located, the argument is an
     * estimation of the object bouding box in pixels */
    BLORT_API BLORTSubRect GetSubRect(int object_width);

protected:
    BLORT_API void AddObject(BLORTObject * object);

    BLORT_API void RemoveObject(BLORTObject * object);

private:
    BLORT_API void ProjectPoint(const TomGine::tgPose & pose, int & u, int & v);

    #ifndef WIN32
    void resultCallback(const blort_msgs::TrackerResults::ConstPtr & trackerResult);
    #endif

    BLORT_API void ignoreBLORTCallback();
    boost::thread ignoreBLORTth;

    bool ignore_blort;
    TomGine::tgCamera::Parameter camPar;
    std::map<std::string, TomGine::tgPose> positions;
    std::vector<BLORTObject *> objects;
    #ifndef WIN32
    ros::Subscriber sub;
    #endif
};

#endif
