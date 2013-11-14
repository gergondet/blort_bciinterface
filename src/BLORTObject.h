#include <bci-interface/DisplayObject/SSVEPStimulus.h>

#include <blort/Tracker/Resources.h>
#include <blort/Tracker/ModelLoader.h>
#include <blort/TomGine/tgCamera.h>
#include <blort/TomGine/tgPose.h>
#include <blort/blort/pal_util.h>

#include <ros/ros.h>
#include <blort_ros/TrackerResults.h>

#include <boost/thread.hpp>

class BLORTObject : public bciinterface::SSVEPStimulus
{
public:
    BLORTObject(ros::NodeHandle & nh, const std::string & object_name, const std::string & filename, int frequency, int screenFrequency, unsigned int wwidth, unsigned int wheight, unsigned int iwidth, unsigned int iheight);

    ~BLORTObject();

    virtual void Display(sf::RenderWindow * app, unsigned int frameCount, sf::Clock & clock);

    virtual bool DrawWithGL() { return true; }
private:
    void resultCallback(const blort_ros::TrackerResults::ConstPtr & trackerResult);
private:
    unsigned int wwidth;
    unsigned int wheight;
    unsigned int iwidth;
    unsigned int iheight;
    std::string object_name;
    std::string filename;
    Tracking::TrackerModel * model;
    TomGine::tgCamera camera;
    TomGine::tgPose pose;
    boost::mutex pose_mutex;
    ros::Subscriber sub;
};
