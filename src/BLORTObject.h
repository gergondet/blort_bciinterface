#ifndef _H_BLORTOBJECT_H_
#define _H_BLORTOBJECT_H_

#include <bci-interface/DisplayObject/SSVEPStimulus.h>
#include <bci-interface/DisplayObject/CVEPStimulus.h>
#include <SFML/Graphics/Color.hpp>

#include <blort/Tracker/ModelLoader.h>
#include <blort/TomGine/tgCamera.h>
#include <blort/TomGine/tgPose.h>
#include <blort/blort/pal_util.h>

#include <ros/ros.h>

#include <boost/thread.hpp>

#include "BLORTObjectsManager.h"

class BLORTObject : public bciinterface::DisplayObject
{
public:
    BLORTObject(const std::string & object_name, const std::string & filename, const std::string & filename_hl, const sf::Color & color, int frequency, int screenFrequency, int wwidth, int wheight, int iwidth, int iheight, BLORTObjectsManager & manager);

    BLORTObject(const std::string & object_name, const std::string & filename, const std::string & filename_hl, const sf::Color & color, bciinterface::CVEPManager & cvep_manager, int wwidth, int wheight, int iwidth, int iheight, BLORTObjectsManager & manager);

    ~BLORTObject();

    virtual void Display(sf::RenderTarget * app, unsigned int frameCount, sf::Clock & clock);

    virtual bool DrawWithGL() { return true; }

    virtual void Highlight();

    virtual void Unhighlight();

    void Update(const TomGine::tgPose & nPose);

    const std::string & getName() { return object_name; }

    void SetSubRect(int t, int l, int w, int h);
private:
    /* No copy */
    BLORTObject(const BLORTObject &);
    BLORTObject & operator=(const BLORTObject &);

    BLORTObjectsManager & manager;

    bciinterface::SSVEPStimulus * ssvep_stim;
    bciinterface::CVEPStimulus * cvep_stim;

    struct BLORTObjectColor
    {
        float r;
        float g;
        float b;
        float a;
    };

    int wwidth;
    int wheight;
    int iwidth;
    int iheight;
    bool highlight;
    std::string object_name;
    std::string filename;
    std::string filename_hl;
    Tracking::TrackerModel * model;
    Tracking::TrackerModel * model_hl;
    BLORTObjectColor color;
    TomGine::tgCamera camera;
    TomGine::tgPose pose;
    boost::mutex pose_mutex;
    ros::Time last_update;

    struct Viewport
    {
        Viewport(int l, int b, int w, int h) : left(l), bottom(b), width(w), height(h) {}
        int left;
        int bottom;
        int width;
        int height;
    };
    Viewport vp;
};

#endif
