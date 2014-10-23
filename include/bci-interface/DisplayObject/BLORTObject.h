#ifndef _H_BLORTOBJECT_H_
#define _H_BLORTOBJECT_H_

#include <bci-interface/DisplayObject/SSVEPStimulus.h>
#ifdef HAS_CVEP_SUPPORT
#include <bci-interface/DisplayObject/CVEPStimulus.h>
#endif
#include <SFML/Graphics/Color.hpp>

#include <blort/Tracker/ModelLoader.h>
#include <blort/TomGine/tgCamera.h>
#include <blort/TomGine/tgPose.h>

#ifndef WIN32
#include <blort/blort/pal_util.h>
#include <ros/ros.h>
#endif

#include <boost/thread.hpp>

#include "BLORTObjectsManager.h"

class BLORTObject : public bciinterface::DisplayObject
{
public:
    BLORT_API BLORTObject(const std::string & object_name, const std::string & filename, const sf::Color & color, int frequency, int screenFrequency, int wwidth, int wheight, int iwidth, int iheight, BLORTObjectsManager & manager);

    #ifdef HAS_CVEP_SUPPORT
    BLORT_API BLORTObject(const std::string & object_name, const std::string & filename, const sf::Color & color, bciinterface::CVEPManager & cvep_manager, int wwidth, int wheight, int iwidth, int iheight, BLORTObjectsManager & manager);
    #endif

    BLORT_API ~BLORTObject();

    BLORT_API virtual void Display(sf::RenderTarget * app, unsigned int frameCount, sf::Clock & clock);

    BLORT_API virtual bool DrawWithGL() { return true; }

    BLORT_API virtual void Highlight();

    BLORT_API virtual void Unhighlight();

    BLORT_API void Update(const TomGine::tgPose & nPose);

    BLORT_API const std::string & getName() { return object_name; }

    BLORT_API void SetSubRect(int t, int l, int w, int h);
private:
    /* No copy */
    BLORTObject(const BLORTObject &);
    BLORTObject & operator=(const BLORTObject &);

    BLORTObjectsManager & manager;

    bciinterface::SSVEPStimulus * ssvep_stim;
    #ifdef HAS_CVEP_SUPPORT
    bciinterface::CVEPStimulus * cvep_stim;
    #endif

    struct BLORTObjectColor
    {
        float r;
        float g;
        float b;
        float a;
    };

    struct BLORTObjectBoundingBox
    {
      float xmin; float xmax;
      float ymin; float ymax;
      float zmin; float zmax;
      GLuint list;

      BLORTObjectBoundingBox() :
        xmin(1e6), xmax(-1e6),
        ymin(1e6), ymax(-1e6),
        zmin(1e6), zmax(-1e6),
        list(0)
      {}

      ~BLORTObjectBoundingBox();

      void Init();
      void Draw();
    };

    int wwidth;
    int wheight;
    int iwidth;
    int iheight;
    bool highlight;
    std::string object_name;
    std::string filename;
    Tracking::TrackerModel * model;
    BLORTObjectBoundingBox bb;
    BLORTObjectColor color;
    TomGine::tgCamera camera;
    TomGine::tgPose pose;
    boost::mutex pose_mutex;
    #ifndef WIN32
    ros::Time last_update;
    #endif

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
