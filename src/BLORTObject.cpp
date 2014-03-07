#include "BLORTObject.h"

BLORTObject::BLORTObject(const std::string & object_name, const std::string & filename, const std::string & filename_hl, const sf::Color & sfcolor, int f, int screen, int wwidth, int wheight, int iwidth, int iheight, BLORTObjectsManager & manager)
: manager(manager),
  wwidth(wwidth), wheight(wheight), iwidth(iwidth), iheight(iheight), highlight(false),
  object_name(object_name), filename(filename), filename_hl(filename_hl), model(0), model_hl(0),
#ifndef WIN32
  last_update(0),
#endif
  vp((wwidth - iwidth)/2, (wheight - iheight)/2, iwidth, iheight)
{
    ssvep_stim = new bciinterface::SSVEPStimulus(f, screen);
    color.r = (float)sfcolor.r/255.0f;
    color.g = (float)sfcolor.g/255.0f;
    color.b = (float)sfcolor.b/255.0f;
    color.a = (float)sfcolor.a/255.0f;
    manager.AddObject(this);
}

#ifdef HAS_CVEP_SUPPORT
BLORTObject::BLORTObject(const std::string & object_name, const std::string & filename, const std::string & filename_hl, const sf::Color & sfcolor, bciinterface::CVEPManager & cvep_manager, int wwidth, int wheight, int iwidth, int iheight, BLORTObjectsManager & manager)
: manager(manager),
  wwidth(wwidth), wheight(wheight), iwidth(iwidth), iheight(iheight), highlight(false),
  object_name(object_name), filename(filename), filename_hl(filename_hl), model(0), model_hl(0),
  last_update(0),
  vp((wwidth - iwidth)/2, (wheight - iheight)/2, iwidth, iheight)
{
    cvep_stim = new bciinterface::CVEPStimulus(cvep_manager);
    color.r = (float)sfcolor.r/255.0f;
    color.g = (float)sfcolor.g/255.0f;
    color.b = (float)sfcolor.b/255.0f;
    color.a = (float)sfcolor.a/255.0f;
    manager.AddObject(this);
}
#endif

BLORTObject::~BLORTObject()
{
    manager.RemoveObject(this);
    delete model;
    delete model_hl;
    delete ssvep_stim;
#ifdef HAS_CVEP_SUPPORT
    delete cvep_stim;
#endif
}

void BLORTObject::Display(sf::RenderTarget * app, unsigned int frameCount, sf::Clock & clock)
{
    if(!model)
    {
        /* TODO Get this from camera_info topic */
        camera.Load(manager.GetCameraParameter());
        pose.t = vec3(0.0, 0.1, 0.0);
        pose.Rotate(0.0f, 0.0f, 0.5f);
        model = new Tracking::TrackerModel();
        model->setBFC(false);
        model_hl = new Tracking::TrackerModel();
        model_hl->setBFC(false);
        Tracking::ModelLoader loader;
        loader.LoadPly(*model, filename.c_str());
        loader.LoadPly(*model_hl, filename_hl.c_str());
    }

#ifndef WIN32
    if( (ros::Time::now() - last_update).sec < 2 )
#endif
    {
        boost::mutex::scoped_lock(pose_mutex);
        camera.Activate();
        pose.Activate();
        glViewport(vp.left, vp.bottom, vp.width, vp.height);
        glEnable(GL_SCISSOR_TEST);
        glScissor((wwidth - iwidth)/2, (wheight - iheight)/2, iwidth, iheight);
        Tracking::TrackerModel * m = highlight ? model_hl : model;
        #ifdef HAS_CVEP_SUPPORT
        if( (ssvep_stim && ssvep_stim->DisplayActive(frameCount)) || (cvep_stim && cvep_stim->GetDisplay()) )
        #else
        if( (ssvep_stim && ssvep_stim->DisplayActive(frameCount)) )
        #endif
        {
            m->drawPass();
        }
        else
        {
            glDisable(GL_TEXTURE_2D);
            glColor4f(color.r, color.g, color.b, color.a);
            m->drawFaces();
        }
        glDisable(GL_SCISSOR_TEST);
        pose.Deactivate();
    }
}

void BLORTObject::Highlight()
{
    highlight = true;
}

void BLORTObject::Unhighlight()
{
    highlight = false;
}

void BLORTObject::Update(const TomGine::tgPose & nPose)
{
    boost::mutex::scoped_lock(pose_mutex);
    #ifndef WIN32
    last_update = ros::Time::now();
    #endif
    pose = nPose;
}

void BLORTObject::SetSubRect(int left, int top, int width, int height)
{
    double ratio_x = (double)iwidth/(double)width;
    double ratio_y = (double)iheight/(double)height;
    double scale_x = (double)manager.GetCameraParameter().width/(double)width;
    double scale_y = (double)manager.GetCameraParameter().height/(double)height;
    vp.width = scale_x*iwidth;
    vp.height = scale_y*iheight;
    vp.left = (wwidth - iwidth)/2 + ratio_x*(-left);
    vp.bottom = (wheight - iheight)/2 + ratio_y*(-(int)manager.GetCameraParameter().height + top + height);
}
