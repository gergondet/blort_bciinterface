#include "BLORTObject.h"

BLORTObject::BLORTObject(const std::string & object_name, const std::string & filename, const std::string & filename_hl, int f, int screen, unsigned int wwidth, unsigned int wheight, unsigned int iwidth, unsigned int iheight, BLORTObjectsManager & manager)
: bciinterface::SSVEPStimulus(f, screen), manager(manager),
  wwidth(wwidth), wheight(wheight), iwidth(iwidth), iheight(iheight), highlight(false),
  object_name(object_name), filename(filename), filename_hl(filename_hl), model(0), model_hl(0), last_update(0)
{
    manager.AddObject(this);
}

BLORTObject::~BLORTObject()
{
    manager.RemoveObject(this);
    delete model;
    delete model_hl;
}

void BLORTObject::Display(sf::RenderTarget * app, unsigned int frameCount, sf::Clock & clock)
{
    if(!model)
    {
        /* TODO Get this from camera_info topic */
        TomGine::tgCamera::Parameter camPar;
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
        camera.Load(camPar);
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

    if( (ros::Time::now() - last_update).sec < 2 && DisplayActive(frameCount) )
    {
        boost::mutex::scoped_lock(pose_mutex);
        camera.Activate();
        pose.Activate();
        glViewport((wwidth - iwidth)/2, (wheight - iheight)/2, iwidth, iheight);
        if(highlight)
        {
            model_hl->drawPass();
        }
        else
        {
            model->drawPass();
        }
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
    last_update = ros::Time::now();
    pose = nPose;
}
