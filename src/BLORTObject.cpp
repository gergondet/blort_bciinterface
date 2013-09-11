#include "BLORTObject.h"

BLORTObject::BLORTObject(const std::string & filename, int f, int screen, unsigned int wwidth, unsigned int wheight, unsigned int iwidth, unsigned int iheight)
: bciinterface::SSVEPStimulus(f, screen), 
  wwidth(wwidth), wheight(wheight), iwidth(iwidth), iheight(iheight), 
  filename(filename), model(0)
{
    TomGine::tgCamera::Parameter camPar;
    camPar.width = 640;
    camPar.height = 480;
    camPar.fx = 538.047058;
    camPar.fy = 538.226013;
    camPar.cx = 319.509888;
    camPar.cy = 239.332718;
    camPar.k1 = 0.031828;
    camPar.k2 = -0.106567;
    camPar.k3 = 0;
    camPar.p1 = -0.001167;
    camPar.p2 = 0.002602;
    camPar.pos.x = 0.31188; 
    camPar.pos.y = -0.3;
    camPar.pos.z = 0.243049;
    vec3 rotv; rotv.x = -1.917587; rotv.y = -0.453044; rotv.z = 0.389306;
    camPar.rot.fromRotVector(rotv);
    camPar.zNear = 0.1f;
    camPar.zFar = 5.0f;
    camera.Load(camPar);
    pose.t = vec3(0.0, 0.1, 0.0);
    pose.Rotate(0.0f, 0.0f, 0.5f);
    wratio = ((double)iwidth)/((double)camPar.width);
    hratio = ((double)iheight)/((double)camPar.height);
}

BLORTObject::~BLORTObject()
{
    delete model;
}

void BLORTObject::Display(sf::RenderWindow * app, unsigned int frameCount, sf::Clock & clock)
{
    if(!model)
    {
        model = new Tracking::TrackerModel();
        Tracking::ModelLoader loader;
        loader.LoadPly(*model, filename.c_str());
    }

    if(DisplayActive(frameCount))
    {
        camera.Activate();
        pose.Activate();
        glViewport((wwidth - iwidth)/2, (wheight - iheight)/2, iwidth, iheight);
        glScaled(wratio, hratio, 1);
        model->drawPass();
        pose.Deactivate();
    }
}
