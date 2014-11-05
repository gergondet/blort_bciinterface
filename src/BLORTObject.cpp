#include "bci-interface/DisplayObject/BLORTObject.h"

BLORTObject::BLORTObject(const std::string & object_name, const std::string & filename, const sf::Color & sfcolor, int f, int screen, int wwidth, int wheight, int iwidth, int iheight, BLORTObjectsManager & manager)
: manager(manager),
  wwidth(wwidth), wheight(wheight), iwidth(iwidth), iheight(iheight), highlight(false),
  object_name(object_name), filename(filename), model(0),
#ifndef WIN32
  last_update(0),
#endif
  vp((wwidth - iwidth)/2, (wheight - iheight)/2, iwidth, iheight)
{
#ifdef HAS_CVEP_SUPPORT
    cvep_stim = 0;
#endif
    ssvep_stim = new bciinterface::SSVEPStimulus(f, screen);
    color.r = (float)sfcolor.r/255.0f;
    color.g = (float)sfcolor.g/255.0f;
    color.b = (float)sfcolor.b/255.0f;
    color.a = (float)sfcolor.a/255.0f;
    manager.AddObject(this);
}

#ifdef HAS_CVEP_SUPPORT
BLORTObject::BLORTObject(const std::string & object_name, const std::string & filename, const sf::Color & sfcolor, bciinterface::CVEPManager & cvep_manager, int wwidth, int wheight, int iwidth, int iheight, BLORTObjectsManager & manager)
: manager(manager),
  wwidth(wwidth), wheight(wheight), iwidth(iwidth), iheight(iheight), highlight(false),
  object_name(object_name), filename(filename), model(0),
  last_update(0),
  vp((wwidth - iwidth)/2, (wheight - iheight)/2, iwidth, iheight)
{
    ssvep_stim = 0;
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
    delete ssvep_stim;
#ifdef HAS_CVEP_SUPPORT
    delete cvep_stim;
#endif
}

BLORTObject::BLORTObjectBoundingBox::~BLORTObjectBoundingBox()
{
  glDeleteLists(list, 1);
}

void BLORTObject::BLORTObjectBoundingBox::Init()
{
  list = glGenLists(1);
  glNewList(list, GL_COMPILE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0,1,0,0.25);
    glLineWidth(4.0f);
    glBegin(GL_LINE_LOOP);
      glVertex3f(xmin, ymin, zmin); /*1*/
      glVertex3f(xmin, ymin, zmax); /*2*/
      glVertex3f(xmax, ymin, zmax); /*3*/
      glVertex3f(xmax, ymin, zmin); /*4*/
    glEnd();
    glBegin(GL_LINE_LOOP);
      glVertex3f(xmin, ymax, zmin); /*5*/
      glVertex3f(xmin, ymax, zmax); /*6*/
      glVertex3f(xmax, ymax, zmax); /*7*/
      glVertex3f(xmax, ymax, zmin); /*8*/
    glEnd();
    glBegin(GL_LINE_LOOP);
      glVertex3f(xmax, ymin, zmax);
      glVertex3f(xmax, ymin, zmin);
      glVertex3f(xmax, ymax, zmin);
      glVertex3f(xmax, ymax, zmax);
    glEnd();
    glBegin(GL_LINE_LOOP);
      glVertex3f(xmin, ymin, zmin);
      glVertex3f(xmin, ymin, zmax);
      glVertex3f(xmin, ymax, zmax);
      glVertex3f(xmin, ymax, zmin);
    glEnd();
    glBegin(GL_LINE_LOOP);
      glVertex3f(xmin, ymin, zmin); /*1*/
      glVertex3f(xmax, ymin, zmin); /*4*/
      glVertex3f(xmax, ymax, zmin); /*8*/
      glVertex3f(xmin, ymax, zmin); /*5*/
    glEnd();
    glBegin(GL_LINE_LOOP);
      glVertex3f(xmin, ymin, zmax); /*2*/
      glVertex3f(xmax, ymin, zmax); /*3*/
      glVertex3f(xmax, ymax, zmax); /*7*/
      glVertex3f(xmin, ymax, zmax); /*6*/
    glEnd();
  glEndList();
}

void BLORTObject::BLORTObjectBoundingBox::Draw()
{
  mat4 mv;
  mat3 rot;
  vec3 v_cam_object;
  float s = -0.001f;
  glGetFloatv(GL_MODELVIEW_MATRIX, mv);

  rot[0] = mv[0]; rot[1] = mv[4]; rot[2] = mv[8];
  rot[3] = mv[1]; rot[4] = mv[5]; rot[5] = mv[9];
  rot[6] = mv[2]; rot[7] = mv[6]; rot[8] = mv[10];

  v_cam_object[0] = mv[12];
  v_cam_object[1] = mv[13];
  v_cam_object[2] = mv[14];

  v_cam_object = rot * v_cam_object * s;

  glPushMatrix();
  glTranslatef(v_cam_object[0], v_cam_object[1], v_cam_object[2]);
  glCallList(list);
  glPopMatrix();
}

void BLORTObject::Display(sf::RenderTarget * app, unsigned int frameCount, sf::Clock & clock)
{
    if(!model)
    {
        /* TODO Get this from camera_info topic */
        camera.Load(manager.GetCameraParameter());
        model = new Tracking::TrackerModel();
        model->setBFC(false);
        Tracking::ModelLoader loader;
        loader.LoadPly(*model, filename.c_str());
        for(size_t i = 0; i < model->m_vertices.size(); ++i)
        {
          if(model->m_vertices[i].pos.x < bb.xmin) bb.xmin = model->m_vertices[i].pos.x;
          if(model->m_vertices[i].pos.x > bb.xmax) bb.xmax = model->m_vertices[i].pos.x;
          if(model->m_vertices[i].pos.y < bb.ymin) bb.ymin = model->m_vertices[i].pos.y;
          if(model->m_vertices[i].pos.y > bb.ymax) bb.ymax = model->m_vertices[i].pos.y;
          if(model->m_vertices[i].pos.z < bb.zmin) bb.zmin = model->m_vertices[i].pos.z;
          if(model->m_vertices[i].pos.z > bb.zmax) bb.zmax = model->m_vertices[i].pos.z;
        }
        bb.Init();
    }

#ifndef WIN32
    //if( (ros::Time::now() - last_update).sec < 2 )
#endif
    {
        boost::mutex::scoped_lock(pose_mutex);
        camera.Activate();
        pose.Activate();
        glViewport(vp.left, vp.bottom, vp.width, vp.height);
        glEnable(GL_SCISSOR_TEST);
        glScissor((wwidth - iwidth)/2, (wheight - iheight)/2, iwidth, iheight);
        Tracking::TrackerModel * m = model;
        #ifdef HAS_CVEP_SUPPORT
        if( (ssvep_stim && ssvep_stim->DisplayActive(frameCount)) || (cvep_stim && cvep_stim->GetDisplay()) )
        #else
        if( (ssvep_stim && ssvep_stim->DisplayActive(frameCount)) )
        #endif
        {
            #ifdef HAS_CVEP_SUPPORT
            if(cvep_stim)
            {
                glDisable(GL_TEXTURE_2D);
                glColor4f(color.r, color.g, color.b, color.a);
                m->drawFaces();
            }
            else
            #endif
            {
                m->drawPass();
            }
        }
        else
        {
            glDisable(GL_TEXTURE_2D);
            #ifdef HAS_CVEP_SUPPORT
            if(cvep_stim)
            {
                glColor4f(0,0,0, color.a);
            }
            else
            #endif
            {
                glColor4f(color.r, color.g, color.b, color.a);
            }
            m->drawFaces();
        }
        if(highlight)
        {
          bb.Draw();
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
