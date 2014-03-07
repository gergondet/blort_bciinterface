#include <blort/Tracker/TrackerModel.h>
#include <blort/Tracker/ModelLoader.h>
#include <blort/Tracker/Resources.h>
#include <GL/gl.h>
#include <boost/bind.hpp>
#include <SFML/OpenGL.hpp>
#include <SFML/Graphics.hpp>
#include <iostream>

int main(int argc, char * argv[])
{
    sf::RenderWindow window(sf::VideoMode(640, 480), "OpenGL");
#ifdef WIN32
    glewInit();
#endif

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
#ifndef WIN32
    g_Resources->SetShaderPath("/home/gergondet/ros/perception_blort/blort_ros/Tracker/shader/");
#else
    g_Resources->SetShaderPath("C:/devel/share/blort_ros/Tracker/shader/");
#endif

    TomGine::tgCamera::Parameter camPar;
    TomGine::tgCamera camera;
    camPar.width = 640;
    camPar.height = 480;
    camPar.fx = 568.168672;
    camPar.fy = 566.360327;
    camPar.cx = 349.631248;
    camPar.cy = 256.295344;
    camPar.k1 = -0.059636;
    camPar.k2 = 0.232352;
    camPar.k3 = 0;
    camPar.p1 = -0.003432;
    camPar.p2 = 0.017361;
    camPar.pos.x = 0.31188;
    camPar.pos.y = -0.3;
    camPar.pos.z = 0.243049;
    vec3 rotv; rotv.x = -1.917587; rotv.y = -0.453044; rotv.z = 0.389306;
    camPar.rot.fromRotVector(rotv);
    camera.Load(camPar);

    Tracking::TrackerModel * model = new Tracking::TrackerModel();
    Tracking::ModelLoader loader;
#ifndef WIN32
    loader.LoadPly(*model, "/home/gergondet/ros/perception_blort/blort_ros/Resources/ply/can.ply");
#else
    loader.LoadPly(*model, "C:/devel/share/blort_ros/Resources/ply/Pringles.ply");
#endif

    bool running = true;
    while (running)
    {
        // handle events
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                // end the program
                running = false;
            }
            else if (event.type == sf::Event::Resized)
            {
                // adjust the viewport when the window is resized
                glViewport(0, 0, event.size.width, event.size.height);
            }
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        window.clear(sf::Color(0x77, 0x77, 0x77, 255));

        window.pushGLStates();
        window.popGLStates();
        camera.Activate();
        model->drawPass();

        window.display();
    }

    delete model;
    return 0;
}
