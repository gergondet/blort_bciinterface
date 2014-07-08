#include <bci-interface/BCIInterface.h>
#include <bci-interface/EventHandler.h>
#include <bci-interface/Utils/FontManager.h>
#include <bci-interface/DisplayObject/FPSCounter.h>
#include <bci-interface/Background/BufferBG.h>
#include <bci-interface/CommandReceiver/UDPReceiver.h>
#include <bci-interface/CommandInterpreter/SimpleInterpreter.h>
#include "bci-interface/DisplayObject/BLORTObject.h"
#include "bci-interface/DisplayObject/BLORTObjectsManager.h"

#include <bci-interface/CommandOverrider.h>
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
#include <boost/thread.hpp>

#ifndef WIN32
#include <bci-interface/Background/ROSBackground.h>
#include <ros/package.h>
#endif

void spinner(bool & iface_closed)
{
#ifndef WIN32
    while(ros::ok() && !iface_closed)
    {
        ros::spinOnce();
    }
#endif
}

#ifndef WIN32
struct TestCameraSwitch : public bciinterface::EventHandler
{
    TestCameraSwitch(bciinterface::ROSBackground & bg, BLORTObjectsManager & manager, BLORTObject * object) : bg(bg), manager(manager), object(object), zoom_in(false), current_video_node("camera/rgb/image_color")
    {
    }

    virtual void Process(sf::Event & event)
    {
        if( event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Space )
        {
            if(current_video_node == "camera/rgb/image_color")
            {
                current_video_node = "vscore/image";
            }
            else
            {
                current_video_node = "camera/rgb/image_color";
            }
            bg.SetCameraTopic(current_video_node);
        }
        if( event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::S )
        {
            if(zoom_in)
            {
                bg.SetSubRect(0, 0, 640, 480);
                object->SetSubRect(0, 0, 640, 480);
            }
            else
            {
                BLORTSubRect srect = manager.GetSubRect(200);
                bg.SetSubRect(srect.left, srect.top, srect.width, srect.height);
                object->SetSubRect(srect.left, srect.top, srect.width, srect.height);
            }
            zoom_in = !zoom_in;
        }
    }

    bciinterface::ROSBackground & bg;
    BLORTObjectsManager & manager;
    BLORTObject * object;
    bool zoom_in;
    std::string current_video_node;
};
#endif

using namespace bciinterface;

int main(int argc, char * argv[])
{
#ifndef WIN32
    ros::init(argc, argv, "bci");
    ros::NodeHandle nh;
#endif
    bool closed = false;

    bool debug = false;
    if(argc > 1)
    {
        std::stringstream ss;
        ss << argv[1];
        ss >> debug;
    }

    unsigned int width = 1280;
    unsigned int height = 800;
    unsigned int rwidth = 640;
    unsigned int rheight = 480;
    BCIInterface iface(width, height);
#ifdef WIN32
    sf::Context context;
    glewInit();
#endif
    iface.InitOculus();

    #ifndef WIN32
    BLORTObjectsManager bomanager(nh, ros::package::getPath("blort_ros") + "/Tracker/shader/", debug);
    #else
    BLORTObjectsManager bomanager("C:/devel/share/blort_ros/Tracker/shader/", true);
    #endif

#ifndef WIN32
    FontManager fm;
    FPSCounter fps_c(fm.GetDefaultFont());
    iface.AddNonOwnedObject(&fps_c);
#endif

    rwidth = rwidth*iface.GetRenderScale();
    rheight = rheight*iface.GetRenderScale();

    UDPReceiver * receiver = new UDPReceiver(1111);
    SimpleInterpreter * interpreter = new SimpleInterpreter();
    iface.SetCommandReceiver(receiver);
    iface.SetCommandInterpreter(interpreter);

    CommandOverrider overrider;
    overrider.AddOverrideCommand(sf::Keyboard::Up, 1);
    overrider.AddOverrideCommand(sf::Keyboard::Right, 2);
    overrider.AddOverrideCommand(sf::Keyboard::Down, 3);
    overrider.AddOverrideCommand(sf::Keyboard::Left, 4);
    iface.SetCommandOverrider(&overrider);

#ifndef WIN32
    bciinterface::ROSBackground bg("/camera/rgb/image_raw", rwidth, rheight, rwidth, rheight);
    iface.SetBackground(&bg);
#endif

#ifndef WIN32
    BLORTObject * obj = new BLORTObject("coke", ros::package::getPath("blort_ros") + "/Resources/ply/coke.ply", ros::package::getPath("blort_ros") + "/Resources/ply/coke.ply", sf::Color(255, 0, 0, 255), 1, 60, rwidth, rheight, rwidth, rheight, bomanager);
#else
    BLORTObject * obj = new BLORTObject("can", "C:/devel/share/blort_ros/Resources/ply/Pringles.ply", "C:/devel/share/blort_ros/Resources/ply/Pringles.ply", sf::Color(255, 0, 0, 255), 1, 60, rwidth, rheight, rwidth, rheight, bomanager);
#endif
    iface.AddObject(obj);

#ifndef WIN32
    TestCameraSwitch tcs(bg, bomanager, obj);
    iface.AddEventHandler(&tcs);
#endif

    boost::thread th = boost::thread(boost::bind(&spinner, boost::ref(closed)));

    int cmd = -1;
    iface.OculusDisplayLoop(cmd);
    closed = true;
    th.join();

    iface.SetBackground(0);

    return 0;
}
