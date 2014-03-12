#include "BLORTObject.h"
#include "BLORTObjectsManager.h"

#include <bci-interface/BCIInterface.h>
#include <bci-interface/EventHandler.h>
#include <bci-interface/Background/BufferBG.h>
#include <bci-interface/CommandReceiver/UDPReceiver.h>
#include <bci-interface/CommandInterpreter/SimpleInterpreter.h>
#include <bci-interface/CommandOverrider.h>
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
#include <boost/thread.hpp>

#ifndef WIN32
#include <bci-interface/Background/ROSBackground.h>
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

struct TestCameraSwitch : public bciinterface::EventHandler
{
#ifndef WIN32
    TestCameraSwitch(bciinterface::ROSBackground & bg, BLORTObjectsManager & manager, BLORTObject * object) : bg(bg), manager(manager), object(object), zoom_in(false), current_video_node("camera/rgb/image_color")
#else
    TestCameraSwitch(BLORTObjectsManager & manager, BLORTObject * object) : manager(manager), object(object), zoom_in(false), current_video_node("camera/rgb/image_color")
#endif
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
#ifndef WIN32
            bg.SetCameraTopic(current_video_node);
#endif
        }
        if( event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::S )
        {
#ifndef WIN32
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
#endif
        }
    }
#ifndef WIN32
    bciinterface::ROSBackground & bg;
#endif
    BLORTObjectsManager & manager;
    BLORTObject * object;
    bool zoom_in;
    std::string current_video_node;
};

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

    int wwidth = 1024;
    int wheight = 768;
    int iwidth = 800;
    int iheight = 600;
    BCIInterface iface(wwidth, wheight);
#ifdef WIN32
    sf::Context context;
    glewInit();
#endif
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
    BLORTObjectsManager bomanager(nh, "/home/gergondet/ros/perception_blort/blort_ros/Tracker/shader/", debug);
#else
    BLORTObjectsManager bomanager("C:/devel/share/blort_ros/Tracker/shader/", true);
#endif


#ifndef WIN32
    bciinterface::ROSBackground bg("/camera/rgb/image_raw", wwidth, wheight, iwidth, iheight);
    iface.SetBackground(&bg);
#endif

#ifndef WIN32
    BLORTObject * obj = new BLORTObject("can", "/home/gergondet/ros/perception_blort/blort_ros/Resources/ply/can.ply", "/home/gergondet/ros/perception_blort/blort_ros/Resources/ply/can_hl.ply", sf::Color(255, 0, 0, 255), 1, 60, wwidth, wheight, iwidth, iheight, bomanager);
#else
    BLORTObject * obj = new BLORTObject("can", "C:/devel/share/blort_ros/Resources/ply/Pringles.ply", "C:/devel/share/blort_ros/Resources/ply/Pringles.ply", sf::Color(255, 0, 0, 255), 1, 60, wwidth, wheight, iwidth, iheight, bomanager);
#endif
    iface.AddObject(obj);

#ifndef WIN32
    TestCameraSwitch tcs(bg, bomanager, obj);
#else
    TestCameraSwitch tcs(bomanager, obj);
#endif
    iface.AddEventHandler(&tcs);

    boost::thread th = boost::thread(boost::bind(&spinner, boost::ref(closed)));

    iface.DisplayLoop(false);
    closed = true;
    th.join();

    iface.SetBackground(0);

    return 0;
}
