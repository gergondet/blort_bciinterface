#include "BLORTObject.h"
#include "BLORTObjectsManager.h"

#include <bci-interface/BCIInterface.h>
#include <bci-interface/Utils/FontManager.h>
#include <bci-interface/DisplayObject/FPSCounter.h>
#include <bci-interface/Background/BufferBG.h>
#include <bci-interface/CommandReceiver/UDPReceiver.h>
#include <bci-interface/CommandInterpreter/SimpleInterpreter.h>
#include <bci-interface/CommandOverrider.h>
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
#include <boost/thread.hpp>

#include <bci-interface/Background/ROSBackground.h>

void spinner(bool & iface_closed)
{
    while(ros::ok() && !iface_closed)
    {
        ros::spinOnce();
    }
}

using namespace bciinterface;

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "bci");
    ros::NodeHandle nh;
    bool closed = false;

    bool debug = false;
    if(argc > 1)
    {
        std::stringstream ss;
        ss << argv[1];
        ss >> debug;
    }
    BLORTObjectsManager bomanager(nh, debug);

    unsigned int width = 1280;
    unsigned int height = 800;
    unsigned int rwidth = 640;
    unsigned int rheight = 480;
    BCIInterface iface(width, height);
    iface.InitOculus("/home/gergondet/devel/share/OculusWindow");

    FontManager fm;
    FPSCounter fps_c(fm.GetDefaultFont());

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

    bciinterface::ROSBackground bg("/camera/rgb/image_color", rwidth, rheight, rwidth, rheight);
    iface.SetBackground(&bg);

    g_Resources->SetShaderPath("/home/gergondet/ros/perception_blort/blort_ros/Tracker/shader/");

    {
        BLORTObject * obj = new BLORTObject("can", "/home/gergondet/ros/perception_blort/blort_ros/Resources/ply/can.ply", "/home/gergondet/ros/perception_blort/blort_ros/Resources/ply/can_hl.ply", 1, 60, rwidth, rheight, rwidth, rheight, bomanager);
        iface.AddObject(obj);
    }
    {
        BLORTObject * obj = new BLORTObject("thermos", "/home/gergondet/ros/perception_blort/blort_ros/Resources/ply/thermos.ply", "/home/gergondet/ros/perception_blort/blort_ros/Resources/ply/thermos.ply", 1, 60, rwidth, rheight, rwidth, rheight, bomanager);
        iface.AddObject(obj);
    }
    iface.AddNonOwnedObject(&fps_c);

    boost::thread th = boost::thread(boost::bind(&spinner, boost::ref(closed)));

    int cmd = -1;
    iface.OculusDisplayLoop(cmd);
    closed = true;
    th.join();

    iface.SetBackground(0);

    return 0;
}
