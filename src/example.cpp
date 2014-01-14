#include "BLORTObject.h"
#include "BLORTObjectsManager.h"

#include <bci-interface/BCIInterface.h>
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

    unsigned int wwidth = 1024;
    unsigned int wheight = 768;
    unsigned int iwidth = 800;
    unsigned int iheight = 600;
    BCIInterface iface(wwidth, wheight);
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

    bciinterface::ROSBackground bg("/camera/rgb/image_color", wwidth, wheight, iwidth, iheight);
    iface.SetBackground(&bg);

    g_Resources->SetShaderPath("/home/gergondet/ros/perception_blort/blort_ros/Tracker/shader/");

    {
        BLORTObject * obj = new BLORTObject("can", "/home/gergondet/ros/perception_blort/blort_ros/Resources/ply/can.ply", "/home/gergondet/ros/perception_blort/blort_ros/Resources/ply/can_hl.ply", 1, 60, wwidth, wheight, iwidth, iheight, bomanager);
        iface.AddObject(obj);
    }
    {
        BLORTObject * obj = new BLORTObject("thermos", "/home/gergondet/ros/perception_blort/blort_ros/Resources/ply/thermos.ply", "/home/gergondet/ros/perception_blort/blort_ros/Resources/ply/thermos.ply", 1, 60, wwidth, wheight, iwidth, iheight, bomanager);
        iface.AddObject(obj);
    }

    boost::thread th = boost::thread(boost::bind(&spinner, boost::ref(closed)));

    iface.DisplayLoop(false);
    closed = true;
    th.join();

    iface.SetBackground(0);

    return 0;
}
