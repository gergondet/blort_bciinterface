#include "BLORTObject.h"
#include "BLORTObjectsManager.h"

#include <bci-interface/BCIInterface.h>
#include <bci-interface/Background/BufferBG.h>
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

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "bci");
    ros::NodeHandle nh;
    bool closed = false;

    BLORTObjectsManager bomanager(nh);

    unsigned int wwidth = 1024;
    unsigned int wheight = 768;
    unsigned int iwidth = 800;
    unsigned int iheight = 600;
    bciinterface::BCIInterface iface(wwidth, wheight);

    bciinterface::ROSBackground bg("/camera/rgb/image_color", wwidth, wheight, iwidth, iheight);
    iface.SetBackground(&bg);

    g_Resources->SetShaderPath("/home/gergondet/ros/perception_blort/blort_ros/Tracker/shader/");

    {
        BLORTObject * obj = new BLORTObject("can", "/home/gergondet/ros/perception_blort/blort_ros/Resources/ply/can.ply", 10, 60, wwidth, wheight, iwidth, iheight);
    
        iface.AddObject(obj);
        bomanager.AddObject(obj);
    }

    {
        BLORTObject * obj = new BLORTObject("thermos", "/home/gergondet/ros/perception_blort/blort_ros/Resources/ply/thermos.ply", 10, 60, wwidth, wheight, iwidth, iheight);
        iface.AddObject(obj);
        bomanager.AddObject(obj);
    }

    boost::thread th = boost::thread(boost::bind(&spinner, boost::ref(closed)));

    iface.DisplayLoop(false);
    closed = true;
    th.join();

    iface.SetBackground(0);

    return 0;
}
