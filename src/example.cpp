#include "BLORTObject.h"

#include <bci-interface/BCIInterface.h>
#include <bci-interface/Background/BufferBG.h>
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
#include <boost/thread.hpp>

int main(int argc, char * argv[])
{
    unsigned int wwidth = 1024;
    unsigned int wheight = 768;
    unsigned int iwidth = 800;
    unsigned int iheight = 600;
    bciinterface::BCIInterface iface(wwidth, wheight);
    bciinterface::BufferBG * bufferBG = new bciinterface::BufferBG(640, 480, wwidth, wheight, iwidth, iheight);
    uint32_t * buffer = new uint32_t[640*480];
    for(unsigned int x = 0; x < 640; ++x)
    {
        for(unsigned int y = 0; y < 480; ++y)
        {
            buffer[640*y + x] = 0xFF0000FF;
            if(x >= 320 && y < 240)
            {
                buffer[640*y + x] = 0xFF00FF00;
            }
            if(x < 320 && y < 240)
            {
                buffer[640*y + x] = 0xFFFF0000;
            }
            if(x < 320 && y >= 240)
            {
                buffer[640*y + x] = 0xFF00FFFF;
            }
        }
    }
    bufferBG->SetSubRect(0, 0, 320, 240);
    bufferBG->SetSubRect(0, 0, 640, 480);
    bufferBG->UpdateFromBuffer_RGB((unsigned char*)buffer);
    iface.SetBackground(bufferBG);


    g_Resources->SetShaderPath("/home/gergondet/ros/perception_blort/blort_ros/Tracker/shader/");

    iface.AddObject(new BLORTObject("/home/gergondet/ros/perception_blort/blort_ros/Resources/ply/Pringles.ply", 10, 60, wwidth, wheight, iwidth, iheight));

    iface.DisplayLoop(false);

    delete bufferBG;

    return 0;
}
