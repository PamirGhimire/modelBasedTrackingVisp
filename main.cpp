#include <iostream>
#include "MyFreenectDevice.hpp"
#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
#include <visp/vpImageIo.h>
#include <visp/vpIoTools.h>
#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDisplayX.h>
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpParseArgv.h>


int main(int argc, const char ** argv){
    std::string opt_ipath;
    std::string configFile;
    std::string modelFile;
    std::string initFile;

    opt_ipath = "..";
    configFile = vpIoTools::path(opt_ipath) + vpIoTools::path("/configFiles/box.xml");
    modelFile = vpIoTools::path(opt_ipath) + vpIoTools::path("/configFiles/box.cao");
    initFile = vpIoTools::path(opt_ipath) + vpIoTools::path("/configFiles/box.init");
    

    vpImage<unsigned char>  I(480,640);
    vpDisplayX d(I);
    d.init(I, 100, 100, "3D model Tracking") ;
    Freenect::Freenect freenect;
    MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);
    device.startVideo();

    //Initialize the aquisition (just grabb some images from the kinect)
    for(int i=0;i<50;i++){
        device.getVideo(I);
        vpDisplay::display(I);
        vpDisplay::flush(I);
    }

    //*******************************************
    //*************TODO by STUDENTS**************
    //*******************************************
    vpHomogeneousMatrix cMo;

    // instantiation of md edge tracker
    vpMbEdgeTracker myedgetracker;

    // load the config file
    myedgetracker.loadConfigFile(configFile);

    // load the cao mode
    myedgetracker.loadModel(modelFile);


    // extraction of camera params
    vpCameraParameters mycamparams;
    myedgetracker.getCameraParameters(mycamparams);

    // initialize object pose
    myedgetracker.initClick(I, initFile);
    // get the initial pose
    myedgetracker.getPose(cMo);

    while (true){

        device.getVideo(I);
        vpDisplay::display(I);


        // display the model
        myedgetracker.display(I, cMo, mycamparams, vpColor::red);

        myedgetracker.track(I);

        //update pose of the tracker
        myedgetracker.getPose(cMo);

        myedgetracker.setDisplayFeatures(true);

        vpDisplay::flush(I);


        if (vpDisplay::getClick(I, false))
            break;
    }


    //*******************************************
    //*******************************************
    //*******************************************
    

    return 0;
}

