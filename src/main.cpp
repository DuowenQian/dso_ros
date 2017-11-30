/**
* This file is part of DSO.
* 
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/





#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "util/settings.h"
#include "FullSystem/FullSystem.h"
#include "util/Undistort.h"
#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"
#include "ROSOutputPublisher.h"


#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include "cv_bridge/cv_bridge.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


std::string calib = "";
std::string vignetteFile = "";
std::string gammaFile = "";
bool useSampleOutput=false;

using namespace dso;

void parseArgument(char* arg)
{
	int option;
	char buf[1000];

	if(1==sscanf(arg,"sampleoutput=%d",&option))
	{
		if(option==1)
		{
			useSampleOutput = true;
			printf("USING SAMPLE OUTPUT WRAPPER!\n");
		}
		return;
	}

	if(1==sscanf(arg,"quiet=%d",&option))
	{
		if(option==1)
		{
			setting_debugout_runquiet = true;
			printf("QUIET MODE, I'll shut up!\n");
		}
		return;
	}


	if(1==sscanf(arg,"nolog=%d",&option))
	{
		if(option==1)
		{
			setting_logStuff = false;
			printf("DISABLE LOGGING!\n");
		}
		return;
	}

	if(1==sscanf(arg,"nogui=%d",&option))
	{
		if(option==1)
		{
			disableAllDisplay = true;
			printf("NO GUI!\n");
		}
		return;
	}
	if(1==sscanf(arg,"nomt=%d",&option))
	{
		if(option==1)
		{
			multiThreading = false;
			printf("NO MultiThreading!\n");
		}
		return;
	}
	if(1==sscanf(arg,"calib=%s",buf))
	{
		calib = buf;
		printf("loading calibration from %s!\n", calib.c_str());
		return;
	}
	if(1==sscanf(arg,"vignette=%s",buf))
	{
		vignetteFile = buf;
		printf("loading vignette from %s!\n", vignetteFile.c_str());
		return;
	}

	if(1==sscanf(arg,"gamma=%s",buf))
	{
		gammaFile = buf;
		printf("loading gammaCalib from %s!\n", gammaFile.c_str());
		return;
	}

	printf("could not parse argument \"%s\"!!\n", arg);
}




FullSystem* fullSystem = 0;
Undistort* undistorter = 0;
int frameID = 0;

void vidCb(const sensor_msgs::ImageConstPtr msgRGB, const sensor_msgs::ImageConstPtr msgD)
{
	cv_bridge::CvImagePtr cv_ptrRGB = cv_bridge::toCvCopy(msgRGB, sensor_msgs::image_encodings::MONO8); //convert image to mono
	//toCvCopy creates a copy of the image data from the ROS message
	cv_bridge::CvImageConstPtr cv_ptrD = cv_bridge::toCvShare(msgD);
	// toCvShare will point the returned cv::Mat at the ROS message data, avoiding a copy.
	// not allowed to modify the returned CvImage
	assert(cv_ptrRGB->image.type() == CV_8U); // Mat::type returns the type of a matrix element.
	assert(cv_ptrRGB->image.channels() == 1); // Mat::channels returns number of matrix channels.
	//assert will call abort if the expression inside the brackets is false.
	cv::Mat depth_image = cv_ptrD->image;
	// TODO assert Depth image

	if(setting_fullResetRequested) // TODO write setting_fullResetRequested condition for when tracking is lost or when too much rotation.
	{
		std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
		delete fullSystem;
		for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();
		fullSystem = new FullSystem();
		fullSystem->linearizeOperation=false;
		fullSystem->outputWrapper = wraps;
	    if(undistorter->photometricUndist != 0)
	    	fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
		setting_fullResetRequested=false;
	}

	MinimalImageB minImg((int)cv_ptrRGB->image.cols, (int)cv_ptrRGB->image.rows,(unsigned char*)cv_ptrRGB->image.data); //see MinimalImage.h
	ImageAndExposure* undistImg = undistorter->undistort<unsigned char>(&minImg, 1,0, 1.0f); // undistort minimal img and store as ImageAndExposure object
	undistImg->timestamp=cv_ptrRGB->header.stamp.toSec(); // set timestamp
	fullSystem->addActiveFrame(undistImg, depth_image, frameID); //DO THE THING! see FullSystem.cpp
	// TODO addActiveFrame(undistImg, cv_ptrD, frameID);

	frameID++;
	delete undistImg;

}





int main( int argc, char** argv )
{
	ros::init(argc, argv, "dso_live"); //Initialize ROS node called "dso_live"


	// Parse input arguments
	for(int i=1; i<argc;i++) parseArgument(argv[i]);

	// Set Parameters
	setting_desiredImmatureDensity = 1000;
	setting_desiredPointDensity = 1200;
	setting_minFrames = 5;
	setting_maxFrames = 7;
	setting_maxOptIterations=4;
	setting_minOptIterations=1;
	setting_logStuff = false;
	setting_kfGlobalWeight = 1.3;


	printf("MODE WITH CALIBRATION, but without exposure times!\n");
	setting_photometricCalibration = 2;
	setting_affineOptModeA = 0;
	setting_affineOptModeB = 0;


	// Get undistorter
    undistorter = Undistort::getUndistorterForFile(calib, gammaFile, vignetteFile);

    setGlobalCalib(
            (int)undistorter->getSize()[0],
            (int)undistorter->getSize()[1],
            undistorter->getK().cast<float>());

    // Initialize FullSystem ...
    // This will initialize coarseDistanceMap, coarseTracker, coarseTracker_forNewKF, coarseInitializer, pixelSelector,
    // and ef (Energy Functional) amongst other things
    fullSystem = new FullSystem();
    fullSystem->linearizeOperation=false;

    ros::NodeHandle nh; //Start the Node

    if(!disableAllDisplay){
	    fullSystem->outputWrapper.push_back(new IOWrap::PangolinDSOViewer(
	    		 (int)undistorter->getSize()[0],
	    		 (int)undistorter->getSize()[1])); // create a new entry in outputWrapper for ROSOutputPublisher
    	fullSystem->outputWrapper.push_back(new IOWrap::ROSOutputPublisher(nh)); // create a new entry in outputWrapper for ROSOutputPublisher
    }


    if(useSampleOutput)
        fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());


    if(undistorter->photometricUndist != 0)
    	fullSystem->setGammaFunction(undistorter->photometricUndist->getG());

    // SUBSCRIBER

    //ros::Subscriber imgSub = nh.subscribe("/camera/rgb/image_mono", 1, &vidCb);

    // TEST
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_mono", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/sw_registered/image_rect_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub); //where sync_pol(10) is a constructor and 10 is queue_size
    sync.registerCallback(boost::bind(&vidCb,_1,_2));
    // TEST

    ros::spin();

    fullSystem->printResult("result1.txt");

    for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
    {
        ow->join();
        delete ow;
    }

    delete undistorter;
    delete fullSystem;

	return 0;
}

