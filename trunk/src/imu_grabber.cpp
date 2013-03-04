#include <time.h>
#include <sys/types.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <phidget21.h>

#include "cv.h"
#include "highgui.h"
#include <opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>

struct timeval start;

double getTime(){
	struct timeval end;
	gettimeofday(&end, NULL);
	double time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	return time;
}
CPhidgetSpatialHandle spatial;

//callback that will run if the Spatial generates an error
int CCONV ErrorHandler(CPhidgetHandle spatial, void *userptr, int ErrorCode, const char *unknown){
	printf("IMU:Error handled. %d - %s \n", ErrorCode, unknown);
	return 0;
}

int CCONV SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count){
	//struct timeval start, end;
	//gettimeofday(&start, NULL);
	
	for(int i = 0; i < count; i++){
		float acc_x = data[i]->acceleration[0];
		float acc_y = data[i]->acceleration[1];
		float acc_z = data[i]->acceleration[2];
		float gyro_x = data[i]->angularRate[0];
		float gyro_y = data[i]->angularRate[1];
		float gyro_z = data[i]->angularRate[2];
		float comp_x = data[i]->magneticField[0];
		float comp_y = data[i]->magneticField[1];
		float comp_z = data[i]->magneticField[2];
		struct timeval end;
		gettimeofday(&end, NULL);
		float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
		//printf("time: %f\n",time);
		printf("%3.10f %3.10f %3.10f %3.10f %3.10f %3.10f %3.10f %3.10f %3.10f %3.10f\n",time,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,comp_x,comp_y,comp_z);
	}
	return 0;
}

void connect(){
	spatial = 0;
	int result;
	const char *err;
	CPhidgetSpatial_create(&spatial);

	CPhidget_set_OnError_Handler((CPhidgetHandle)spatial, ErrorHandler, NULL);
	CPhidgetSpatial_set_OnSpatialData_Handler(spatial, SpatialDataHandler, NULL);

	//open the spatial object for device connections
	CPhidget_open((CPhidgetHandle)spatial, -1);

	//get the program to wait for a spatial device to be attached
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)spatial, 10000))){
		CPhidget_getErrorDescription(result, &err);
		printf("IMU:Problem waiting for attachment: %s\n", err);
		exit(0);
	}
	CPhidgetSpatial_setDataRate(spatial, 1);
}

void dissconnect(){
	CPhidget_close((CPhidgetHandle)spatial);
	CPhidget_delete((CPhidgetHandle)spatial);
}

int total = 0;

class SimpleOpenNIProcessor
{
public:
  void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
  {
    static unsigned count = 0;
    static double last = pcl::getTime ();
    if (++count == 1)
    {
      double now = pcl::getTime ();
      //std::cout << "distance of center pixel :" << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z << " mm. Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
      count = 0;
      last = now;
    }
	
    total++;
    IplImage * rgb_img		= cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
    char * rgb_data 		= (char *)(rgb_img->imageData);
    
    IplImage * depth_img	= cvCreateImage(cvSize(640, 480), IPL_DEPTH_16U, 1);
	//float * depth_data		= (float *)depth_img->imageData;
    unsigned short * depth_data		= (unsigned short *)depth_img->imageData;
    for(int i = 0; i < 640; i++){
		for(int j = 0; j < 480; j++){
			int ind = 640*j+i;
			int r = cloud->points[ind].r;
			int g = cloud->points[ind].g;
			int b = cloud->points[ind].b;
			rgb_data[3*ind+0] = b;
			rgb_data[3*ind+1] = g;
			rgb_data[3*ind+2] = r;
			//depth_data[3*ind+0]	= cloud->points[ind].x;
			//depth_data[3*ind+1]	= cloud->points[ind].y;
			//depth_data[3*ind+2]	= cloud->points[ind].z;
			//printf("%i %i %i\n",i,j,(unsigned short)(1000*cloud->points[ind].z));
			depth_data[ind]	= (unsigned short)(1000.0f*cloud->points[ind].z);
		}
	}
    
    cvNamedWindow("rgb", CV_WINDOW_AUTOSIZE );
	cvShowImage("rgb", rgb_img);
	
	cvNamedWindow("depth", CV_WINDOW_AUTOSIZE );
	cvShowImage("depth", depth_img);
	
	cvWaitKey(50);
	double currenttime = getTime();
	char buffer[250];
	sprintf (buffer, "outputimgs/rgb_%f.jpg", currenttime);
	//printf ("%s\n",buffer);
	cvSaveImage(buffer,rgb_img); 
	
	sprintf (buffer, "outputimgs/depth_%f.jpg", currenttime);
	//printf ("%s\n",buffer);
	cvSaveImage(buffer,depth_img);
	
	cvReleaseImage( &rgb_img );
	cvReleaseImage( &depth_img );
  }
  
  void run ()
  {
    // create a new grabber for OpenNI devices
    pcl::Grabber* interface = new pcl::OpenNIGrabber();

    // make callback function from member function
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
      boost::bind (&SimpleOpenNIProcessor::cloud_cb_, this, _1);

    // connect callback function for desired signal. In this case its a point cloud with color values
    boost::signals2::connection c = interface->registerCallback (f);

    // start receiving point clouds
    interface->start ();

    // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
    while (true)
      boost::this_thread::sleep (boost::posix_time::seconds (1));

    // stop the grabber
    interface->stop ();
  }
};

int main ()
{
	gettimeofday(&start, NULL);
	connect();
  SimpleOpenNIProcessor v;
  v.run ();
  return (0);
}


//int main(int argc, char **argv){
//	printf("START\n");
	
	//connect();
	//while (true){usleep(1000);}
	//dissconnect();
	
	
	
//	printf("DONE\n");
//	return 0;
//}
