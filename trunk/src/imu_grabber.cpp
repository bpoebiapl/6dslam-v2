#include <time.h>
#include <sys/types.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <phidget21.h>

CPhidgetSpatialHandle spatial;

//callback that will run if the Spatial generates an error
int CCONV ErrorHandler(CPhidgetHandle spatial, void *userptr, int ErrorCode, const char *unknown){
	printf("IMU:Error handled. %d - %s \n", ErrorCode, unknown);
	return 0;
}

int CCONV SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count){
	struct timeval start, end;
	gettimeofday(&start, NULL);
	
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
		gettimeofday(&end, NULL);
		float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;

		printf("%i,%i -> acc: %f %f %f gyro: %f %f %f comp: %f %f %f\n",end.tv_sec , end.tv_usec ,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,comp_x,comp_y,comp_z);
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


int main(int argc, char **argv){
	printf("START\n");
	connect();
	while (true){usleep(1000);}
	dissconnect();
	printf("DONE\n");
	return 0;
}
