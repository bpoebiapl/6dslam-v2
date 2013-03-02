#include "RGBDFrame.h"
#include "math.h"


//RGBDSegmentation * segmenter;

int frame_id_counter = 0;
RGBDFrame::RGBDFrame(){}

void RGBDFrame::init_pointcloud(IplImage* rgb_img, IplImage* depth_img){
	
	float d_scaleing			= input->calibration->ds/input->calibration->scale;
	float centerX				= input->calibration->cx;
	float centerY				= input->calibration->cy;
	float invFocalX				= 1.0f/input->calibration->fx;
    float invFocalY				= 1.0f/input->calibration->fy;
	char * rgb_data				= (char *)rgb_img->imageData;
	unsigned short * depth_data	= (unsigned short *)depth_img->imageData;
	
	xyz_						= PointCloudXYZ::Ptr (new PointCloudXYZ);
	xyz_->width    				= depth_img->width;
	xyz_->height   				= depth_img->height;
	xyz_->points.resize (xyz_->width * xyz_->height);
	int point_index = 0;
	for(int h = 0; h < height; h++)
	{
		for(int w = 0; w < width; w++)
		{
			int ind = 640*h+w;
	
			float tmp_z = float(depth_data[ind]) * d_scaleing;
			float tmp_x = 0;
			float tmp_y = 0;

			if(tmp_z > 0){
				tmp_x = (w - centerX) * tmp_z * invFocalX;
		       	tmp_y = (h - centerY) * tmp_z * invFocalY;
			}
			xyz_->points[point_index].x = tmp_x;
			xyz_->points[point_index].y = tmp_y;
			xyz_->points[point_index].z = tmp_z;
			point_index++;
		}
	}
}

void RGBDFrame::init_normals(){
	printf("start init normals\n");
	search_method_xyz_ 	= SearchMethod::Ptr (new SearchMethod);
	normals_ 			= SurfaceNormals::Ptr (new SurfaceNormals);
	
	PointCloudXYZ::Ptr tmp_cloud = PointCloudXYZ::Ptr (new PointCloudXYZ);
	printf("1:nr xyz: %i\n",xyz_->width*xyz_->height);
	
	const float depth_limit = 100000.0;
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud		(xyz_);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits	(0, depth_limit);
	pass.filter				(*tmp_cloud);
	printf("2:nr xyz: %i\n",tmp_cloud->width*tmp_cloud->height);
	
	const float voxel_grid_size = 0.02f;
	pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
	vox_grid.setInputCloud (tmp_cloud);
	vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>); 
	vox_grid.filter (*tempCloud);
	xyz_ = tempCloud; 

//	pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
//	ne.setMaxDepthChangeFactor(0.02f);
//	ne.setNormalSmoothingSize(10.0f);
//	ne.setInputCloud(xyz_);
//	ne.compute(*normals_);

	printf("3:nr xyz: %i\n",xyz_->width*xyz_->height);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
	norm_est.setInputCloud (xyz_);
	norm_est.setSearchMethod (search_method_xyz_);
	norm_est.setRadiusSearch (normal_radius_);
	norm_est.compute (*normals_);

	features_ = LocalFeatures::Ptr (new LocalFeatures);
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
	fpfh_est.setInputCloud (xyz_);
	fpfh_est.setInputNormals (normals_);
	fpfh_est.setSearchMethod (search_method_xyz_);
	fpfh_est.setRadiusSearch (feature_radius_);
//	fpfh_est.compute (*features_);
//	printf("nr features: %i\n",features_->width*features_->height);

}

void RGBDFrame::init_jointcloudnormals(IplImage* rgb_img, IplImage* depth_img){
	float d_scaleing	= input->calibration->ds/input->calibration->scale;
	float centerX		= input->calibration->cx;
	float centerY		= input->calibration->cy;
	float invFocalX	= 1.0f/input->calibration->fx;
    float invFocalY	= 1.0f/input->calibration->fy;
	
	char * rgb_data		= (char *)rgb_img->imageData;
	unsigned short * depth_data	= (unsigned short *)depth_img->imageData;
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud->width    = rgb_img->width;
	cloud->height   = rgb_img->height;
	//cloud.is_dense = false;
	cloud->points.resize (cloud->width * cloud->height);
	int point_index = 0;
	
	for(int h = 0; h < height; h++)
	{
		for(int w = 0; w < width; w++)
		{
			int ind = 640*h+w;
			int tmp_r = char(rgb_data[3*ind+2]);
			int tmp_g = char(rgb_data[3*ind+1]);
			int tmp_b = char(rgb_data[3*ind+0]);

			if(tmp_r < 0){tmp_r = 255+tmp_r;}
			if(tmp_g < 0){tmp_g = 255+tmp_g;}
			if(tmp_b < 0){tmp_b = 255+tmp_b;}
	
			float tmp_z = float(depth_data[ind]) * d_scaleing;
			float tmp_x = 0;
			float tmp_y = 0;

			if(tmp_z > 0){
				tmp_x = (w - centerX) * tmp_z * invFocalX;
		       	tmp_y = (h - centerY) * tmp_z * invFocalY;
			}
			cloud->points[point_index].x = tmp_x;
			cloud->points[point_index].y = tmp_y;
			cloud->points[point_index].z = tmp_z;
			cloud->points[point_index].r = float(tmp_r);
			cloud->points[point_index].g = float(tmp_g);
			cloud->points[point_index].b = float(tmp_b);
			
			point_index++;
		}
	}
	//if(calculate_pointcloud)	{xyz_ = cloud;}
	
		SurfaceNormals::Ptr norm 			= SurfaceNormals::Ptr (new SurfaceNormals);
		pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
		ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
		ne.setMaxDepthChangeFactor(0.02f);
		ne.setNormalSmoothingSize(10.0f);
		ne.setInputCloud(cloud);
		ne.compute(*norm);
	//if(calculate_normals)		{normals_ = norm;}
	
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr full_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	pcl::concatenateFields(*cloud,*norm,*full_cloud);
	this->cloud = full_cloud;
}

void RGBDFrame::init_segmentation(IplImage* rgb_img, IplImage* depth_img){
	/*
	segments = segmenter->segment(rgb_img,depth_img);
	
	segmentation = new char*[width];
	for(int i = 0; i < width; i++){
		segmentation[i] = new char[height];
		for(int j = 0; j < height; j++){
			segmentation[i][j] = 0;
		}
	}
	
	for(int i = 0; i < width; i++){
		for(int j = 0; j < height; j++){
			segmentation[i][j] = 0;
		}
	}
	
	for(int i = 0; i < segments->size(); i++){
		segmentation_fit * seg = segments->at(i);
		vector< int > * seg_w = seg->seg_w;
		vector< int > * seg_h = seg->seg_h;
		//printf("seg_id:%i, seg_w->size() %i\n",i,seg_w->size());
		for(int j = 0; j < seg_w->size(); j++){
			unsigned int w = seg_w->at(j);
			unsigned int h = seg_h->at(j);
			segmentation[w][h] = char(i+1);
		}
	}
	*/
}

void RGBDFrame::init_filter(){
	float filter_distance				= 10000;
	PointCloudXYZ::Ptr tmp_cloud = PointCloudXYZ::Ptr (new PointCloudXYZ);
	//printf("1:nr xyz: %i\n",xyz_->width*xyz_->height);
	
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud		(xyz_);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits	(0.00001, filter_distance);
	pass.filter				(*tmp_cloud);
	//printf("2:nr xyz: %i\n",tmp_cloud->width*tmp_cloud->height);
	xyz_ = tmp_cloud;
	
	const float voxel_grid_size = 0.05f;//12f;
	pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
	vox_grid.setInputCloud (tmp_cloud);
	vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>); 
	vox_grid.filter (*tempCloud);
	xyz_ = tempCloud;
	//printf("3:nr xyz: %i\n",tempCloud->width*tempCloud->height);
}

RGBDFrame::RGBDFrame(Frame_input * fi, FeatureExtractor * extractor, RGBDSegmentation * segmenter)
{
	struct timeval start, end;
	gettimeofday(&start, NULL);
	
	bool calculate_pointcloud 			= true;
	bool calculate_normals 				= false;
	bool calculate_jointcloudnormals	= false;
	bool calculate_segmentation 		= false;

	bool filter_pointcloud 				= true;


	id = frame_id_counter++;
	//printf("id:%i\n",id);
	input = fi;
	IplImage* rgb_img 	= cvLoadImage(fi->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
	IplImage* depth_img = cvLoadImage(fi->depth_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
	
	//printf("rgb: %s\n",fi->rgb_path.c_str());
	//printf("depth: %s\n",fi->depth_path.c_str());

	
	width = rgb_img->width;
	height = rgb_img->height;
	
	keypoints = extractor->getKeyPointSet(rgb_img,depth_img);
	
	vector<FeatureDescriptor * > words = fi->calibration->words;
	float * bow = new float[words.size()];
	for(unsigned int j = 0; j < words.size();j++){bow[j]=0;}
	for(unsigned int i = 0; i < keypoints->valid_key_points.size();i++){
		int best_id = 0;
		float best = 10000000000;
		KeyPoint * kp = keypoints->valid_key_points.at(i);
		for(unsigned int j = 0; j < words.size();j++){
			float d = kp->descriptor->distance(words.at(j));
			//printf("kp...\n");
			//kp->descriptor->print();
			//printf("words...\n");
			//words.at(j)->print();
			//printf("%i %i -> %f\n",i,j,d);
			kp->cluster_distances.push_back(d);
			if(d < 0.25){
				kp->cluster_distance_pairs.push_back(make_pair(j,d));
			}
			if(d < best){best = d; best_id = j;}
		}
		bow[best_id]++;
		kp->sortDistances();
		//kp->print();
	}
	for(unsigned int i = 0; i < keypoints->invalid_key_points.size();i++){
		int best_id = 0;
		float best = 10000000000;
		KeyPoint * kp = keypoints->invalid_key_points.at(i);
		for(unsigned int j = 0; j < words.size();j++){
			float d = kp->descriptor->distance(words.at(j));
			//kp->cluster_distances.push_back(d);
			//kp->cluster_distance_pairs.push_back(make_pair(j,d));
			if(d < best){best = d; best_id = j;}
		}
		bow[best_id]++;
		kp->sortDistances();
		delete kp;
		//kp->print();
	}

	float bow_div = float(keypoints->invalid_key_points.size()+keypoints->valid_key_points.size());
	for(unsigned int j = 0; j < words.size();j++){bow[j]/=bow_div;}
	
	image_descriptor = new FloatHistogramFeatureDescriptor(bow,words.size());
	
	planes = segmenter->segment(rgb_img,depth_img);

	normal_radius_	= 0.15f;
	feature_radius_	= 0.15f;

	if(calculate_pointcloud)			{init_pointcloud(rgb_img,depth_img);}
	if(calculate_segmentation)			{init_segmentation(rgb_img,depth_img);}
	if(calculate_normals)				{init_normals();}
	if(calculate_jointcloudnormals)		{init_jointcloudnormals(rgb_img,depth_img);}
	if(filter_pointcloud)				{init_filter();}
	
	float d_scaleing	= input->calibration->ds/input->calibration->scale;
	float centerX		= input->calibration->cx;
	float centerY		= input->calibration->cy;
	float invFocalX	= 1.0f/input->calibration->fx;
    float invFocalY	= 1.0f/input->calibration->fy;
    
	unsigned short * depth_data	= (unsigned short *)depth_img->imageData;
	int step = 5;
	for(int w = step; w < width; w+=step){
		for(int h = step; h < height; h+=step){
			
			int ind = 640*h+w;
			float x = 0;
			float y = 0;
			float z = float(depth_data[ind]) * d_scaleing;

			if(z > 0){
				x = (w - centerX) * z * invFocalX;
		       	y = (h - centerY) * z * invFocalY;
		       	float * vp = new float[3];
		       	vp[0] = x;
		       	vp[1] = y;
		       	vp[2] = z;
		       	validation_points.push_back(vp);
		       	//printf("%i %i -> %f %f %f\n",w,h,x,y,z);
			}
		}
	}
	
	cvReleaseImage( &rgb_img );
	cvReleaseImage( &depth_img );
	
	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	//printf("RGBDFrame total cost: %f\n",time);
}


RGBDFrame::RGBDFrame(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_cloud, vector<FeatureDescriptor *> * centers)
{	
	id = frame_id_counter++;
	cloud						= input_cloud;
	width 						= input_cloud->width;
	height 						= input_cloud->height;
	
	FeatureExtractor * fe = new SurfExtractor();
	keypoints = fe->getKeyPointSet(input_cloud);
	delete fe;
	/*
	for(unsigned int i = 0; i < keypoints->valid_key_points.size();i++){
		KeyPoint * current = keypoints->valid_key_points.at(i);
		for(unsigned int j = 0; j < centers->size(); j++)
		{
			current->cluster_distances.push_back(current->descriptor->distance(centers->at(j)));
		}
	}
	
	for(unsigned int i = 0; i < keypoints->invalid_key_points.size();i++){
		KeyPoint * current = keypoints->invalid_key_points.at(i);
		for(unsigned int j = 0; j < centers->size(); j++)
		{
			current->cluster_distances.push_back(current->descriptor->distance(centers->at(j)));
		}
	}
	*/
	//delete kps;
	//planes = segment(input_cloud);

	
	int input_valid = 0;
	for(unsigned int i = 0; i < input_cloud->width*input_cloud->height; i++){
		if(input_cloud->points[i].z > 0){input_valid++;}
	}
	//printf("input_valid: %i\n",input_valid);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_cloud_clean (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	input_cloud_clean->width    = input_valid;
	input_cloud_clean->height   = 1;
	input_cloud_clean->is_dense = false;
	input_cloud_clean->points.resize (input_valid);
	
	input_valid = 0;
	for(unsigned int i = 0; i < input_cloud->width*input_cloud->height; i++){
		if(input_cloud->points[i].z > 0){
			//input_cloud_clean->points[input_valid] = input_cloud->points[i];
			input_cloud_clean->points[input_valid].x = input_cloud->points[i].x;
			input_cloud_clean->points[input_valid].y = input_cloud->points[i].y;
			input_cloud_clean->points[input_valid].z = input_cloud->points[i].z;
			input_cloud_clean->points[input_valid].r = input_cloud->points[i].r;
			input_cloud_clean->points[input_valid].g = input_cloud->points[i].g;
			input_cloud_clean->points[input_valid].b = input_cloud->points[i].b;
			input_cloud_clean->points[input_valid].normal_x = input_cloud->points[i].normal_x;
			input_cloud_clean->points[input_valid].normal_y = input_cloud->points[i].normal_y;
			input_cloud_clean->points[input_valid].normal_z = input_cloud->points[i].normal_z;
			input_valid++;
		}
	}
	
	//cloud = input_cloud;
	cloud = input_cloud_clean;
}

RGBDFrame::~RGBDFrame(){printf("~RGBDFrame()\n");}

void RGBDFrame::showPlanes(){
	IplImage* rgb_img 	= cvLoadImage(input->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
	IplImage* depth_img = cvLoadImage(input->depth_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
	
	float d_scaleing	= input->calibration->ds/input->calibration->scale;
	float centerX		= input->calibration->cx;
	float centerY		= input->calibration->cy;
	float invFocalX		= 1.0f/input->calibration->fx;
    float invFocalY		= 1.0f/input->calibration->fy;
    
	unsigned short * depth_data	= (unsigned short *)depth_img->imageData;
	char * rgb_data		= (char *)rgb_img->imageData;
	int step = 5;
	for(int w = step; w < width; w+=1){
		for(int h = step; h < height; h+=1){
			
			int ind = 640*h+w;
			float x = 0;
			float y = 0;
			float z = float(depth_data[ind]) * d_scaleing;
		   	int best_i;
			float best = 1000;
			if(z > 0){
				x = (w - centerX) * z * invFocalX;
		       	y = (h - centerY) * z * invFocalY;

				for(int i = 0; i < planes->size(); i++){
					float d = fabs(planes->at(i)->distance(x,y,z));
					if(d < best){
						best = d;
						best_i = i;
					}
				}
				//printf("%i %i %f\n",w,h,best);
				if(best < 0.01){
					if(best_i == 0){
						rgb_data[3*ind+0] = 0;
						rgb_data[3*ind+1] = 0;
						rgb_data[3*ind+2] = 0;
					}else if(best_i == 1){
						rgb_data[3*ind+0] = 255;
						rgb_data[3*ind+1] = 0;
						rgb_data[3*ind+2] = 0;
					}else if(best_i == 2){
						rgb_data[3*ind+0] = 0;
						rgb_data[3*ind+1] = 0;
						rgb_data[3*ind+2] = 255;
					}else if(best_i == 3){
						rgb_data[3*ind+0] = 0;
						rgb_data[3*ind+1] = 255;
						rgb_data[3*ind+2] = 0;
					}else if(best_i == 4){
						rgb_data[3*ind+0] = 255;
						rgb_data[3*ind+1] = 0;
						rgb_data[3*ind+2] = 255;
					}else if(best_i == 5){
						rgb_data[3*ind+0] = 0;
						rgb_data[3*ind+1] = 255;
						rgb_data[3*ind+2] = 255;
					}else if(best_i == 6){
						rgb_data[3*ind+0] = 0;
						rgb_data[3*ind+1] = 255;
						rgb_data[3*ind+2] = 255;
					}else if(best_i == 7){
						rgb_data[3*ind+0] = 255;
						rgb_data[3*ind+1] = 255;
						rgb_data[3*ind+2] = 255;
					}
				}
			}
		}
	}
	
	
	cvNamedWindow("frame planes", CV_WINDOW_AUTOSIZE );
	cvShowImage("frame planes", rgb_img);
	cvWaitKey(0);
	cvReleaseImage( &rgb_img );
	cvReleaseImage( &depth_img );
}
