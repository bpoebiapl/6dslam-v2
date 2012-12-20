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
	
	const float voxel_grid_size = 0.12f;
	pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
	vox_grid.setInputCloud (tmp_cloud);
	vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>); 
	vox_grid.filter (*tempCloud);
	xyz_ = tempCloud;
	//printf("3:nr xyz: %i\n",tempCloud->width*tempCloud->height);
}

RGBDFrame::RGBDFrame(Frame_input * fi, FeatureExtractor * extractor)
{

	bool calculate_pointcloud 			= true;
	bool calculate_normals 				= false;
	bool calculate_jointcloudnormals	= false;
	bool calculate_segmentation 		= false;

	bool filter_pointcloud 				= true;


	id = frame_id_counter++;
	input = fi;
	IplImage* rgb_img 	= cvLoadImage(fi->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
	IplImage* depth_img = cvLoadImage(fi->depth_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
	width = rgb_img->width;
	height = rgb_img->height;
	keypoints = extractor->getKeyPointSet(rgb_img,depth_img);
	normal_radius_	= 0.15f;
	feature_radius_	= 0.15f;
	//printf("checking distances\n");
/*	
	struct timeval start, end;
	gettimeofday(&start, NULL);
	
	printf("keypoints->valid_key_points.size() = %i\n",keypoints->valid_key_points.size());
	printf("keypoints->invalid_key_points.size() = %i\n",keypoints->invalid_key_points.size());
	
	double ** fd_mat = new double*[keypoints->valid_key_points.size()];
	double ** pd_mat = new double*[keypoints->valid_key_points.size()];
	double ** overlap_mat = new double*[keypoints->valid_key_points.size()];
	
	bool * taken = new bool[keypoints->valid_key_points.size()];
	
	for(unsigned int i = 0; i < keypoints->valid_key_points.size();i++){
		fd_mat[i] = new double[keypoints->valid_key_points.size()];
		pd_mat[i] = new double[keypoints->valid_key_points.size()];
		overlap_mat[i] = new double[keypoints->valid_key_points.size()];
		taken[i] = false;
	}
	for(unsigned int i = 0; i < keypoints->valid_key_points.size();i++){
		KeyPoint * current = keypoints->valid_key_points.at(i);
		fd_mat[i][i] = 0;
		pd_mat[i][i] = 1;
		for(unsigned int j = i+1; j < keypoints->valid_key_points.size(); j++)
		{
			KeyPoint * current2 = keypoints->valid_key_points.at(j);
			float fd = sqrt(current->descriptor->distance(current2->descriptor));
			float x_delta = current->point->x - current2->point->x;
			float y_delta = current->point->y - current2->point->y;
			float z_delta = current->point->z - current2->point->z;
			float pd = sqrt(x_delta*x_delta + y_delta*y_delta + z_delta*z_delta);
			fd_mat[i][j] = fd;
			fd_mat[j][i] = fd;
			
			pd_mat[i][j] = pd;
			pd_mat[j][i] = pd;
			overlap_mat[i][j] = -1;
			if(pd < 0.05){
				int w1 = current->point->w;
				int h1 = current->point->h;
				int w2 = current2->point->w;
				int h2 = current2->point->h;
				
				float steps = 10;
				float mw = float(w1 - w2)/steps;
				float mh = float(h1 - h2)/steps;
				
				for(int k = 0; k < int(steps); k++){
					
				}
				printf("%i %i <->%i %i -> %f\n",w1,h1,w2,h2,overlap_mat[i][j]);
			}
		}
	}
	
	vector<vector<int> * > * components = new vector<vector<int> * >();
	for(unsigned int i = 0; i < keypoints->valid_key_points.size();i++){
		if(!taken[i]){
			taken[i] = true;
			vector<int> * component = new vector<int>();
			components->push_back(component);
			vector<int> todolist;
			todolist.push_back(i);
			while(todolist.size() > 0)
			{
				int ii = todolist.back();
				todolist.pop_back();
				component->push_back(ii);
				for(unsigned int jj = 0; jj < keypoints->valid_key_points.size();jj++){
					if(!taken[jj] && pd_mat[ii][jj] < 0.05){
						taken[jj] = true;
						todolist.push_back(jj);
					}
				}
			}
		}
	}
	IplImage* rgb_img_clone 	= cvLoadImage(fi->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
	printf("nr components = %i\n",components->size());
	for(int i = 0; i < components->size(); i++)
	{
		printf("%i: component size = %i\n",i,components->at(i)->size());
		int r = rand()%256;
		int g = rand()%256;
		int b = rand()%256;
		for(int j = 0; j < components->at(i)->size(); j++)
		{
			KeyPoint * kp = keypoints->valid_key_points.at(components->at(i)->at(j));
			printf("%i:%i -> %i\n",i,j,components->at(i)->at(j));
			cvCircle(rgb_img_clone, cvPoint(kp->point->w, kp->point->h), 3, cvScalar(b, g, r, 0), 1, 8, 0);
		}
	}
	
	for(unsigned int i = 0; i < keypoints->valid_key_points.size();i++){
		delete[] fd_mat[i];
		delete[] pd_mat[i];
		delete[] overlap_mat[i];
	}
	delete[] taken;
	delete[] fd_mat;
	delete[] pd_mat;
	delete[] overlap_mat;
	
	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	printf("d time: %f\n",time);

	cvShowImage("rgb_img_clone", rgb_img_clone);
	cvWaitKey(0);
	cvReleaseImage( &rgb_img_clone);
*/

	if(calculate_pointcloud)			{init_pointcloud(rgb_img,depth_img);}
	if(calculate_segmentation)			{init_segmentation(rgb_img,depth_img);}
	if(calculate_normals)				{init_normals();}
	if(calculate_jointcloudnormals)		{init_jointcloudnormals(rgb_img,depth_img);}
	if(filter_pointcloud)				{init_filter();}
	cvReleaseImage( &rgb_img );
	cvReleaseImage( &depth_img );	
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
