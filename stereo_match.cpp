#include"Rec3DTest.h"

 int point_num= 0;

int input_points(vector<Point3f> &points){
  int c = 0;
  ofstream ofile;
  ofile.open("input.txt",ios::app);
  for(int i = 0; i < points.size(); i++){
    ofile<<points[i].x<<" "<< points[i].y<<" "<<points[i].z<<"\n";
  }
  ofile.close();
  return 0;
}

string getPlyheader(){

  stringstream num;
  num << point_num;
  string ply_header( "ply\n");
  ply_header += "format ascii 1.0\n";
  ply_header += "element vertex " + num.str() + "\n";
  ply_header += "property float x\n";
  ply_header += "property float y\n";
  ply_header += "property float z\n";
//  ply_header += "property uchar red\n";
//  ply_header += "property uchar green\n";
//  ply_header += "property uchar blue\n";
  ply_header += "end_header\n";

  return ply_header;
}

#if 0
static void saveXYZ(const char* filename, vector<Point3f> &points){

	
    point_num += points.size();
     input_points(points);
    ofstream ofile;
    ofile.open("out.ply",ios::ate | ios::trunc);
    ifstream is("input.txt");
    ofile<<getPlyheader();

    string line;

    while( getline(is, line) ){
      ofile<<line<<"\n";
    }

    is.close();
    ofile.close();
    system("rm -rf ./input.txt");

}

#else 


static void saveXYZ(const char* filename, const Mat& mat)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");	
    point_num += 157780;	
    fprintf(fp, "%s", "ply\n");
    fprintf(fp, "%s", "format ascii 1.0\n");
    fprintf(fp, "%s %d %s" , "element vertex",point_num,"\n");
    fprintf(fp, "%s", "property float x\n");
    fprintf(fp, "%s", "property float y\n");
    fprintf(fp, "%s", "property float z\n");
    fprintf(fp, "%s", "end_header\n");
	
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
        }
    }
    fclose(fp);
}

#endif


int main(int argc, char** argv)
{

    const char* intrinsic_filename = 0;
    const char* extrinsic_filename = 0;
    const char* disparity_filename = 0;
    const char* point_cloud_filename = 0;

//    double* pimg = 0;
	
    intrinsic_filename = "intrinsics.yml";
    extrinsic_filename = "extrinsics.yml";
    disparity_filename = "disparity_filename.txt";
    point_cloud_filename = "out.ply";
	Mat_<double> ds;
	
	
	

    enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3 };
    int alg = STEREO_SGBM;
    int SADWindowSize = 0, numberOfDisparities = 0;
    bool no_display = false;
    float scale = 1.f;

    Ptr<StereoBM> bm = StereoBM::create(16,9);
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);
                  	        
    int color_mode = alg ;
	
    Mat img1 = imread("/home/ply/leftout.jpg", 1);
    Mat img2 = imread("/home/ply/rightout.jpg", 1);

    int h = img1.rows;
    int w = img1.cols;
    double f = 0.8*w;

    if (img1.empty() ||img2.empty())
    {
        printf("Command-line parameter error: could not load the first input image file\n");
        return -1;
    }
 
    Size img_size = img1.size();

    Rect roi1, roi2;
   
	Mat Q;
   
	
    if( intrinsic_filename )
    {
        // reading intrinsic parameters

	
        FileStorage fs(intrinsic_filename, FileStorage::READ);
        if(!fs.isOpened())
        {
            printf("Failed to open file %s\n", intrinsic_filename);
            return -1;
        }

        Mat M1, D1, M2, D2;
        fs["M1"] >> M1;
        fs["D1"] >> D1;
        fs["M2"] >> M2;
        fs["D2"] >> D2;

#if 0
	cout << "M1:"<<M1<<endl;
	cout << "D1:"<<D1<<endl;
	cout << "M2:"<<M2<<endl;
	cout << "D2:"<<D2<<endl;

#endif

        M1 *= scale;
        M2 *= scale;

        fs.open(extrinsic_filename, FileStorage::READ);
        if(!fs.isOpened())
        {
            printf("Failed to open file %s\n", extrinsic_filename);
            return -1;
        }

        Mat R, T, R1, P1, R2, P2;
        fs["R"] >> R;
        fs["T"] >> T;

	cout << "R:"<<R<<endl;
	cout << "T:"<<T<<endl;


        stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 1, img_size, &roi1, &roi2 );

		 cout << " Q:"<<Q<<endl;

#if 0 
	cout << "R1:"<<R1<<endl;
	cout << "R2:"<<R2<<endl;
	cout << "P1:"<<P1<<endl;
	cout << "P2:"<<P2<<endl;
	cout << "Q:"<<Q<<endl;
	
#endif

	 ds = Q;		
		
        Mat map11, map12, map21, map22;
        initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

        Mat img1r, img2r;
        remap(img1, img1r, map11, map12, INTER_LINEAR);
        remap(img2, img2r, map21, map22, INTER_LINEAR);

//        img1 = img1r;
  //      img2 = img2r;
    }

	

    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;

    cout << "numberOfDisparities:"<<numberOfDisparities <<endl;

    bm->setROI1(roi1);
    bm->setROI2(roi2);
    bm->setPreFilterCap(31);
    bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
    bm->setMinDisparity(0);
    bm->setNumDisparities(numberOfDisparities);
    bm->setTextureThreshold(10);
    bm->setUniquenessRatio(15);
    bm->setSpeckleWindowSize(100);
    bm->setSpeckleRange(32);
    bm->setDisp12MaxDiff(1);

    sgbm->setPreFilterCap(63);
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);

    int cn = img1.channels();
	cout << numberOfDisparities<<endl;
    cout <<"cn :"<< cn << endl;
    cn = 3;

    sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(16);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setMode(alg == STEREO_HH ? StereoSGBM::MODE_HH : StereoSGBM::MODE_SGBM);

    Mat disp, disp8;
    // Boundary extension  to  get  disp area the same with image
    //Mat img1p, img2p, dispp;
    //copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
    //copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

    int64 t = getTickCount();
	alg == STEREO_SGBM;
    if( alg == STEREO_BM )
        bm->compute(img1, img2 );
    else if( alg == STEREO_SGBM || alg == STEREO_HH )
        sgbm->compute(img1, img2, disp);

    t = getTickCount() - t;
    printf("Time elapsed: %fms\n", t*1000/getTickFrequency());


	// cut disp to the same size of  origin image
    //disp = dispp.colRange(numberOfDisparities, img1p.cols);
    if( alg != STEREO_VAR )
        disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
    else
        disp.convertTo(disp8, CV_8U);
    if( !no_display )
    {     
    	cout << " no_display : "<<no_display<<endl;
        imwrite("left01.jpg", img1);     
        imwrite("right01.jpg", img2);
        imwrite("disparity.jpg", disp8);
	printf("\n");
    }

    cout<<"disparity_filename"<<endl;

    Mat xyz;
    vector<Point3f>points;
    if(point_cloud_filename)
    {
        printf("storing the point cloud...");
        fflush(stdout);
  //     Mat Q = (Mat_<double>(4,4)<<1, 0, 0, -0.5*w , 0,-1, 0,  0.5*h, 0, 0, 0,-f , 0, 0, 1,0);
       cout << " Q:"<<Q<<endl;

        reprojectImageTo3D(disp8, xyz, Q,true);
/*
	for (int i = 0; i < img1.rows; i++)
        {
        	
	        uchar* rgb_ptr = img1.ptr<uchar>(i);
	        uchar* disp_ptr = disp.ptr<uchar>(i);
			
	        for (int j = 0; j < img1.cols; j++)
	        {
	            //Get 3D coordinates	         
	            uchar d = disp_ptr[j];
	
	            if ( d == 0 ) continue; //Discard bad pixels
		   				
	            double pw = -1.0 * static_cast<double>(d) * ds(3,2) + ds(3,3);
	            double px = static_cast<double>(j) + ds(0,3);
	            double py = static_cast<double>(i) + ds(1,3);
	            double pz = ds(2,3);
	            
	            px = px/pw;
	            py = py/pw;
	            pz = pz/pw;
			
//		    points= (Mat_<double>(1, 3)<<px, py, pz);

		    points.push_back(Point3f(px, py, pz));
			    		 			 			
	          }
	}
	cout <<"points num is: "<<points.size()<<endl;
		
        saveXYZ(point_cloud_filename, points);
        */
        saveXYZ(point_cloud_filename, xyz);
        printf("\n");
    }



    return 0;
}
