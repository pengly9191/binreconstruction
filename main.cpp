#include"Rec3DTest.h"

const float inlier_threshold = 105.5f; // Distance threshold to identify inliers
const float nn_match_ratio = 105.8f;   // Nearest neighbor matching ratio


static void saveXYZ(const char* filename, const Mat& mat)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");	
    int point_num = 157780;	
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





int main(void)
{

	Mat img1 = imread("/home/ply/image/1.jpg", IMREAD_GRAYSCALE);
	Mat img2 = imread("/home/ply/image/2.jpg", IMREAD_GRAYSCALE);

	if( img1.empty() || img2.empty())
	  cout<<"image err..."<<endl;

	 Size  imageSize = img1.size();
	   
	Mat homography;
	FileStorage fs("H1to3p.xml", FileStorage::READ);
	fs.getFirstTopLevelNode() >> homography;

	vector<KeyPoint> kpts1, kpts2;
	Mat desc1, desc2;

	Ptr<AKAZE> akaze = AKAZE::create();

	akaze->detectAndCompute(img1, noArray(), kpts1, desc1);
	akaze->detectAndCompute(img2, noArray(), kpts2, desc2);

	Mat outimg1,outimg2 ;
	drawKeypoints(img1,kpts1,outimg1);	

	imwrite("outimg1.png", outimg1);

	// delete error match
	BFMatcher matcher(NORM_HAMMING);
	vector< vector<DMatch> > nn_matches;
	matcher.knnMatch(desc1, desc2, nn_matches, 2);
		
	vector<KeyPoint> matched1, matched2, inliers1, inliers2;
	vector<DMatch> good_matches;


	for(size_t i = 0; i < nn_matches.size(); i++) {
	    DMatch first = nn_matches[i][0];
		
	    float dist1 = nn_matches[i][0].distance;
	    float dist2 = nn_matches[i][1].distance;

	    if(dist1 < nn_match_ratio * dist2) {
	        matched1.push_back(kpts1[first.queryIdx]);
	        matched2.push_back(kpts2[first.trainIdx]);
	    
	    }
	}

	for(unsigned i = 0; i < matched1.size(); i++) {

	    Mat col = Mat::ones(3, 1, CV_64F);
	    col.at<double>(0) = matched1[i].pt.x;
	    col.at<double>(1) = matched1[i].pt.y;

	    col = homography * col;
	    col /= col.at<double>(2);
	    double dist = sqrt( pow(col.at<double>(0) - matched2[i].pt.x, 2) +
	                        pow(col.at<double>(1) - matched2[i].pt.y, 2));

	    if(dist < inlier_threshold) {
	        int new_i = static_cast<int>(inliers1.size());
	        inliers1.push_back(matched1[i]);
	        inliers2.push_back(matched2[i]);
	        good_matches.push_back(DMatch(new_i, new_i, 0));
	    }
	}

	Mat res;
	drawMatches(img1, inliers1, img2, inliers2, good_matches, res);

	imwrite("res1.png", res);

	double inlier_ratio = inliers1.size() * 1.0 / matched1.size();
	cout << "A-KAZE Matching Results" << endl;
	cout << "*******************************" << endl;
	cout << "# Keypoints 1:                        \t" << kpts1.size() << endl;
	cout << "# Keypoints 2:                        \t" << kpts2.size() << endl;
	cout << "# Matches:                            \t" << matched1.size() << endl;
	cout << "# Inliers:                            \t" << inliers1.size() << endl;
	cout << "# Inliers Ratio:                      \t" << inlier_ratio << endl;
	cout << endl;


	vector<Point2f>P1,P2;
	for (int i = 0; i< matched1.size() ; i++){
		P1.push_back(Point2f(matched1[i].pt.x,matched1[i].pt.y));
		// P1.push_back(matched1[i].pt);
	}
	for (int i = 0; i< matched2.size() ; i++){
		P2.push_back(Point2f(matched2[i].pt.x,matched2[i].pt.y));
	}	

	
#if 1
	Mat F = findFundamentalMat(P1,P2,FM_RANSAC,1.0,0.99);

	cout << "F:"<< F << endl;

	if (F.empty()){
		cout << "F is empty " << endl;
		return 0;
	}
	Mat H1,H2;
	stereoRectifyUncalibrated(P1,P2,F,imageSize,H1,H2,3);
	cout << "H :"<< H1<< "\n" << H2<< endl;

	const double Fx = 3478.5054928;
	const double Fy = 3485.1401911;
	const double Cx = 2077.1772353;
	const double Cy = 1550.1664024;
	M1 = (Mat_<double>(3, 3)<<Fx,0.,Cx,0,Fy,Cy,0,0,1);
	M2 = M1;
	
	Mat map11, map12, map21, map22;
        initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

        Mat img1r, img2r;
        remap(img1, img1r, map11, map12, INTER_LINEAR);
        remap(img2, img2r, map21, map22, INTER_LINEAR);

	imwrite("home/ply/image/remap1.jpg",img1);
	imwrite("home/ply/image/remap1r.jpg",img1r);
	

#endif

////////////////////////////////////////////////////////////////////////
#if 0

Mat M1, D1, M2, D2;
//  read M from file 

	const char* intrinsic_filename =  "intrinsics.yml";
   	  
	FileStorage fsin(intrinsic_filename, FileStorage::READ);
	if(!fsin.isOpened())
	{
	    printf("Failed to open file %s\n", intrinsic_filename);
	    return -1;
	}

	
	fsin["M1"] >> M1;
	fsin["M2"] >> M2;


	const double Fx = 3478.5054928;
	const double Fy = 3485.1401911;
	const double Cx = 2077.1772353;
	const double Cy = 1550.1664024;
	M1 = (Mat_<double>(3, 3)<<Fx,0.,Cx,0,Fy,Cy,0,0,1);
	M2 = M1;
	
/*

	 Mat mask; // inlier mask
	 undistortPoints(imgpts1, imgpts1, K, dist_coefficients, noArray(), K);
	 undistortPoints(imgpts2, imgpts2, K, dist_coefficients, noArray(), K);
	 double focal = K.at<double>(0,0);
	 Point2d principalPoint(K.at<double>(0,2), K.at<double>(1,2));
	 Mat E = findEssentialMat(imgpts1, imgpts2, focal, principalPoint, RANSAC, 0.999, 3, mask);
	 correctMatches(E, imgpts1, imgpts2, imgpts1, imgpts2);
	 recoverPose(E, imgpts1, imgpts2, R, t, focal, principalPoint, mask); 

It might be that correctedMatches should be called with (non-normalised) image/pixel coordinates


*/


	Mat mask;
	double focal_length = 0.5*(M1.at<double>(0) + M1.at<double>(4));
	Point2d principle_point(M1.at<double>(2), M1.at<double>(5));
	Mat E = findEssentialMat(P1, P2, focal_length, principle_point, FM_RANSAC, 0.999, 1.0, mask);

	 
	cout << "E:"<< E << endl;

	
	double feasible_count = countNonZero(mask);
	cout << (int)feasible_count << " -in- " << feasible_count / P1.size() << endl;
//    (feasible_count / P1.size())   <0.6    because if it value > 50% ,the result is unreliable;

//	if (feasible_count <= 15 || (feasible_count / P1.size()) < 0.6)
//	    return 0;
	    
	Mat R,T;
	int pass_count = recoverPose(E, P1, P2, R, T, focal_length, principle_point, mask);

	
	
	cout << "R:"<<R<< endl;
	cout << "T:"<<T<< endl;
	
/*	
	if (((double)pass_count) / feasible_count < 0.7)
	    return 0;

*/	

	Mat proj1(3, 4, CV_32FC1);
    	Mat proj2(3, 4, CV_32FC1);
			
	proj1(Range(0, 3), Range(0, 3)) = Mat::eye(3, 3, CV_32FC1);
	proj1.col(3) = Mat::zeros(3, 1, CV_32FC1);

	R.convertTo(proj2(Range(0, 3), Range(0, 3)), CV_32FC1);
	
	cout <<" proj2:"<< proj2 << endl;
	T.convertTo(proj2.col(3), CV_32FC1);

	cout <<" proj2:"<< proj2 << endl;

	
	Mat fK1,fK2,structure;
	M1.convertTo(fK1, CV_32FC1);
	M2.convertTo(fK2, CV_32FC1);

		
	proj1 = fK1*proj1;
	proj2 = fK2*proj2;

	cout <<" proj1:"<< proj1 << endl;
	cout <<" proj2:"<< proj2 << endl;

	triangulatePoints(proj1, proj2, P1, P2,structure);
	
	cout << structure.rows << endl;
	cout << structure.cols << endl;
	
 	saveXYZ("out.ply", structure.t());

///////////////////////////////////////////////////////////
	
#endif


    return 0;
}
