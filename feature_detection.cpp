#include"Rec3DTest.h"


void drawCornerOnImage(Mat& image,const Mat&binary)
{
    Mat_<uchar>::const_iterator it=binary.begin<uchar>();
    Mat_<uchar>::const_iterator itd=binary.end<uchar>();
    for(int i=0;it!=itd;it++,i++)
    {
        if(*it)
            circle(image,Point(i%image.cols,i/image.cols),5,Scalar(0,255,0),1);    
    }
}

void harris_detection(Mat &image,double threshold_value){
/*
    Mat dst, dst_norm, dst_norm_scaled;  
    dst = Mat::zeros( image.size(), CV_32FC1 );
    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.01;
    int thresh = 100;
    int max_thresh = 255;
  
    /// Detecting corners
    cornerHarris( image, dst, blockSize, apertureSize, k );

    imwrite("outimage/dst.png",dst);
  
    /// Normalizing
    normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
    convertScaleAbs( dst_norm, dst_norm_scaled );
  
    /// Drawing a circle around corners
    for( int j = 0; j < dst_norm.rows ; j++ )
       { for( int i = 0; i < dst_norm.cols; i++ )
            {
              if( (int) dst_norm.at<float>(j,i) > thresh )
                {
                 circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
                }
            }
       }

	imwrite("outimage/dst_norm_scaled.png",dst_norm_scaled);
*/		

    int blockSize = 3;
    int apertureSize = 3;
    double k = 0.001;
    Mat cornerStrength;
    cornerHarris(image,cornerStrength,blockSize, apertureSize, k);
    threshold(cornerStrength,cornerStrength,threshold_value,255,THRESH_BINARY);
    imwrite("outimage/cornerStrength.png",cornerStrength);

    double maxStrength;
    double minStrength;
    minMaxLoc(cornerStrength,&minStrength,&maxStrength);
 
    Mat dilated;
    Mat locaMax;
    dilate(cornerStrength,dilated,Mat());
    compare(cornerStrength,dilated,locaMax,CMP_EQ);
 
    Mat cornerMap;
    double qualityLevel=0.01;
    double th=qualityLevel*maxStrength; 
    threshold(cornerStrength,cornerMap,th,255,THRESH_BINARY);
    cornerMap.convertTo(cornerMap,CV_8U);
    bitwise_and(cornerMap,locaMax,cornerMap);
 
    drawCornerOnImage(image,cornerMap);
  
   
    imwrite("outimage/harris_image.png",cornerMap);



}


void surf_detection(Mat &image,int minHessian){

        
	Ptr<SURF> detector = SURF::create( minHessian );
	
	vector<KeyPoint>keypoints;
	Mat descriptors;

	detector->detectAndCompute( image, Mat(), keypoints, descriptors );
	drawKeypoints(image,keypoints,image,Scalar(255,0,0),DrawMatchesFlags::DEFAULT);
	imwrite("outimage/sift_image.png",image);

/*	
    SiftFeatureDetector  siftdtc;
    vector<KeyPoint>kp1,kp2;
 
    siftdtc.detect(image,kp1);
    Mat outimg1;
    drawKeypoints(image,kp1,outimg1);
    imshow("image1 keypoints",outimg1);
	
    KeyPoint kp;
 
    vector<KeyPoint>::iterator itvc;
    for(itvc=kp1.begin();itvc!=kp1.end();itvc++)
    {
        cout<<"angle:"<<itvc->angle<<"\t"<<itvc->class_id<<"\t"<<itvc->octave<<"\t"<<itvc->pt<<"\t"<<itvc->response<<endl;
    }
 
    siftdtc.detect(img2,kp2);
    Mat outimg2;
    drawKeypoints(img2,kp2,outimg2);
    imshow("image2 keypoints",outimg2);
 
 
    SiftDescriptorExtractor extractor;
    Mat descriptor1,descriptor2;
    BruteForceMatcher<L2<float>> matcher;
    vector<DMatch> matches;
    Mat img_matches;
    extractor.compute(img,kp1,descriptor1);
    extractor.compute(img2,kp2,descriptor2);
 
 
    imshow("desc",descriptor1);
    cout<<endl<<descriptor1<<endl;
    matcher.match(descriptor1,descriptor2,matches);
 
    drawMatches(img,kp1,img2,kp2,matches,img_matches);

*/
}




int surf_detection(Mat &img1,Mat &img2){
/*	
	vector<KeyPoint> keypoints1,keypoints2;
	SurfFeatureDetector detector(400);
	detector.detect(img1, keypoints1);
	detector.detect(img2, keypoints2);

	SurfDescriptorExtractor surfDesc;
	Mat descriptros1,descriptros2;
	surfDesc.compute(img1,keypoints1,descriptros1);
	surfDesc.compute(img2,keypoints2,descriptros2);

	BruteForceMatcher<L2<float>>matcher;
	vector<DMatch> matches;
	matcher.match(descriptros1,descriptros2,matches);
	std::nth_element(matches.begin(),matches.begin()+24,matches.end());
	matches.erase(matches.begin()+25,matches.end());

	Mat imageMatches;
	drawMatches(img1,keypoints1,img2,keypoints2,matches,
	imageMatches,Scalar(255,0,0));

	*/
}


void akaze_detection(Mat &image){

    vector<KeyPoint> kpts1, kpts2;
    Mat desc1, desc2;

    Ptr<AKAZE> akaze = AKAZE::create();
	
    akaze->detectAndCompute(image, noArray(), kpts1, desc1);
    
    Mat outimg ;
    drawKeypoints(image,kpts1,outimg);	
    imwrite("outimage/akaze_img.png", outimg);

}

void extract_features(
    vector<string>& image_names,
    vector<vector<KeyPoint>>& key_points_for_all,
    vector<Mat>& descriptor_for_all,
    vector<vector<Vec3b>>& colors_for_all
    )
{
    key_points_for_all.clear();
    descriptor_for_all.clear();
    Mat image;

    Ptr<Feature2D> sift = xfeatures2d::SIFT::create(0, 3, 0.04, 10);
    for (auto it = image_names.begin(); it != image_names.end(); ++it)
    {
        image = imread(*it);
        if (image.empty()) continue;

        vector<KeyPoint> key_points;
        Mat descriptor;
        sift->detectAndCompute(image, noArray(), key_points, descriptor);

        if (key_points.size() <= 10) continue;

        key_points_for_all.push_back(key_points);
        descriptor_for_all.push_back(descriptor);

        vector<Vec3b> colors(key_points.size());
        for (int i = 0; i < key_points.size(); ++i)
        {
            Point2f& p = key_points[i].pt;
            colors[i] = image.at<Vec3b>(p.y, p.x);
        }
        colors_for_all.push_back(colors);
    }
}


void match_features(Mat& query, Mat& train, vector<DMatch>& matches)
{
    vector<vector<DMatch>> knn_matches;
    BFMatcher matcher(NORM_L2);
    matcher.knnMatch(query, train, knn_matches, 2);

    float min_dist = FLT_MAX;
    for (int r = 0; r < knn_matches.size(); ++r)
    {
        //Ratio Test
        if (knn_matches[r][0].distance > 0.6*knn_matches[r][1].distance)
            continue;

        float dist = knn_matches[r][0].distance;
        if (dist < min_dist) min_dist = dist;
    }

    matches.clear();
    for (size_t r = 0; r < knn_matches.size(); ++r)
    {
        if (
            knn_matches[r][0].distance > 0.6*knn_matches[r][1].distance ||
            knn_matches[r][0].distance > 5 * max(min_dist, 10.0f)
            )
            continue;

        matches.push_back(knn_matches[r][0]);
    }
}


bool find_transform(Mat& K, vector<Point2f>& p1, vector<Point2f>& p2, Mat& R, Mat& T, Mat& mask)
{
    double focal_length = 0.5*(K.at<double>(0) + K.at<double>(4));
    Point2d principle_point(K.at<double>(2), K.at<double>(5));

    Mat E = findEssentialMat(p1, p2, focal_length, principle_point, RANSAC, 0.999, 1.0, mask);
    if (E.empty()) return false;

    double feasible_count = countNonZero(mask);
    cout << (int)feasible_count << " -in- " << p1.size() << endl;
    if (feasible_count <= 15 || (feasible_count / p1.size()) < 0.6)
        return false;

    int pass_count = recoverPose(E, p1, p2, R, T, focal_length, principle_point, mask);

    if (((double)pass_count) / feasible_count < 0.7)
        return false;

    return true;
}



void reconstruct(Mat& K, Mat& R, Mat& T, vector<Point2f>& p1, vector<Point2f>& p2, Mat& structure)
{
    Mat proj1(3, 4, CV_32FC1);
    Mat proj2(3, 4, CV_32FC1);

    proj1(Range(0, 3), Range(0, 3)) = Mat::eye(3, 3, CV_32FC1);
    proj1.col(3) = Mat::zeros(3, 1, CV_32FC1);

    R.convertTo(proj2(Range(0, 3), Range(0, 3)), CV_32FC1);
    T.convertTo(proj2.col(3), CV_32FC1);

    Mat fK;
    K.convertTo(fK, CV_32FC1);
    proj1 = fK*proj1;
    proj2 = fK*proj2;

    triangulatePoints(proj1, proj2, p1, p2, structure);
}



#if 1
int main()
{
	

    Mat imgL = imread("aloeL.jpg", IMREAD_GRAYSCALE);
    Mat imgR = imread("aloeR.jpg", IMREAD_GRAYSCALE);

    if(imgL.empty() || imgR.empty()){
	cout << "read image error "<< endl;
	return 0 ;
    }
	
//    double threshold_value = 0.0001;
//    harris_detection(image,threshold_value);
	
//    int minHessian = 80;
 //   sift_detection(image,minHessian);

//    akaze_detection(image);

    vector<string> image_names = {"aloeL.jpg","aloeR.jpg"};
    vector<vector<KeyPoint>> key_points_for_all;
    vector<Mat> descriptor_for_all;
    vector<vector<Vec3b>> colors_for_all;
	
    Mat query;
    Mat train;
    vector<DMatch> matches;

    
	
    extract_features( image_names,key_points_for_all,descriptor_for_all, colors_for_all);
    match_features(query, train, matches);
    find_transform(Mat& K, vector<Point2f>& p1, vector<Point2f>& p2, Mat& R, Mat& T, Mat& mask);
    reconstruct(Mat& K, Mat& R, Mat& T, vector<Point2f>& p1, vector<Point2f>& p2, Mat& structure);
	
    return 0;
	
}

#endif




