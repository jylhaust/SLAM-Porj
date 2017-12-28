#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;
/**/
void find_feature_matches(
    const Mat& img1, const Mat& img2,
    vector<KeyPoint>& keypoint1,
    vector<KeyPoint>& keypoint2,
    vector<DMatch>& matches );

void pose_estimation_2d2d(
    vector<KeyPoint> keypoint1,
    vector<KeyPoint> keypoint2,
    vector<DMatch> matches,
    Mat& R, Mat& t );

Point2d pixel2cam ( const Point2d& p, const Mat& k );

int main( int argc, char** argv )
{
   if( argc != 2 )
   {
       cout<<"img not find"<<endl;
        // return 1;
   }
   cout<<"argc:"<<" "<<argc<<endl;

    cv::Mat img1 = cv::imread( argv[1], CV_LOAD_IMAGE_COLOR );///home/yuanlin/SLAM-Porj/VO/6.jpg
    cv::Mat img2 = cv::imread( argv[2], CV_LOAD_IMAGE_COLOR );

    vector<KeyPoint> keypoint1, keypoint2;
    vector<DMatch> matches;
    // find_feature_matches (img1, img2, keypoint1, keypoint2, matches);
/* */
    Mat R,t;
    // pose_estimation_2d2d( keypoint1, keypoint2, matches, R, t );

    Mat tx = ( Mat_<double> (3,3) << 
                0,  -t.at<double> (2,0), t.at<double> (1,0), 
                t.at<double> (2,0), 0,   -t.at<double> (0,0)         
                -t.at<double> (1,0), t.at<double> (0,0), 0 );
    cout<<"t^R:"<<tx*R<<endl;        
     
    cv::imshow( "origin",img1 );
    cv::waitKey(0);
    return 0;
}

void pose_estimation_2d2d( 
    vector<KeyPoint> keypoint1,
    vector<KeyPoint> keypoint2,
    vector<DMatch> matches,
    Mat& R, Mat& t)
{
    //相机内参
    Mat K = ( Mat_<double> (3,3 ) << 520.9, 325.1, 0, 521.0, 249.7,0 ,0, 1);

    vector<Point2f> point1;
    vector<Point2f> point2;
    for( int i = 0; i<( int ) matches.size(); i++)
    {
        point1.push_back( keypoint1[matches[i].queryIdx].pt );
        point2.push_back( keypoint2[ matches[i].trainIdx].pt );

    }
    //基础矩阵计算
    Mat funda_mat;
    // funda_mat = findFundamentalMat ( point1, point2, CV_FM_8POINT );

    //本质矩阵计算
    // cv::Point2d principal_point (325.1, 249.7);
    int focal_length = 521; 
    Mat esst_mat;
    // esst_mat = findEssentialMat( point1, point2, focal_length, principal_point, RANSAC );

    //单应矩阵计算
    Mat homo_mat;
    // homo_mat = findHomography( point1, point2, RANSAC, 3, noArray());

    // recoverPose( esst_mat, point1, point2, R, t, focal_length, principal_point);

}
/*
void find_feature_matches( const Mat& img1, const Mat& img2,
                        vector<KeyPoint>& keypoint1,
                        vector<KeyPoint<& keypoint2,
                        vector< DMatch >& matches )
{at
    Mat descriptor1, descriptor2;
    // Ptr<FeatureDetector> detector = ORB::create();
    // Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );

    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::creat( "BruteForce-Hamming");
    detector->compute ( img1, keypoint1, descriptor1 );
    detector->compute ( img2, keypoint2, descriptor2 );

    vector<DMatch> match;
    matcher->match( descriptor1, descriptor2, match);
    double min_dist = 10000, max_dist = 0;

    for( int i = 0;i < descriptor1.rows; i++)
    {
        double dist = match[i].distance;
        if ( dist<min_dist ) min_dist = dist;
        if ( dist>max_dist ) max_dist = dist;
    }
    for( int i = 0; i < descriptor1.rows; i++ )
    {
        if( match[i].distance <= max ( 2*min_dist, 30 ))
            matches.push_back( match[i] );
    }
}
*/
Point2d pixel2cam( const Point2d& p, const Mat& K )
{
    return Point2d
            (
                ( p.x - K.at<double> (0,2) ) / K.at<double> (0,0),
                ( p.y - K.at<double> (1,2) ) / K.at<double> (1,1)
            );
}