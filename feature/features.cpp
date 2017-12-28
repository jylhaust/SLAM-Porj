#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

int main( int argc, char** argv)
{
    cv::Mat img1 = cv::imread("/home/yuanlin/SLAM-Porj/feature/a.jpg");
    cv::Mat img2 = cv::imread("/home/yuanlin/SLAM-Porj/feature/b.jpg");

    vector<cv::KeyPoint> keypoint1, keypoint2; //关键点
    cv::Mat descriptor1, descriptor2;          //描述子  
    // cv::Ptr<cv::ORB> orb = cv::ORB( 500, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31,20 );//OpenCV 3.0
    cv::ORB orb;

    // 1： 检测FAST角点位置
    orb.detect( img1,keypoint1 );
    orb.detect( img2,keypoint2 );

    // 2： 根据角点位置计算BRIEF描述子
    orb.compute( img1,keypoint1, descriptor1);
    orb.compute( img2,keypoint2, descriptor2);

    cv::Mat outimg1;
    cv::drawKeypoints( img1,keypoint1,outimg1,cv::Scalar::all(-1),cv::DrawMatchesFlags::DEFAULT);
    // cv::imshow("orbfea",outimg1);

    // 3: 描述子匹配，使用Hamming距离
    vector<cv::DMatch> matches;
    cv::BFMatcher matcher( cv::NORM_HAMMING );
    matcher.match( descriptor1,descriptor2,matches );

    //4: 匹配后点对的筛选
    double min_dist = 10000, max_dist = 0;
    for ( int i = 0;i < descriptor1.rows; i++ )
    {
        double dist = matches[i].distance;
        if ( dist<min_dist ) min_dist = dist;
        if ( dist>max_dist ) max_dist = dist;
    }
    cout<<"Max dist: "<<max_dist<<endl;
    cout<<"min dist: "<<min_dist<<endl;
    // ->
    vector< cv::DMatch > good_matches;
    for ( int i=0;i<descriptor1.rows;i++ )
    {
         if( matches[i].distance <= max (2*min_dist, 30.0) )
            good_matches.push_back( matches[i] );
    }

    //5: 绘制匹配结果
    cv::Mat img_match;
    cv::Mat img_goodmatch;        
    cv::drawMatches( img1, keypoint1, img2, keypoint2, matches, img_match );
    cv::drawMatches( img1, keypoint1, img2, keypoint2, good_matches, img_goodmatch);
    cv::imshow( "img_match", img_match );
    cv::imshow( "img_goodmatch", img_goodmatch );
    cv::waitKey(0);
    return 0;
}