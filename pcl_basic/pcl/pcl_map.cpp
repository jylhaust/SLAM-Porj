#include <iostream>
#include <fstream>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry>
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
/**/
int main()
{
    vector<cv::Mat> corlorImgs,depthImgs; //与直接定义Mat矩阵的区别
    //vector<Eigen::Isometry3d> poses;  //typedef Transform< double, 3, Isometry >
    //内存对齐
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses; //poses为每个图像对应的相机的位姿
    // cout<<"poses:"<<sizeof(poses)<<endl;
    ifstream fin("./pose.txt");
    if(!fin)
    {
        cerr<<"input error"<<endl;
        return 1;
    }

    for(int i=0; i<5; i++)
    {
        boost::format fmt("./%s/%d.%s");
        corlorImgs.push_back( cv::imread( (fmt%"color"%(i+1)%"png").str() ));
        depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(),-1 ));//使用-1读取原始图像

        double data[7] = {0};
        for ( auto& d:data)
        {
            fin>>d;
        }
        Eigen::Quaterniond q( data[6], data[3], data[4], data[5]); //定义四元数
        //typedef Transform<double,3,Isometry> Isometry3d;
        Eigen::Isometry3d T(q);       //变换矩阵（4X4）
        cout<<"T(q):"<<T.cols<<endl;
        //平移向量Vector3d是typedef Matrix< double ,3 ,1> Vector3d简化
        T.pretranslate(Eigen::Vector3d( data[0],data[1],data[2] )); //???
        //
       poses.push_back(T);
    }
    // cout<<"T:"<<sizeof(poses[1])<<endl; 
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;

    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    PointCloud::Ptr pointCloud(new PointCloud);

    for( int i=0;i<5;i++ )
    {
        cv::Mat color = corlorImgs[i];
        // cout<<color.cols<<","<<color.rows<<endl;
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i]; //相机的位姿
        for( int v=0; v<color.rows;v++)
        {
            for( int u=0;u<color.cols; u++)
            {
                unsigned int d = depth.ptr<unsigned short> (v)[u]; //获取每个像素点对应的深度值
                if( d==0 ) continue;
                Eigen::Vector3d point;          //相机坐标系下的坐标Pc
                point[2] = double(d)/depthScale;
                point[0] = (u-cx)*point[2]/fx;
                point[1] = (v-cy)*point[2]/fy;

                Eigen::Vector3d pointWorld = T*point; //变换到世界坐标系下Pw

                PointT p;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                p.b = color.data[ v*color.step+u*color.channels() ];
                p.g = color.data[ v*color.step+u*color.channels()+1 ];
                p.r = color.data[ v*color.step+u*color.channels()+2 ];
                pointCloud->points.push_back(p);
            }
        }

    }

    pointCloud->is_dense = false;
    cout<<"point-number:"<<pointCloud->size()<<endl;
    pcl::io::savePCDFileBinary( "map.pcd",*pointCloud );  
    return 0;
}
