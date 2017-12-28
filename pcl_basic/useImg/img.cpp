#include <iostream>
#include <chrono>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main()
{
    // printf("make success\n");
    // std::cout<<"make success!"<<endl;
    cv::Mat img;
    img = cv::imread("/home/yuanlin/SLAM-Porj/8.jpg",1);
    if(img.data == nullptr)
    {
        cerr<<"pic not find"<<endl;
        return 0;
    }
    cout<<"width:"<<img.cols<<",high:"<<img.rows<<",channels:"<<img.channels()<<endl;

    // cv::imshow("img",img);
    // cv::waitKey(0);
    // cv::Mat sdfa;
    // img = cv::imread("8.jpg",0);
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for(size_t y=0; y<img.rows; y++)
    {
        for(size_t x=0;x<img.cols;x++)
        {
            unsigned char* row_ptr = img.ptr<unsigned char> (y);//row_ptr 第y行头指针
            
            unsigned char* data_ptr = &row_ptr[x*img.channels()];//data_ptr指向待访问的像素数据
            for(int c = 0; c < img.channels(); c++)
            {
                unsigned char data = data_ptr[c];
            }
         } 
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"search picture time:"<<time_used.count()<<endl;  

    cv::Mat image_another = img;//直接赋值不会复制数据，改变image_another会改变img
    //浅拷贝+
    image_another(cv::Rect(0,0,200,200)).setTo(255);
    cv::imshow("img",img);
    cv::waitKey(0);        

    cv::Mat image_clone = img.clone();
    image_clone(cv::Rect(0,0,100,100)).setTo(255);
    cv::imshow("img2",img);
    cv::imshow("image_clone",image_clone);
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}