#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;

struct CUREVE_FITTING_COST
{
    //利用结构体函数初始化
    CUREVE_FITTING_COST( double x, double y) : _x(x),_y(y) {}

    template <typename T>
    bool operator() ( const T* const abc, T* residual ) const //参数是模型参数（3维）与残差 
    {
        // y - exp(ax^2+bx+c)
        residual[0] = T (_y) - ceres::exp ( abc[0]*T(_x)*T(_x) + abc[1]*T(_x) + abc[2] );
        return true;
    }

    const double _x, _y;
}; 

int main( int arc, char** argv )
{
    double a = 1.0,b = 2.0, c = 1.0;
    int N = 100;
    double w_sigma=1.0;
    cv::RNG rng;          //产生高斯随机噪声
    double abc[3] = {0,0,0};
    vector<double> x_data, y_data;
    
    for( int i=0; i<N; i++)
    {
        double x = i/100.0;
        x_data.push_back( x );
        y_data.push_back(
            exp (a*x*x + b*x + c) + rng.gaussian ( w_sigma )
        );
       // cout<<x_data[i]<<" "<<y_data[i]<<endl;
    }

    //构造最小二乘问题
    ceres::Problem problem;
    for ( int i=0; i<N; i++)
    {
       problem.AddResidualBlock(
            // 添加误差项
            new ceres::AutoDiffCostFunction<CUREVE_FITTING_COST, 1, 3> (
                new CUREVE_FITTING_COST( x_data[i],y_data[i] )
           ),
           nullptr, //核函数，这里不需要
           abc     //待估计量
        );
    }

    //配置求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;  //增量方程的求解
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary; //优化信息
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve( options, &problem, &summary); //开始优化
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> (t2 - t1);

    cout<<"solveTimecost:"<<time_used.count()<<endl;
    cout<<"estimatedParameter:";
    for(auto a:abc) cout<<a<<" "<<endl;


    return 0;
};