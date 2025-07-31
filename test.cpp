#include "file_operation.h"
#include <unistd.h>
#include <iostream>
#include <string>			//string lib
#include "Eigen/Eigen"
#include <cmath>
#include "curve_fitting.h"
#include "robotarm_kinematic.h"
#include <algorithm>

using namespace std;
using namespace Eigen;

void test(MatrixXd t)
{
    cout << t << endl;
}

int main(int argc, char *argv[])
{
    file_operation fo;
//    fo.file_open("/home/ubuntu1604/data/data_202403271414");
//    fo.file_open("/home/ubuntu1604-lenovo-workstation/libhub/src/Linktouch_ST6_500_20_ftsensor_data/build/force_calibration/data_202403260907");
//    fo.file_open("/home/ubuntu1604-lenovo-workstation/libhub/src/ft_sensor_data/build/force_calibration/data_202404091738");
    fo.file_open("/home/ubuntu2204/libhub/data/2025.07.03_16.33.35_sine_2deg_1Hz.txt");
    vector<vector<double>> data;
    fo.file_read_data(data, 1, 8, 1);

//    curve_fitting cf;

//    robotarm_kinematic rk(1);

//    double data_max[6] = {-1000.0, -1000.0, -1000.0, -1000.0, -1000.0, -1000.0};
//    double data_min[6] = {1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0};
//    double max_error[6] = {0};

    /******************************************双imu加速度拟合力*****************************************/
//    vector<vector<double>> data;
//    fo.file_read_data(data, 6, 15);
//    MatrixXd x_data_matrix(data.size(),6);
//    VectorXd x_data_vector(data.size());
//    MatrixXd y_data_matrix(data.size(),6);
//    VectorXd y_data_vector(data.size());
//    MatrixXd z_data_matrix(data.size(),6);
//    VectorXd z_data_vector(data.size());
//    MatrixXd data_error(data.size(), 3);

//    for (int i=0; i<data.size(); i++)
//    {
//        Matrix3d calibration_matrix = (rk.rotation_with_z_axis(data[i][2]) * rk.rotation_with_y_axis(data[i][1]) *  rk.rotation_with_x_axis(data[i][0]) * rk.rotation_with_x_axis(3.1415926)).inverse();
//        x_data_matrix(i,0) = calibration_matrix(0,0);
//        x_data_matrix(i,1) = calibration_matrix(0,1);
//        x_data_matrix(i,2) = calibration_matrix(0,2);
//        x_data_matrix(i,3) = 1;
//        x_data_matrix(i,4) = data[i][9];
//        x_data_matrix(i,5) = data[i][12];
//        x_data_vector(i) = data[i][3];

//        y_data_matrix(i,0) = calibration_matrix(1,0);
//        y_data_matrix(i,1) = calibration_matrix(1,1);
//        y_data_matrix(i,2) = calibration_matrix(1,2);
//        y_data_matrix(i,3) = 1;
//        y_data_matrix(i,4) = data[i][10];
//        y_data_matrix(i,5) = data[i][13];
//        y_data_vector(i) = data[i][4];

//        z_data_matrix(i,0) = calibration_matrix(2,0);
//        z_data_matrix(i,1) = calibration_matrix(2,1);
//        z_data_matrix(i,2) = calibration_matrix(2,2);
//        z_data_matrix(i,3) = 1;
//        z_data_matrix(i,4) = data[i][11];
//        z_data_matrix(i,5) = data[i][14];
//        z_data_vector(i) = data[i][5];

//        data_max[0] = max(data_max[0], data[i][3]);
//        data_max[1] = max(data_max[1], data[i][4]);
//        data_max[2] = max(data_max[2], data[i][5]);

//        data_min[0] = min(data_min[0], data[i][3]);
//        data_min[1] = min(data_min[1], data[i][4]);
//        data_min[2] = min(data_min[2], data[i][5]);
//    }

//    double x_res[6],y_res[6],z_res[6];
//    double r2_x = cf.linear_fitting(x_data_matrix, x_data_vector, x_res);
//    cout << " x fitting result: " << x_res[0] << ", " << x_res[1] << ", " << x_res[2] << ", " << x_res[3] << ", " << x_res[4] << ", " << x_res[5] << "(" << r2_x << ")"<< endl;

//    double r2_y = cf.linear_fitting(y_data_matrix, y_data_vector, y_res);
//    cout << " y fitting result: " << y_res[0] << ", " << y_res[1] << ", " << y_res[2] << ", " << y_res[3] << ", " << y_res[4] << ", " << y_res[5] << "(" << r2_y << ")"<< endl;

//    double r2_z = cf.linear_fitting(z_data_matrix, z_data_vector, z_res);
//    cout << " z fitting result: " << z_res[0] << ", " << z_res[1] << ", " << z_res[2] << ", " << z_res[3] << ", " << z_res[4] << ", " << z_res[5] << "(" << r2_z << ")"<< endl;

//    for (int i=0; i<data.size(); i++)
//    {
//        data_error(i,0) = x_res[0]*x_data_matrix(i,0) + x_res[1]*x_data_matrix(i,1) + x_res[2]*x_data_matrix(i,2) + x_res[3]*x_data_matrix(i,3) + x_res[4]*x_data_matrix(i,4) + x_res[5]*x_data_matrix(i,5) - x_data_vector[i];
//        data_error(i,1) = y_res[0]*y_data_matrix(i,0) + y_res[1]*y_data_matrix(i,1) + y_res[2]*y_data_matrix(i,2) + y_res[3]*y_data_matrix(i,3) + y_res[4]*y_data_matrix(i,4) + y_res[5]*y_data_matrix(i,5) - y_data_vector[i];
//        data_error(i,2) = z_res[0]*z_data_matrix(i,0) + z_res[1]*z_data_matrix(i,1) + z_res[2]*z_data_matrix(i,2) + z_res[3]*z_data_matrix(i,3) + z_res[4]*z_data_matrix(i,4) + z_res[5]*z_data_matrix(i,5) - z_data_vector[i];

//        max_error[0] = max(max_error[0], fabs(data_error(i,0)));
//        max_error[1] = max(max_error[1], fabs(data_error(i,1)));
//        max_error[2] = max(max_error[2], fabs(data_error(i,2)));
//    }
    /***************************************************************************************************/
    /******************************************双imu加速度拟合力 Matrix*****************************************/
//    vector<vector<double>> data;
//    fo.file_read_data(data, 6, 15);
//    MatrixXd f_data_matrix(data.size()*3,9);
//    VectorXd f_data_vector(data.size()*3);
//    VectorXd f_res_vector(9);
//    for (int i=0; i<data.size(); i++)
//    {
//        Matrix3d calibration_matrix = (rk.rotation_with_z_axis(data[i][2]) * rk.rotation_with_y_axis(data[i][1]) *  rk.rotation_with_x_axis(data[i][0]) * rk.rotation_with_x_axis(3.1415926)).inverse();
//        Matrix3d accelartion_matrix;
//        accelartion_matrix << data[i][9], data[i][12], 0, data[i][10], data[i][13], 0, 0, 0, (data[i][11]-data[i][14])/2.0;

//        f_data_matrix.block(i*3,0,3,3) = calibration_matrix;
//        f_data_matrix.block(i*3,3,3,3) = accelartion_matrix;
//        f_data_matrix.block(i*3,6,3,3) = Matrix3d::Identity();

//        f_data_vector(i*3) = data[i][3];
//        f_data_vector(i*3+1) = data[i][4];
//        f_data_vector(i*3+2) = data[i][5];

//        data_max[0] = max(data_max[0], data[i][3]);
//        data_max[1] = max(data_max[1], data[i][4]);
//        data_max[2] = max(data_max[2], data[i][5]);

//        data_min[0] = min(data_min[0], data[i][3]);
//        data_min[1] = min(data_min[1], data[i][4]);
//        data_min[2] = min(data_min[2], data[i][5]);
//    }

//    double f_res[9] = {0};
//    double f = cf.linear_fitting(f_data_matrix, f_data_vector, f_res);
//    cout << "f_res:" <<  f_res[0] << " " << f_res[1] << " " << f_res[2] << " " << f_res[3] << " " << f_res[4] << " " << f_res[5] << " " << f_res[6] << " " << f_res[7] << " " << f_res[8] << " (" << f << ")" << endl;
//    for (int i=0; i<9; i++)
//    {
//        f_res_vector(i) = f_res[i];
//    }

//    VectorXd f_data_test(data.size()*3);
//    f_data_test = f_data_matrix * f_res_vector;

//    for (int i=0; i<data.size(); i++)
//    {
//        max_error[0] = max(max_error[0], fabs(f_data_test(i*3)-f_data_vector(i*3)));
//        max_error[1] = max(max_error[1], fabs(f_data_test(i*3+1)-f_data_vector(i*3+1)));
//        max_error[2] = max(max_error[2], fabs(f_data_test(i*3+2)-f_data_vector(i*3+2)));
//    }
    /***************************************************************************************************/
    /******************************************无imu加速度拟合力*****************************************/
//    vector<vector<double>> data;
//    fo.file_read_data(data, 0, 9);
//    for (int i=0; i<data.size(); i++)
//    {
//        for (int j=0; j<9; j++)
//        {
//            cout << data[i][j] << " ";
//        }
//        cout << endl;
//    }


//    MatrixXd x_data_matrix(data.size(),4);
//    VectorXd x_data_vector(data.size());
//    MatrixXd y_data_matrix(data.size(),4);
//    VectorXd y_data_vector(data.size());
//    MatrixXd z_data_matrix(data.size(),4);
//    VectorXd z_data_vector(data.size());
//    MatrixXd data_error(data.size(), 3);

//    for (int i=0; i<data.size(); i++)
//    {
//        //nbit test
////        Matrix3d calibration_matrix = (rk.rotation_with_z_axis(data[i][2]) * rk.rotation_with_y_axis(data[i][1]) *  rk.rotation_with_x_axis(data[i][0]) * rk.rotation_with_x_axis(3.1415926)).inverse();
//        //linktouch test
////        Matrix3d calibration_matrix = (rk.rotation_with_z_axis(data[i][2]) * rk.rotation_with_y_axis(data[i][1]) *  rk.rotation_with_x_axis(data[i][0]) * rk.rotation_with_x_axis(3.1415926) * rk.rotation_with_z_axis(-3.1415926/4.0)).inverse();
//        Matrix3d calibration_matrix = (rk.rotation_with_z_axis(data[i][2]) * rk.rotation_with_y_axis(data[i][1]) *  rk.rotation_with_x_axis(data[i][0])).inverse();

//        x_data_matrix(i,0) = calibration_matrix(0,0);
//        x_data_matrix(i,1) = calibration_matrix(0,1);
//        x_data_matrix(i,2) = calibration_matrix(0,2);
//        x_data_matrix(i,3) = 1;
//        x_data_vector(i) = data[i][3];

//        y_data_matrix(i,0) = calibration_matrix(1,0);
//        y_data_matrix(i,1) = calibration_matrix(1,1);
//        y_data_matrix(i,2) = calibration_matrix(1,2);
//        y_data_matrix(i,3) = 1;
//        y_data_vector(i) = data[i][4];

//        z_data_matrix(i,0) = calibration_matrix(2,0);
//        z_data_matrix(i,1) = calibration_matrix(2,1);
//        z_data_matrix(i,2) = calibration_matrix(2,2);
//        z_data_matrix(i,3) = 1;
//        z_data_vector(i) = data[i][5];

//        data_max[0] = max(data_max[0], data[i][3]);
//        data_max[1] = max(data_max[1], data[i][4]);
//        data_max[2] = max(data_max[2], data[i][5]);

//        data_min[0] = min(data_min[0], data[i][3]);
//        data_min[1] = min(data_min[1], data[i][4]);
//        data_min[2] = min(data_min[2], data[i][5]);
//    }

//    double x_res[4],y_res[4],z_res[4];
//    double r2_x = cf.linear_fitting(x_data_matrix, x_data_vector, x_res);
//    cout << " x fitting result: " << x_res[0] << ", " << x_res[1] << ", " << x_res[2] << ", " << x_res[3] << "(" << r2_x << ")"<< endl;
//    double r2_y = cf.linear_fitting(y_data_matrix, y_data_vector, y_res);
//    cout << " y fitting result: " << y_res[0] << ", " << y_res[1] << ", " << y_res[2] << ", " << y_res[3] << "(" << r2_y << ")"<< endl;
//    double r2_z = cf.linear_fitting(z_data_matrix, z_data_vector, z_res);
//    cout << " z fitting result: " << z_res[0] << ", " << z_res[1] << ", " << z_res[2] << ", " << z_res[3] << "(" << r2_z << ")"<< endl;

//    for (int i=0; i<data.size(); i++)
//    {
//        data_error(i,0) = x_res[0]*x_data_matrix(i,0) + x_res[1]*x_data_matrix(i,1) + x_res[2]*x_data_matrix(i,2) + x_res[3]*x_data_matrix(i,3) - x_data_vector[i];
//        data_error(i,1) = y_res[0]*y_data_matrix(i,0) + y_res[1]*y_data_matrix(i,1) + y_res[2]*y_data_matrix(i,2) + y_res[3]*y_data_matrix(i,3) - y_data_vector[i];
//        data_error(i,2) = z_res[0]*z_data_matrix(i,0) + z_res[1]*z_data_matrix(i,1) + z_res[2]*z_data_matrix(i,2) + z_res[3]*z_data_matrix(i,3) - z_data_vector[i];

//        max_error[0] = max(max_error[0], fabs(data_error(i,0)));
//        max_error[1] = max(max_error[1], fabs(data_error(i,1)));
//        max_error[2] = max(max_error[2], fabs(data_error(i,2)));
//    }
    /***************************************************************************************************/

    /******************************************无imu加速度拟合力矩*****************************************/
//    MatrixXd tx_data_matrix(data.size(),3);
//    VectorXd tx_data_vector(data.size());
//    MatrixXd ty_data_matrix(data.size(),3);
//    VectorXd ty_data_vector(data.size());
//    MatrixXd tz_data_matrix(data.size(),1);
//    VectorXd tz_data_vector(data.size());
//    MatrixXd tdata_error(data.size(), 3);

//    MatrixXd t_data_matrix(data.size()*3,6);
//    VectorXd t_data_vector(data.size()*3);
//    VectorXd t_res_vector(6);

//    for (int i=0; i<data.size(); i++)
//    {
//        tx_data_matrix(i,0) = -data[i][5];
//        tx_data_matrix(i,1) = data[i][4];
//        tx_data_matrix(i,2) = 1;
//        tx_data_vector(i) = data[i][6];

//        ty_data_matrix(i,0) = data[i][5];
//        ty_data_matrix(i,1) = -data[i][3];
//        ty_data_matrix(i,2) = 1;
//        ty_data_vector(i) = data[i][7];

//        t_data_matrix(i*3,0) = 0;
//        t_data_matrix(i*3,1) = -data[i][5];
//        t_data_matrix(i*3,2) = data[i][4];
//        t_data_matrix(i*3,3) = 1;
//        t_data_matrix(i*3,4) = 0;
//        t_data_matrix(i*3,5) = 0;

//        t_data_matrix(i*3+1,0) = data[i][5];
//        t_data_matrix(i*3+1,1) = 0;
//        t_data_matrix(i*3+1,2) = -data[i][3];
//        t_data_matrix(i*3+1,3) = 0;
//        t_data_matrix(i*3+1,4) = 1;
//        t_data_matrix(i*3+1,5) = 0;

//        t_data_matrix(i*3+2,0) = -data[i][4];
//        t_data_matrix(i*3+2,1) = data[i][3];
////        t_data_matrix(i*3+2,0) = 0;
////        t_data_matrix(i*3+2,1) = 0;
//        t_data_matrix(i*3+2,2) = 0;
//        t_data_matrix(i*3+2,3) = 0;
//        t_data_matrix(i*3+2,4) = 0;
//        t_data_matrix(i*3+2,5) = 1;

//        t_data_vector(i*3) = data[i][6];
//        t_data_vector(i*3+1) = data[i][7];
//        t_data_vector(i*3+2) = data[i][8];

//        data_max[3] = max(data_max[3], data[i][6]);
//        data_max[4] = max(data_max[4], data[i][7]);
//        data_max[5] = max(data_max[5], data[i][8]);

//        data_min[3] = min(data_min[3], data[i][6]);
//        data_min[4] = min(data_min[4], data[i][7]);
//        data_min[5] = min(data_min[5], data[i][8]);
//    }

//    double tx_res[3],ty_res[3],tz_res[3];
//    double tr2_x = cf.linear_fitting(tx_data_matrix, tx_data_vector, tx_res);
//    cout << " tx fitting result: " << tx_res[0] << ", " << tx_res[1] << ", " << tx_res[2] << "(" << tr2_x << ")"<< endl;
//    double tr2_y = cf.linear_fitting(ty_data_matrix, ty_data_vector, ty_res);
//    cout << " ty fitting result: " << ty_res[0] << ", " << ty_res[1] << ", " << ty_res[2] << "(" << tr2_y << ")"<< endl;
//    double tr2_z = cf.linear_fitting(tz_data_matrix, tz_data_vector, tz_res);
//    cout << " tz fitting result: " << tz_res[0] << ", " << tz_res[1] << ", " << tz_res[2] << "(" << tr2_z << ")"<< endl;

////    for (int i=0; i<data.size(); i++)
////    {
////        tz_data_matrix(i,0) = 1;
////        tz_data_vector(i) = data[i][8]+data[i][4]*ty_res[0]-data[i][3]*tx_res[0];
////    }

////    double tr2_z = cf.linear_fitting(tz_data_matrix, tz_data_vector, tz_res);
////    cout << " tz fitting result: " << tz_res[0] << "(" << tr2_z << ")"<< endl;

//    double t_res[6] = {0};
//    double tr2 = cf.linear_fitting(t_data_matrix, t_data_vector, t_res);
//    cout << "t_res_vector: " << t_res[0] << ", " << t_res[1] << ", " << t_res[2] << ", " << t_res[3] << ", " << t_res[4] << ", " << t_res[5] << "(" << tr2 << ")"<< endl;
//    t_res_vector[0] = t_res[0];
//    t_res_vector[1] = t_res[1];
//    t_res_vector[2] = t_res[2];
//    t_res_vector[3] = t_res[3];
//    t_res_vector[4] = t_res[4];
//    t_res_vector[5] = t_res[5];
//    VectorXd t_data_test(data.size()*3);
//    t_data_test = t_data_matrix * t_res_vector;

//    for (int i=0; i<data.size(); i++)
//    {
//        max_error[3] = max(max_error[3], fabs(t_data_test(i*3)-t_data_vector(i*3)));
//        max_error[4] = max(max_error[4], fabs(t_data_test(i*3+1)-t_data_vector(i*3+1)));
//        max_error[5] = max(max_error[5], fabs(t_data_test(i*3+2)-t_data_vector(i*3+2)));
//    }
////    cout << "error:" << t_data_vector-t_data_matrix*t_res_vector << endl;

    /***************************************************************************************************/


//    cout << data_error << endl;
//    cout << max_error[0] << "," << max_error[1] << "," << max_error[2] << "," << max_error[3] << "," << max_error[4] << "," << max_error[5] << endl;
//    cout << data_max[0] << "," << data_max[1] << "," << data_max[2] << "," << data_max[3] << "," << data_max[4] << "," << data_max[5] << endl;
//    cout << data_min[0] << "," << data_min[1] << "," << data_min[2] << "," << data_min[3] << "," << data_min[4] << "," << data_min[5] << endl;

    return 1;
}
