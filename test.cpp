#include "file_operation.h"
#include <unistd.h>
#include <iostream>
#include <string>			//string lib
#include "Eigen/Eigen"
#include <cmath>
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
    fo.file_open("../1.txt");
    vector<vector<double>> data;
    fo.file_read_data(data, 0, 2, 1);

    return 1;
}
