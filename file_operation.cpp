/**
 * @file file_operation
 * @brief 读取文件中的数据
 *
 * Copyright (c) 2024 [JUYI LI].
 * All rights reserved. Licensed under the MIT License.
 *
 * author: JUYI LI
 * created: 2024-04-12
 */

#include "file_operation.h"

using namespace std;

file_operation::file_operation()
{
	//构造函数
}  

file_operation::~file_operation()
{
	//析构函数
} 

int file_operation::file_open(const char *file_name)
{
    f.open(file_name, ios::in|ios::out);
    if (!f.is_open())
    {
        cerr << "open file " << file_name << " failed!" << endl;
        return -1;
    }

    return 0;
}

int file_operation::file_read_data(double *data, int data_row_length, int data_col_length)
{
    string line;
    while (getline(f, line))
    {
        if(!line.empty())
        {
//            cout << line << endl;

            istringstream ss(line);

            string d;

//            cout << "data: ";
            int ss_count = 0;
            while (ss>>d)
            {
                if (ss_count > 5)
                {
                    data[ss_count-6] = stod(d);
//                    cout << data[ss_count-6] << " ";
                }
                ss_count++;
            }
//            cout << endl;
        }
    }

    return 0;
}

int file_operation::file_read_data(std::vector<std::vector<double> > &data, int data_start_index, int data_length, int start_line)
{
    string line;

    int current_line = 0; // 当前行计数器

    // 首先跳过指定的开始行数
    while (current_line < start_line && getline(f, line)) {
        current_line++;
    }

    // 检查是否成功跳转到指定行
    if (current_line < start_line) {
        // 文件行数不足，无法达到指定的开始行
        return -1; // 返回错误码
    }

    while (getline(f, line))
    {
        if(!line.empty())
        {
            string d;
            vector<double> data_in_one_row;

//            cout << "data ";

            istringstream ss(line);
            int ss_count = 0;
            while (ss>>d)
            {
                if (ss_count >= data_start_index and ss_count <= data_start_index+data_length)
                {
                    data_in_one_row.push_back(stod(d));
//                    cout << stod(d) << " ";
                }
                ss_count++;
            }

//            cout << endl;
            data.push_back(data_in_one_row);
        }
    }

    return 0;
}

