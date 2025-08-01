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

#ifndef FILE_OPERATION_H
#define FILE_OPERATION_H

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

class file_operation
{
public:	
    //构造和析构函数
    /**
     * @brief file_operation
     * @param param
     */
    file_operation();

    ~file_operation();

    /**
     * @brief 打开文件
     * @param file_name 文件名
     * @return 成功返回0,失败返回-1
     */
    int file_open(const char * file_name);

    int file_read_data(double * data, int data_row, int data_col_length);

    /**
     * @brief 从文件中读取double，空行会自动跳过
     * @param data 数据存储地
     * @param data_start_index 有效数据开始位置，index从0开始，以空格间隔
     * @param data_length 有效数据长度
     * @param start_line 开始读取的行数，index从0开始，默认为0
     * @return 成功返回0
     */
    int file_read_data(std::vector<std::vector<double>> &data, int data_start_index, int data_length, int start_line = 0);
	
protected:
	//protected members here;
	//normally, most of variables and inside functions are here;
    std::ifstream f;

private:
	//private members here;
};

#endif
