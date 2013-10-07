/**
 * @file error.h
 *
 * @brief Class handling a predefined list with errors
 *
 * @license
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy
 * of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 *
 * Copyright (C) 2007-2011 Jos de Jong, http://www.speqmath.com
 *
 * @author 	Jos de Jong, <wjosdejong@gmail.com>
 * @date	2007-12-22, updated 2012-02-06
 */


#ifndef ERROR_H
#define ERROR_H

#include <cstdio>
#include <cstdarg>
#include <cstring>

using namespace std;



class Error {
    public:
        Error(const int row, const int col, const int id, ...);

        int get_row() {return err_row;} // Returns the row of the error
        int get_col() {return err_col;} // Returns the column of the error
        int get_id() {return err_id;}   // Returns the id of the error
        char* get_msg() {return msg;}   // Returns a pointer to the error msg

    private:
        int err_row;    // row where the error occured
        int err_col;    // column (position) where the error occured
        int err_id;     // id of the error
        char msg[255];

        const char* msgdesc(const int id);
};


#endif
