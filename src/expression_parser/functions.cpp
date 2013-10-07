/**
 * @file functions.cpp
 *
 * @brief Class containing extra functions which are not available in the
 * default c++ libraries.
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


#include "error.h"

using namespace std;



/*
 * calculate factorial of value
 * for example 5! = 5*4*3*2*1 = 120
 */
double factorial(double value)
{
    double res;
    int v = static_cast<int>(value);

    if (value != static_cast<double>(v))
    {
        throw Error(-1, -1, 400, "factorial");
    }

    res = v;
    v--;
    while (v > 1)
    {
        res *= v;
        v--;
    }

    if (res == 0) res = 1;        // 0! is per definition 1
    return res;
}

/*
 * calculate the sign of the given value
 */
double sign(double value)
{
    if (value > 0) return 1;
    if (value < 0) return -1;
    return 0;
}
