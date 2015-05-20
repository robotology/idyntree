/**
 * @file main.cpp
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
 * @date	2007, updated 2012-02-06
 */


// declarations
#include <cstdlib>
#include <cstdio>

#include "parser.h"


using namespace std;

int main(int argc, char *argv[])
{
    char expr[255];

    // create a parser object
    Parser prs;

    puts("Enter an expression an press Enter to calculate the result.");
    puts("Enter an empty expression to quit.");
    puts("");

    do
    {
        // request an expression
        printf("> ");
        gets(expr);

        if (strcmp(expr, "") != 0)
        {
            try
            {
                // evaluate the expression
                char* result;
                result = prs.parse(expr);
                printf("\t%s\n", result);
            }
            catch (...)
            {
                printf("\tError: Unknown error occured in parser\n");
            }
        }
    } while (strcmp(expr, "") != 0);

    return EXIT_SUCCESS;
}
