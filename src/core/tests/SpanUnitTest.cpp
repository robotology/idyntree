///////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015 Microsoft Corporation. All rights reserved.
//
// This code is licensed under the MIT License (MIT).
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
///////////////////////////////////////////////////////////////////////////////

//Most of this file has been taken from https://github.com/Microsoft/GSL/blob/master/tests/span_tests.cpp

// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/TestUtils.h>
#include <iDynTree/Span.h>

#include <Eigen/Dense>

#include <array>       // for array
#include <iostream>    // for ptrdiff_t
#include <iterator>    // for reverse_iterator, operator-, operator==
#include <memory>      // for unique_ptr, shared_ptr, make_unique, allo...
#include <regex>       // for match_results, sub_match, match_results<>...
#include <stddef.h>    // for ptrdiff_t
#include <string>      // for string
#include <type_traits> // for integral_constant<>::value, is_default_co...
#include <vector>      // for vector

using namespace std;
using namespace iDynTree;

namespace
{
struct BaseClass
{
};
struct DerivedClass : BaseClass
{
};
}

void default_constructor()
{
    {
        Span<int> s;
        ASSERT_IS_TRUE((s.size() == 0 && s.data() == nullptr));

        Span<const int> cs;
        ASSERT_IS_TRUE((cs.size() == 0 && cs.data() == nullptr));
    }

    {
        Span<int, 0> s;
        ASSERT_IS_TRUE((s.size() == 0 && s.data() == nullptr));

        Span<const int, 0> cs;
        ASSERT_IS_TRUE((cs.size() == 0 && cs.data() == nullptr));
    }

    {
#ifdef CONFIRM_COMPILATION_ERRORS
        Span<int, 1> s;
        ASSERT_IS_TRUE((s.size() == 1 && s.data() == nullptr)); // explains why it can't compile
#endif
    }

    {
        Span<int> s{};
        ASSERT_IS_TRUE((s.size() == 0 && s.data() == nullptr));

        Span<const int> cs{};
        ASSERT_IS_TRUE((cs.size() == 0 && cs.data() == nullptr));
    }
}

void size_optimization()
{
    {
        Span<int> s;
        ASSERT_IS_TRUE(sizeof(s) == sizeof(int*) + sizeof(ptrdiff_t));
    }

    {
        Span<int, 0> s;
        ASSERT_IS_TRUE(sizeof(s) == sizeof(int*));
    }
}

void from_nullptr_size_constructor()
{
    {
        Span<int> s{nullptr, static_cast<Span<int>::index_type>(0)};
        ASSERT_IS_TRUE((s.size() == 0 && s.data() == nullptr));

        Span<const int> cs{nullptr, static_cast<Span<int>::index_type>(0)};
        ASSERT_IS_TRUE((cs.size() == 0 && cs.data() == nullptr));
    }

    {
        Span<int, 0> s{nullptr, static_cast<Span<int>::index_type>(0)};
        ASSERT_IS_TRUE((s.size() == 0 && s.data() == nullptr));

        Span<const int, 0> cs{nullptr, static_cast<Span<int>::index_type>(0)};
        ASSERT_IS_TRUE((cs.size() == 0 && cs.data() == nullptr));
    }

//    {
//        auto workaround_macro = []() {
//            Span<int, 1> s{nullptr, static_cast<Span<int>::index_type>(0)};
//        };
//        CHECK_THROWS_AS(workaround_macro(), fail_fast);
//    }

//    {
//        auto workaround_macro = []() { Span<int> s{nullptr, 1}; };
//        CHECK_THROWS_AS(workaround_macro(), fail_fast);

//        auto const_workaround_macro = []() { Span<const int> cs{nullptr, 1}; };
//        CHECK_THROWS_AS(const_workaround_macro(), fail_fast);
//    }

//    {
//        auto workaround_macro = []() { Span<int, 0> s{nullptr, 1}; };
//        CHECK_THROWS_AS(workaround_macro(), fail_fast);

//        auto const_workaround_macro = []() { Span<const int, 0> s{nullptr, 1}; };
//        CHECK_THROWS_AS(const_workaround_macro(), fail_fast);
//    }

    {
        Span<int*> s{nullptr, static_cast<Span<int>::index_type>(0)};
        ASSERT_IS_TRUE((s.size() == 0 && s.data() == nullptr));

        Span<const int*> cs{nullptr, static_cast<Span<int>::index_type>(0)};
        ASSERT_IS_TRUE((cs.size() == 0 && cs.data() == nullptr));
    }
}

void from_pointer_length_constructor()
{
    int arr[4] = {1, 2, 3, 4};

    {
        for(int i = 0; i<4 ; ++i)
        {
            {
                Span<int> s = { &arr[0], i };
                ASSERT_IS_TRUE(s.size() == i);
                ASSERT_IS_TRUE(s.data() == &arr[0]);
                ASSERT_IS_TRUE(s.empty() == (i == 0));
                for (int j = 0; j < i; ++j)
                {
                    ASSERT_IS_TRUE(arr[j] == s[j]);
                    ASSERT_IS_TRUE(arr[j] == s.at(j));
                    ASSERT_IS_TRUE(arr[j] == s(j));
                }
            }
            {
                Span<int> s = { &arr[i], 4-i };
                ASSERT_IS_TRUE(s.size() == 4-i);
                ASSERT_IS_TRUE(s.data() == &arr[i]);
                ASSERT_IS_TRUE(s.empty() == (4-i == 0));
                for (int j = 0; j < 4-i; ++j)
                {
                    ASSERT_IS_TRUE(arr[j+i] == s[j]);
                    ASSERT_IS_TRUE(arr[j+i] == s.at(j));
                    ASSERT_IS_TRUE(arr[j+i] == s(j));
                }
            }
        }
    }

    {
        Span<int, 2> s{&arr[0], 2};
        ASSERT_IS_TRUE((s.size() == 2 && s.data() == &arr[0]));
        ASSERT_IS_TRUE((s[0] == 1 && s[1] == 2));
    }

    {
        int* p = nullptr;
        Span<int> s{p, static_cast<Span<int>::index_type>(0)};
        ASSERT_IS_TRUE((s.size() == 0 && s.data() == nullptr));
    }

//    {
//        int* p = nullptr;
//        auto workaround_macro = [=]() { Span<int> s{p, 2}; };
//        CHECK_THROWS_AS(workaround_macro(), fail_fast);
//    }

    {
        auto s = make_span(&arr[0], 2);
        ASSERT_IS_TRUE((s.size() == 2 && s.data() == &arr[0]));
        ASSERT_IS_TRUE((s[0] == 1 && s[1] == 2));
    }

    {
        int* p = nullptr;
        auto s = make_span(p, static_cast<Span<int>::index_type>(0));
        ASSERT_IS_TRUE((s.size() == 0 && s.data() == nullptr));
    }

//    {
//        int* p = nullptr;
//        auto workaround_macro = [=]() { make_span(p, 2); };
//        CHECK_THROWS_AS(workaround_macro(), fail_fast);
//    }
}

void from_pointer_pointer_constructor()
{
    int arr[4] = {1, 2, 3, 4};

    {
        Span<int> s{&arr[0], &arr[2]};
        ASSERT_IS_TRUE((s.size() == 2 && s.data() == &arr[0]));
        ASSERT_IS_TRUE((s[0] == 1 && s[1] == 2));
    }

    {
        Span<int, 2> s{&arr[0], &arr[2]};
        ASSERT_IS_TRUE((s.size() == 2 && s.data() == &arr[0]));
        ASSERT_IS_TRUE((s[0] == 1 && s[1] == 2));
    }

    {
        Span<int> s{&arr[0], &arr[0]};
        ASSERT_IS_TRUE((s.size() == 0 && s.data() == &arr[0]));
    }

    {
        Span<int, 0> s{&arr[0], &arr[0]};
        ASSERT_IS_TRUE((s.size() == 0 && s.data() == &arr[0]));
    }

    // this will fail the std::distance() precondition, which asserts on MSVC debug builds
    //{
    //    auto workaround_macro = [&]() { Span<int> s{&arr[1], &arr[0]}; };
    //    CHECK_THROWS_AS(workaround_macro(), fail_fast);
    //}

    // this will fail the std::distance() precondition, which asserts on MSVC debug builds
    //{
    //    int* p = nullptr;
    //    auto workaround_macro = [&]() { Span<int> s{&arr[0], p}; };
    //    CHECK_THROWS_AS(workaround_macro(), fail_fast);
    //}

    {
        int* p = nullptr;
        Span<int> s{p, p};
        ASSERT_IS_TRUE((s.size() == 0 && s.data() == nullptr));
    }

    {
        int* p = nullptr;
        Span<int, 0> s{p, p};
        ASSERT_IS_TRUE((s.size() == 0 && s.data() == nullptr));
    }

    // this will fail the std::distance() precondition, which asserts on MSVC debug builds
    //{
    //    int* p = nullptr;
    //    auto workaround_macro = [&]() { Span<int> s{&arr[0], p}; };
    //    CHECK_THROWS_AS(workaround_macro(), fail_fast);
    //}

    {
        auto s = make_span(&arr[0], &arr[2]);
        ASSERT_IS_TRUE((s.size() == 2 && s.data() == &arr[0]));
        ASSERT_IS_TRUE((s[0] == 1 && s[1] == 2));
    }

    {
        auto s = make_span(&arr[0], &arr[0]);
        ASSERT_IS_TRUE((s.size() == 0 && s.data() == &arr[0]));
    }

    {
        int* p = nullptr;
        auto s = make_span(p, p);
        ASSERT_IS_TRUE((s.size() == 0 && s.data() == nullptr));
    }
}

void from_array_constructor()
{
    int arr[5] = {1, 2, 3, 4, 5};

    {
        Span<int> s{arr};
        ASSERT_IS_TRUE((s.size() == 5 && s.data() == &arr[0]));
    }

    {
        Span<int, 5> s{arr};
        ASSERT_IS_TRUE((s.size() == 5 && s.data() == &arr[0]));
    }

    int arr2d[2][3] = {1, 2, 3, 4, 5, 6};

#ifdef CONFIRM_COMPILATION_ERRORS
    {
        Span<int, 6> s{arr};
    }

    {
        Span<int, 0> s{arr};
        ASSERT_IS_TRUE((s.size() == 0 && s.data() == &arr[0]));
    }

    {
        Span<int> s{arr2d};
        ASSERT_IS_TRUE((s.size() == 6 && s.data() == &arr2d[0][0]));
        ASSERT_IS_TRUE((s[0] == 1 && s[5] == 6));
    }

    {
        Span<int, 0> s{arr2d};
        ASSERT_IS_TRUE((s.size() == 0 && s.data() == &arr2d[0][0]));
    }

    {
        Span<int, 6> s{arr2d};
    }
#endif
    {
        Span<int[3]> s{&(arr2d[0]), 1};
        ASSERT_IS_TRUE((s.size() == 1 && s.data() == &arr2d[0]));
    }

    int arr3d[2][3][2] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

#ifdef CONFIRM_COMPILATION_ERRORS
    {
        Span<int> s{arr3d};
        ASSERT_IS_TRUE((s.size() == 12 && s.data() == &arr3d[0][0][0]));
        ASSERT_IS_TRUE((s[0] == 1 && s[11] == 12));
    }

    {
        Span<int, 0> s{arr3d};
        ASSERT_IS_TRUE((s.size() == 0 && s.data() == &arr3d[0][0][0]));
    }

    {
        Span<int, 11> s{arr3d};
    }

    {
        Span<int, 12> s{arr3d};
        ASSERT_IS_TRUE((s.size() == 12 && s.data() == &arr3d[0][0][0]));
        ASSERT_IS_TRUE((s[0] == 1 && s[5] == 6));
    }
#endif
    {
        Span<int[3][2]> s{&arr3d[0], 1};
        ASSERT_IS_TRUE((s.size() == 1 && s.data() == &arr3d[0]));
    }

    {
        auto s = make_span(arr);
        ASSERT_IS_TRUE((s.size() == 5 && s.data() == &arr[0]));
    }

    {
        auto s = make_span(&(arr2d[0]), 1);
        ASSERT_IS_TRUE((s.size() == 1 && s.data() == &arr2d[0]));
    }

    {
        auto s = make_span(&arr3d[0], 1);
        ASSERT_IS_TRUE((s.size() == 1 && s.data() == &arr3d[0]));
    }
}

void from_dynamic_array_constructor()
{
    double(*arr)[3][4] = new double[100][3][4];

    {
        Span<double> s(&arr[0][0][0], 10);
        ASSERT_IS_TRUE((s.size() == 10 && s.data() == &arr[0][0][0]));
    }

    {
        auto s = make_span(&arr[0][0][0], 10);
        ASSERT_IS_TRUE((s.size() == 10 && s.data() == &arr[0][0][0]));
    }

    delete[] arr;
}

void from_std_array_constructor()
{
    std::array<int, 4> arr = {1, 2, 3, 4};

    {
        Span<int> s{arr};
        ASSERT_IS_TRUE((s.size() == static_cast<ptrdiff_t>(arr.size()) && s.data() == arr.data()));

        Span<const int> cs{arr};
        ASSERT_IS_TRUE((cs.size() == static_cast<ptrdiff_t>(arr.size()) && cs.data() == arr.data()));
    }

    {
        Span<int, 4> s{arr};
        ASSERT_IS_TRUE((s.size() == static_cast<ptrdiff_t>(arr.size()) && s.data() == arr.data()));

        Span<const int, 4> cs{arr};
        ASSERT_IS_TRUE((cs.size() == static_cast<ptrdiff_t>(arr.size()) && cs.data() == arr.data()));
    }

#ifdef CONFIRM_COMPILATION_ERRORS
    {
        Span<int, 2> s{arr};
        ASSERT_IS_TRUE((s.size() == 2 && s.data() == arr.data()));

        Span<const int, 2> cs{arr};
        ASSERT_IS_TRUE((cs.size() == 2 && cs.data() == arr.data()));
    }

    {
        Span<int, 0> s{arr};
        ASSERT_IS_TRUE((s.size() == 0 && s.data() == arr.data()));

        Span<const int, 0> cs{arr};
        ASSERT_IS_TRUE((cs.size() == 0 && cs.data() == arr.data()));
    }

    {
        Span<int, 5> s{arr};
    }

    {
        auto get_an_array = []() -> std::array<int, 4> { return {1, 2, 3, 4}; };
        auto take_a_span = [](Span<int> s) { static_cast<void>(s); };
        // try to take a temporary std::array
        take_a_span(get_an_array());
    }
#endif

    {
        auto get_an_array = []() -> std::array<int, 4> { return {1, 2, 3, 4}; };
        auto take_a_span = [](Span<const int> s) { static_cast<void>(s); };
        // try to take a temporary std::array
        take_a_span(get_an_array());
    }

    {
        auto s = make_span(arr);
        ASSERT_IS_TRUE((s.size() == static_cast<ptrdiff_t>(arr.size()) && s.data() == arr.data()));
    }
}

void from_const_std_array_constructor()
{
    const std::array<int, 4> arr = {1, 2, 3, 4};

    {
        Span<const int> s{arr};
        ASSERT_IS_TRUE((s.size() == static_cast<ptrdiff_t>(arr.size()) && s.data() == arr.data()));
    }

    {
        Span<const int, 4> s{arr};
        ASSERT_IS_TRUE((s.size() == static_cast<ptrdiff_t>(arr.size()) && s.data() == arr.data()));
    }

#ifdef CONFIRM_COMPILATION_ERRORS
    {
        Span<const int, 2> s{arr};
        ASSERT_IS_TRUE((s.size() == 2 && s.data() == arr.data()));
    }

    {
        Span<const int, 0> s{arr};
        ASSERT_IS_TRUE((s.size() == 0 && s.data() == arr.data()));
    }

    {
        Span<const int, 5> s{arr};
    }
#endif

    {
        auto get_an_array = []() -> const std::array<int, 4> { return {1, 2, 3, 4}; };
        auto take_a_span = [](Span<const int> s) { static_cast<void>(s); };
        // try to take a temporary std::array
        take_a_span(get_an_array());
    }

    {
        auto s = make_span(arr);
        ASSERT_IS_TRUE((s.size() == static_cast<ptrdiff_t>(arr.size()) && s.data() == arr.data()));
    }
}

void from_std_array_const_constructor()
{
    std::array<const int, 4> arr = {1, 2, 3, 4};

    {
        Span<const int> s{arr};
        ASSERT_IS_TRUE((s.size() == static_cast<ptrdiff_t>(arr.size()) && s.data() == arr.data()));
    }

    {
        Span<const int, 4> s{arr};
        ASSERT_IS_TRUE((s.size() == static_cast<ptrdiff_t>(arr.size()) && s.data() == arr.data()));
    }

#ifdef CONFIRM_COMPILATION_ERRORS
    {
        Span<const int, 2> s{arr};
        ASSERT_IS_TRUE((s.size() == 2 && s.data() == arr.data()));
    }

    {
        Span<const int, 0> s{arr};
        ASSERT_IS_TRUE((s.size() == 0 && s.data() == arr.data()));
    }

    {
        Span<const int, 5> s{arr};
    }

    {
        Span<int, 4> s{arr};
    }
#endif

    {
        auto s = make_span(arr);
        ASSERT_IS_TRUE((s.size() == static_cast<ptrdiff_t>(arr.size()) && s.data() == arr.data()));
    }
}

void from_container_constructor()
{
    std::vector<int> v = {1, 2, 3};
    const std::vector<int> cv = v;

    {
        Span<int> s{v};
        ASSERT_IS_TRUE((s.size() == static_cast<std::ptrdiff_t>(v.size()) && s.data() == v.data()));

        Span<const int> cs{v};
        ASSERT_IS_TRUE((cs.size() == static_cast<std::ptrdiff_t>(v.size()) && cs.data() == v.data()));
    }

    std::string str = "hello";
    const std::string cstr = "hello";

    {
#ifdef CONFIRM_COMPILATION_ERRORS
        Span<char> s{str};
        ASSERT_IS_TRUE((s.size() == static_cast<std::ptrdiff_t>(str.size()) && s.data() == str.data()));
#endif
        Span<const char> cs{str};
        ASSERT_IS_TRUE((cs.size() == static_cast<std::ptrdiff_t>(str.size()) && cs.data() == str.data()));
    }

    {
#ifdef CONFIRM_COMPILATION_ERRORS
        Span<char> s{cstr};
#endif
        Span<const char> cs{cstr};
        ASSERT_IS_TRUE((cs.size() == static_cast<std::ptrdiff_t>(cstr.size()) &&
              cs.data() == cstr.data()));
    }

    {
#ifdef CONFIRM_COMPILATION_ERRORS
        auto get_temp_vector = []() -> std::vector<int> { return {}; };
        auto use_span = [](Span<int> s) { static_cast<void>(s); };
        use_span(get_temp_vector());
#endif
    }

    {
        auto get_temp_vector = []() -> std::vector<int> { return {}; };
        auto use_span = [](Span<const int> s) { static_cast<void>(s); };
        use_span(get_temp_vector());
    }

    {
#ifdef CONFIRM_COMPILATION_ERRORS
        auto get_temp_string = []() -> std::string { return {}; };
        auto use_span = [](Span<char> s) { static_cast<void>(s); };
        use_span(get_temp_string());
#endif
    }

    {
        auto get_temp_string = []() -> std::string { return {}; };
        auto use_span = [](Span<const char> s) { static_cast<void>(s); };
        use_span(get_temp_string());
    }

    {
#ifdef CONFIRM_COMPILATION_ERRORS
        auto get_temp_vector = []() -> const std::vector<int> { return {}; };
        auto use_span = [](Span<const char> s) { static_cast<void>(s); };
        use_span(get_temp_vector());
#endif
    }

    {
        auto get_temp_string = []() -> const std::string { return {}; };
        auto use_span = [](Span<const char> s) { static_cast<void>(s); };
        use_span(get_temp_string());
    }

    {
#ifdef CONFIRM_COMPILATION_ERRORS
        std::map<int, int> m;
        Span<int> s{m};
#endif
    }

    {
        auto s = make_span(v);
        ASSERT_IS_TRUE((s.size() == static_cast<std::ptrdiff_t>(v.size()) && s.data() == v.data()));

        auto cs = make_span(cv);
        ASSERT_IS_TRUE((cs.size() == static_cast<std::ptrdiff_t>(cv.size()) && cs.data() == cv.data()));
    }
}

void from_convertible_span_constructor()
{
    {
        Span<DerivedClass> avd;
        Span<const DerivedClass> avcd = avd;
        static_cast<void>(avcd);
    }

    {
    #ifdef CONFIRM_COMPILATION_ERRORS
        Span<DerivedClass> avd;
        Span<BaseClass> avb = avd;
        static_cast<void>(avb);
    #endif
    }

    #ifdef CONFIRM_COMPILATION_ERRORS
    {
        Span<int> s;
        Span<unsigned int> s2 = s;
        static_cast<void>(s2);
    }

    {
        Span<int> s;
        Span<const unsigned int> s2 = s;
        static_cast<void>(s2);
    }

    {
        Span<int> s;
        Span<short> s2 = s;
        static_cast<void>(s2);
    }
    #endif
}

void copy_move_and_assignment()
{
    Span<int> s1;
    ASSERT_IS_TRUE(s1.empty());

    int arr[] = {3, 4, 5};

    Span<const int> s2 = arr;
    ASSERT_IS_TRUE((s2.size() == 3 && s2.data() == &arr[0]));

    s2 = s1;
    ASSERT_IS_TRUE(s2.empty());

    auto get_temp_span = [&]() -> Span<int> { return {&arr[1], 2}; };
    auto use_span = [&](Span<const int> s) { ASSERT_IS_TRUE((s.size() == 2 && s.data() == &arr[1])); };
    use_span(get_temp_span());

    s1 = get_temp_span();
    ASSERT_IS_TRUE((s1.size() == 2 && s1.data() == &arr[1]));
}

void first()
{
    int arr[5] = {1, 2, 3, 4, 5};

    {
        Span<int, 5> av = arr;
        ASSERT_IS_TRUE(av.first<2>().size() == 2);
        ASSERT_IS_TRUE(av.first(2).size() == 2);
    }

    {
        Span<int, 5> av = arr;
        ASSERT_IS_TRUE(av.first<0>().size() == 0);
        ASSERT_IS_TRUE(av.first(0).size() == 0);
    }

    {
        Span<int, 5> av = arr;
        ASSERT_IS_TRUE(av.first<5>().size() == 5);
        ASSERT_IS_TRUE(av.first(5).size() == 5);
    }

    {
        Span<int, 5> av = arr;
#ifdef CONFIRM_COMPILATION_ERRORS
        ASSERT_IS_TRUE(av.first<6>().size() == 6);
        ASSERT_IS_TRUE(av.first<-1>().size() == -1);
#endif
        //CHECK_THROWS_AS(av.first(6).size(), fail_fast);
    }

    {
        Span<int> av;
        ASSERT_IS_TRUE(av.first<0>().size() == 0);
        ASSERT_IS_TRUE(av.first(0).size() == 0);
    }
}

void last()
{
    int arr[5] = {1, 2, 3, 4, 5};

    {
        Span<int, 5> av = arr;
        ASSERT_IS_TRUE(av.last<2>().size() == 2);
        ASSERT_IS_TRUE(av.last(2).size() == 2);
    }

    {
        Span<int, 5> av = arr;
        ASSERT_IS_TRUE(av.last<0>().size() == 0);
        ASSERT_IS_TRUE(av.last(0).size() == 0);
    }

    {
        Span<int, 5> av = arr;
        ASSERT_IS_TRUE(av.last<5>().size() == 5);
        ASSERT_IS_TRUE(av.last(5).size() == 5);
    }

    {
        Span<int, 5> av = arr;
#ifdef CONFIRM_COMPILATION_ERRORS
        ASSERT_IS_TRUE(av.last<6>().size() == 6);
#endif
        //CHECK_THROWS_AS(av.last(6).size(), fail_fast);
    }

    {
        Span<int> av;
        ASSERT_IS_TRUE(av.last<0>().size() == 0);
        ASSERT_IS_TRUE(av.last(0).size() == 0);
    }
}

void subspan()
{
    int arr[5] = {1, 2, 3, 4, 5};

    {
        Span<int, 5> av = arr;
        ASSERT_IS_TRUE((av.subspan<2, 2>().size() == 2));
        ASSERT_IS_TRUE(decltype(av.subspan<2, 2>())::extent == 2);
        ASSERT_IS_TRUE(av.subspan(2, 2).size() == 2);
        ASSERT_IS_TRUE(av.subspan(2, 3).size() == 3);
    }

    {
        Span<int, 5> av = arr;
        ASSERT_IS_TRUE((av.subspan<0, 0>().size() == 0));
        ASSERT_IS_TRUE(decltype(av.subspan<0,0>())::extent == 0);
        ASSERT_IS_TRUE(av.subspan(0, 0).size() == 0);
    }

    {
        Span<int, 5> av = arr;
        ASSERT_IS_TRUE((av.subspan<0, 5>().size() == 5));
        ASSERT_IS_TRUE(decltype(av.subspan<0, 5>())::extent == 5);
        ASSERT_IS_TRUE(av.subspan(0, 5).size() == 5);

//        CHECK_THROWS_AS(av.subspan(0, 6).size(), fail_fast);
//        CHECK_THROWS_AS(av.subspan(1, 5).size(), fail_fast);
    }

    {
        Span<int, 5> av = arr;
        ASSERT_IS_TRUE((av.subspan<4, 0>().size() == 0));
        ASSERT_IS_TRUE(decltype(av.subspan<4, 0>())::extent == 0);
        ASSERT_IS_TRUE(av.subspan(4, 0).size() == 0);
        ASSERT_IS_TRUE(av.subspan(5, 0).size() == 0);
//        CHECK_THROWS_AS(av.subspan(6, 0).size(), fail_fast);
    }

    {
        Span<int, 5> av = arr;
        ASSERT_IS_TRUE((av.subspan<1>().size() == 4));
        ASSERT_IS_TRUE(decltype(av.subspan<1>())::extent == 4);
    }

    {
        Span<int> av;
        ASSERT_IS_TRUE((av.subspan<0, 0>().size() == 0));
        ASSERT_IS_TRUE((decltype(av.subspan<0, 0>())::extent == 0));
        ASSERT_IS_TRUE(av.subspan(0, 0).size() == 0);
//        CHECK_THROWS_AS((av.subspan<1, 0>().size()), fail_fast);
    }

    {
        Span<int> av;
        ASSERT_IS_TRUE(av.subspan(0).size() == 0);
//        CHECK_THROWS_AS(av.subspan(1).size(), fail_fast);
    }

    {
        Span<int> av = arr;
        ASSERT_IS_TRUE(av.subspan(0).size() == 5);
        ASSERT_IS_TRUE(av.subspan(1).size() == 4);
        ASSERT_IS_TRUE(av.subspan(4).size() == 1);
        ASSERT_IS_TRUE(av.subspan(5).size() == 0);
//        CHECK_THROWS_AS(av.subspan(6).size(), fail_fast);
        const auto av2 = av.subspan(1);
        for (int i = 0; i < 4; ++i) ASSERT_IS_TRUE(av2[i] == i + 2);
    }

    {
        Span<int, 5> av = arr;
        ASSERT_IS_TRUE(av.subspan(0).size() == 5);
        ASSERT_IS_TRUE(av.subspan(1).size() == 4);
        ASSERT_IS_TRUE(av.subspan(4).size() == 1);
        ASSERT_IS_TRUE(av.subspan(5).size() == 0);
//        CHECK_THROWS_AS(av.subspan(6).size(), fail_fast);
        const auto av2 = av.subspan(1);
        for (int i = 0; i < 4; ++i) ASSERT_IS_TRUE(av2[i] == i + 2);
    }
}

void at_call()
{
    int arr[4] = {1, 2, 3, 4};

    {
        Span<int> s = arr;
        ASSERT_IS_TRUE(s.at(0) == 1);
//        CHECK_THROWS_AS(s.at(5), fail_fast);
    }

    {
        int arr2d[2] = {1, 6};
        Span<int, 2> s = arr2d;
        ASSERT_IS_TRUE(s.at(0) == 1);
        ASSERT_IS_TRUE(s.at(1) == 6);
//        CHECK_THROWS_AS(s.at(2), fail_fast);
    }
}

void operator_function_call()
{
    int arr[4] = {1, 2, 3, 4};

    {
        Span<int> s = arr;
        ASSERT_IS_TRUE(s(0) == 1);
//        CHECK_THROWS_AS(s(5), fail_fast);
    }

    {
        int arr2d[2] = {1, 6};
        Span<int, 2> s = arr2d;
        ASSERT_IS_TRUE(s(0) == 1);
        ASSERT_IS_TRUE(s(1) == 6);
//        CHECK_THROWS_AS(s(2), fail_fast);
    }
}

void iterator_default_init()
{
    Span<int>::iterator it1;
    Span<int>::iterator it2;
    ASSERT_IS_TRUE(it1 == it2);
}

void const_iterator_default_init()
{
    Span<int>::const_iterator it1;
    Span<int>::const_iterator it2;
    ASSERT_IS_TRUE(it1 == it2);
}

void iterator_conversions()
{
    Span<int>::iterator badIt;
    Span<int>::const_iterator badConstIt;
    ASSERT_IS_TRUE(badIt == badConstIt);

    int a[] = {1, 2, 3, 4};
    Span<int> s = a;

    auto it = s.begin();
    auto cit = s.cbegin();

    ASSERT_IS_TRUE(it == cit);
    ASSERT_IS_TRUE(cit == it);

    Span<int>::const_iterator cit2 = it;
    ASSERT_IS_TRUE(cit2 == cit);

    Span<int>::const_iterator cit3 = it + 4;
    ASSERT_IS_TRUE(cit3 == s.cend());
}

void iterator_comparisons()
{
    int a[] = {1, 2, 3, 4};
    {
        Span<int> s = a;
        Span<int>::iterator it = s.begin();
        auto it2 = it + 1;
        Span<int>::const_iterator cit = s.cbegin();

        ASSERT_IS_TRUE(it == cit);
        ASSERT_IS_TRUE(cit == it);
        ASSERT_IS_TRUE(it == it);
        ASSERT_IS_TRUE(cit == cit);
        ASSERT_IS_TRUE(cit == s.begin());
        ASSERT_IS_TRUE(s.begin() == cit);
        ASSERT_IS_TRUE(s.cbegin() == cit);
        ASSERT_IS_TRUE(it == s.begin());
        ASSERT_IS_TRUE(s.begin() == it);

        ASSERT_IS_TRUE(it != it2);
        ASSERT_IS_TRUE(it2 != it);
        ASSERT_IS_TRUE(it != s.end());
        ASSERT_IS_TRUE(it2 != s.end());
        ASSERT_IS_TRUE(s.end() != it);
        ASSERT_IS_TRUE(it2 != cit);
        ASSERT_IS_TRUE(cit != it2);

        ASSERT_IS_TRUE(it < it2);
        ASSERT_IS_TRUE(it <= it2);
        ASSERT_IS_TRUE(it2 <= s.end());
        ASSERT_IS_TRUE(it < s.end());
        ASSERT_IS_TRUE(it <= cit);
        ASSERT_IS_TRUE(cit <= it);
        ASSERT_IS_TRUE(cit < it2);
        ASSERT_IS_TRUE(cit <= it2);
        ASSERT_IS_TRUE(cit < s.end());
        ASSERT_IS_TRUE(cit <= s.end());

        ASSERT_IS_TRUE(it2 > it);
        ASSERT_IS_TRUE(it2 >= it);
        ASSERT_IS_TRUE(s.end() > it2);
        ASSERT_IS_TRUE(s.end() >= it2);
        ASSERT_IS_TRUE(it2 > cit);
        ASSERT_IS_TRUE(it2 >= cit);
    }
}

void begin_end()
{
    {
        int a[] = {1, 2, 3, 4};
        Span<int> s = a;

        Span<int>::iterator it = s.begin();
        Span<int>::iterator it2 = std::begin(s);
        ASSERT_IS_TRUE(it == it2);

        it = s.end();
        it2 = std::end(s);
        ASSERT_IS_TRUE(it == it2);
    }

    {
        int a[] = {1, 2, 3, 4};
        Span<int> s = a;

        auto it = s.begin();
        auto first = it;
        ASSERT_IS_TRUE(it == first);
        ASSERT_IS_TRUE(*it == 1);

        auto beyond = s.end();
        ASSERT_IS_TRUE(it != beyond);
        //CHECK_THROWS_AS(*beyond, fail_fast);

        ASSERT_IS_TRUE(beyond - first == 4);
        ASSERT_IS_TRUE(first - first == 0);
        ASSERT_IS_TRUE(beyond - beyond == 0);

        ++it;
        ASSERT_IS_TRUE(it - first == 1);
        ASSERT_IS_TRUE(*it == 2);
        *it = 22;
        ASSERT_IS_TRUE(*it == 22);
        ASSERT_IS_TRUE(beyond - it == 3);

        it = first;
        ASSERT_IS_TRUE(it == first);
        while (it != s.end()) {
            *it = 5;
            ++it;
        }

        ASSERT_IS_TRUE(it == beyond);
        ASSERT_IS_TRUE(it - beyond == 0);

        for (const auto& n : s) {
            ASSERT_IS_TRUE(n == 5);
        }
    }
}

void cbegin_cend()
{
    {
        int a[] = {1, 2, 3, 4};
        Span<int> s = a;

        Span<int>::const_iterator cit = s.cbegin();
        Span<int>::const_iterator cit2 = std::cbegin(s);
        ASSERT_IS_TRUE(cit == cit2);

        cit = s.cend();
        cit2 = std::cend(s);
        ASSERT_IS_TRUE(cit == cit2);
    }

    {
        int a[] = {1, 2, 3, 4};
        Span<int> s = a;

        auto it = s.cbegin();
        auto first = it;
        ASSERT_IS_TRUE(it == first);
        ASSERT_IS_TRUE(*it == 1);

        auto beyond = s.cend();
        ASSERT_IS_TRUE(it != beyond);
        //CHECK_THROWS_AS(*beyond, fail_fast);

        ASSERT_IS_TRUE(beyond - first == 4);
        ASSERT_IS_TRUE(first - first == 0);
        ASSERT_IS_TRUE(beyond - beyond == 0);

        ++it;
        ASSERT_IS_TRUE(it - first == 1);
        ASSERT_IS_TRUE(*it == 2);
        ASSERT_IS_TRUE(beyond - it == 3);

        int last = 0;
        it = first;
        ASSERT_IS_TRUE(it == first);
        while (it != s.cend()) {
            ASSERT_IS_TRUE(*it == last + 1);

            last = *it;
            ++it;
        }

        ASSERT_IS_TRUE(it == beyond);
        ASSERT_IS_TRUE(it - beyond == 0);
    }
}

void rbegin_rend()
{
    {
        int a[] = {1, 2, 3, 4};
        Span<int> s = a;

        auto it = s.rbegin();
        auto first = it;
        ASSERT_IS_TRUE(it == first);
        ASSERT_IS_TRUE(*it == 4);

        auto beyond = s.rend();
        ASSERT_IS_TRUE(it != beyond);
        //CHECK_THROWS_AS(*beyond, fail_fast);

        ASSERT_IS_TRUE(beyond - first == 4);
        ASSERT_IS_TRUE(first - first == 0);
        ASSERT_IS_TRUE(beyond - beyond == 0);

        ++it;
        ASSERT_IS_TRUE(it - first == 1);
        ASSERT_IS_TRUE(*it == 3);
        *it = 22;
        ASSERT_IS_TRUE(*it == 22);
        ASSERT_IS_TRUE(beyond - it == 3);

        it = first;
        ASSERT_IS_TRUE(it == first);
        while (it != s.rend()) {
            *it = 5;
            ++it;
        }

        ASSERT_IS_TRUE(it == beyond);
        ASSERT_IS_TRUE(it - beyond == 0);

        for (const auto& n : s) {
            ASSERT_IS_TRUE(n == 5);
        }
    }
}

void crbegin_crend()
{
    {
        int a[] = {1, 2, 3, 4};
        Span<int> s = a;

        auto it = s.crbegin();
        auto first = it;
        ASSERT_IS_TRUE(it == first);
        ASSERT_IS_TRUE(*it == 4);

        auto beyond = s.crend();
        ASSERT_IS_TRUE(it != beyond);
        //CHECK_THROWS_AS(*beyond, fail_fast);

        ASSERT_IS_TRUE(beyond - first == 4);
        ASSERT_IS_TRUE(first - first == 0);
        ASSERT_IS_TRUE(beyond - beyond == 0);

        ++it;
        ASSERT_IS_TRUE(it - first == 1);
        ASSERT_IS_TRUE(*it == 3);
        ASSERT_IS_TRUE(beyond - it == 3);

        it = first;
        ASSERT_IS_TRUE(it == first);
        int last = 5;
        while (it != s.crend()) {
            ASSERT_IS_TRUE(*it == last - 1);
            last = *it;

            ++it;
        }

        ASSERT_IS_TRUE(it == beyond);
        ASSERT_IS_TRUE(it - beyond == 0);
    }
}

void comparison_operators()
{
    {
        Span<int> s1;
        Span<int> s2;
        ASSERT_IS_TRUE(s1 == s2);
        ASSERT_IS_TRUE(!(s1 != s2));
        ASSERT_IS_TRUE(!(s1 < s2));
        ASSERT_IS_TRUE(s1 <= s2);
        ASSERT_IS_TRUE(!(s1 > s2));
        ASSERT_IS_TRUE(s1 >= s2);
        ASSERT_IS_TRUE(s2 == s1);
        ASSERT_IS_TRUE(!(s2 != s1));
        ASSERT_IS_TRUE(!(s2 < s1));
        ASSERT_IS_TRUE(s2 <= s1);
        ASSERT_IS_TRUE(!(s2 > s1));
        ASSERT_IS_TRUE(s2 >= s1);
    }

    {
        int arr[] = {2, 1};
        Span<int> s1 = arr;
        Span<int> s2 = arr;

        ASSERT_IS_TRUE(s1 == s2);
        ASSERT_IS_TRUE(!(s1 != s2));
        ASSERT_IS_TRUE(!(s1 < s2));
        ASSERT_IS_TRUE(s1 <= s2);
        ASSERT_IS_TRUE(!(s1 > s2));
        ASSERT_IS_TRUE(s1 >= s2);
        ASSERT_IS_TRUE(s2 == s1);
        ASSERT_IS_TRUE(!(s2 != s1));
        ASSERT_IS_TRUE(!(s2 < s1));
        ASSERT_IS_TRUE(s2 <= s1);
        ASSERT_IS_TRUE(!(s2 > s1));
        ASSERT_IS_TRUE(s2 >= s1);
    }

    {
        int arr[] = {2, 1}; // bigger

        Span<int> s1;
        Span<int> s2 = arr;

        ASSERT_IS_TRUE(s1 != s2);
        ASSERT_IS_TRUE(s2 != s1);
        ASSERT_IS_TRUE(!(s1 == s2));
        ASSERT_IS_TRUE(!(s2 == s1));
        ASSERT_IS_TRUE(s1 < s2);
        ASSERT_IS_TRUE(!(s2 < s1));
        ASSERT_IS_TRUE(s1 <= s2);
        ASSERT_IS_TRUE(!(s2 <= s1));
        ASSERT_IS_TRUE(s2 > s1);
        ASSERT_IS_TRUE(!(s1 > s2));
        ASSERT_IS_TRUE(s2 >= s1);
        ASSERT_IS_TRUE(!(s1 >= s2));
    }

    {
        int arr1[] = {1, 2};
        int arr2[] = {1, 2};
        Span<int> s1 = arr1;
        Span<int> s2 = arr2;

        ASSERT_IS_TRUE(s1 == s2);
        ASSERT_IS_TRUE(!(s1 != s2));
        ASSERT_IS_TRUE(!(s1 < s2));
        ASSERT_IS_TRUE(s1 <= s2);
        ASSERT_IS_TRUE(!(s1 > s2));
        ASSERT_IS_TRUE(s1 >= s2);
        ASSERT_IS_TRUE(s2 == s1);
        ASSERT_IS_TRUE(!(s2 != s1));
        ASSERT_IS_TRUE(!(s2 < s1));
        ASSERT_IS_TRUE(s2 <= s1);
        ASSERT_IS_TRUE(!(s2 > s1));
        ASSERT_IS_TRUE(s2 >= s1);
    }

    {
        int arr[] = {1, 2, 3};

        Span<int> s1 = {&arr[0], 2}; // shorter
        Span<int> s2 = arr;          // longer

        ASSERT_IS_TRUE(s1 != s2);
        ASSERT_IS_TRUE(s2 != s1);
        ASSERT_IS_TRUE(!(s1 == s2));
        ASSERT_IS_TRUE(!(s2 == s1));
        ASSERT_IS_TRUE(s1 < s2);
        ASSERT_IS_TRUE(!(s2 < s1));
        ASSERT_IS_TRUE(s1 <= s2);
        ASSERT_IS_TRUE(!(s2 <= s1));
        ASSERT_IS_TRUE(s2 > s1);
        ASSERT_IS_TRUE(!(s1 > s2));
        ASSERT_IS_TRUE(s2 >= s1);
        ASSERT_IS_TRUE(!(s1 >= s2));
    }

    {
        int arr1[] = {1, 2}; // smaller
        int arr2[] = {2, 1}; // bigger

        Span<int> s1 = arr1;
        Span<int> s2 = arr2;

        ASSERT_IS_TRUE(s1 != s2);
        ASSERT_IS_TRUE(s2 != s1);
        ASSERT_IS_TRUE(!(s1 == s2));
        ASSERT_IS_TRUE(!(s2 == s1));
        ASSERT_IS_TRUE(s1 < s2);
        ASSERT_IS_TRUE(!(s2 < s1));
        ASSERT_IS_TRUE(s1 <= s2);
        ASSERT_IS_TRUE(!(s2 <= s1));
        ASSERT_IS_TRUE(s2 > s1);
        ASSERT_IS_TRUE(!(s1 > s2));
        ASSERT_IS_TRUE(s2 >= s1);
        ASSERT_IS_TRUE(!(s1 >= s2));
    }
}

void fixed_size_conversions()
{
    int arr[] = {1, 2, 3, 4};

    // converting to an Span from an equal size array is ok
    Span<int, 4> s4 = arr;
    ASSERT_IS_TRUE(s4.size() == 4);

    // converting to dynamic_range is always ok
    {
        Span<int> s = s4;
        ASSERT_IS_TRUE(s.size() == s4.size());
        static_cast<void>(s);
    }

// initialization or assignment to static Span that REDUCES size is NOT ok
#ifdef CONFIRM_COMPILATION_ERRORS
    {
        Span<int, 2> s = arr;
    }
    {
        Span<int, 2> s2 = s4;
        static_cast<void>(s2);
    }
#endif

    // even when done dynamically
    {
        Span<int> s = arr;
        auto f = [&]() {
            Span<int, 2> s2 = s;
            static_cast<void>(s2);
        };
//        CHECK_THROWS_AS(f(), fail_fast);
    }

    // but doing so explicitly is ok

    // you can convert statically
    {
        const Span<int, 2> s2 = {arr, 2};
        static_cast<void>(s2);
    }
    {
        const Span<int, 1> s1 = s4.first<1>();
        static_cast<void>(s1);
    }

    // ...or dynamically
    {
        // NB: implicit conversion to Span<int,1> from Span<int>
        Span<int, 1> s1 = s4.first(1);
        static_cast<void>(s1);
    }

    // initialization or assignment to static Span that requires size INCREASE is not ok.
    int arr2[2] = {1, 2};

#ifdef CONFIRM_COMPILATION_ERRORS
    {
        Span<int, 4> s3 = arr2;
    }
    {
        Span<int, 2> s2 = arr2;
        Span<int, 4> s4a = s2;
    }
#endif
    {
        auto f = [&]() {
            Span<int, 4> _s4 = {arr2, 2};
            static_cast<void>(_s4);
        };
//        CHECK_THROWS_AS(f(), fail_fast);
    }

    // this should fail - we are trying to assign a small dynamic Span to a fixed_size larger one
    Span<int> av = arr2;
    auto f = [&]() {
        Span<int, 4> _s4 = av;
        static_cast<void>(_s4);
    };
//    CHECK_THROWS_AS(f(), fail_fast);
}

void interop_with_std_regex()
{
    char lat[] = {'1', '2', '3', '4', '5', '6', 'E', 'F', 'G'};
    Span<char> s = lat;
    const auto f_it = s.begin() + 7;

    std::match_results<Span<char>::iterator> match;

    std::regex_match(s.begin(), s.end(), match, std::regex(".*"));
    ASSERT_IS_TRUE(match.ready());
    ASSERT_IS_TRUE(!match.empty());
    ASSERT_IS_TRUE(match[0].matched);
    ASSERT_IS_TRUE(match[0].first == s.begin());
    ASSERT_IS_TRUE(match[0].second == s.end());

    std::regex_search(s.begin(), s.end(), match, std::regex("F"));
    ASSERT_IS_TRUE(match.ready());
    ASSERT_IS_TRUE(!match.empty());
    ASSERT_IS_TRUE(match[0].matched);
    ASSERT_IS_TRUE(match[0].first == f_it);
    ASSERT_IS_TRUE(match[0].second == (f_it + 1));
}

void default_constructible()
{
    ASSERT_IS_TRUE((std::is_default_constructible<Span<int>>::value));
    ASSERT_IS_TRUE((std::is_default_constructible<Span<int, 0>>::value));
    ASSERT_IS_TRUE((!std::is_default_constructible<Span<int, 42>>::value));
}

void from_eigen_constructor()
{
    Eigen::Vector3d vec;
    vec[0] = 1;
    vec[1] = 2;
    vec[2] = 3;

    Span<double> s(vec);

    ASSERT_IS_TRUE(vec.size() == s.size());
    for(size_t i=0; i < vec.size(); i++) {
        ASSERT_IS_TRUE(vec[i] == s[i]);
    }

    auto other = make_span(vec);
    ASSERT_IS_TRUE(vec.size() == other.size());
    for(size_t i=0; i < vec.size(); i++) {
        ASSERT_IS_TRUE(vec[i] == other[i]);
    }
}

int main(){
    default_constructor();
    size_optimization();
    from_nullptr_size_constructor();
    from_pointer_length_constructor();
    from_pointer_pointer_constructor();
    from_array_constructor();
    from_dynamic_array_constructor();
    from_std_array_constructor();
    from_const_std_array_constructor();
    from_std_array_const_constructor();
    from_container_constructor();
    from_convertible_span_constructor();
    copy_move_and_assignment();
    first();
    last();
    subspan();
    at_call();
    operator_function_call();
    iterator_default_init();
    const_iterator_default_init();
    iterator_conversions();
    iterator_comparisons();
    begin_end();
    cbegin_cend();
    rbegin_rend();
    crbegin_crend();
    comparison_operators();
    fixed_size_conversions();
    interop_with_std_regex();
    default_constructible();
    from_eigen_constructor();

    return EXIT_SUCCESS;
}
