#pragma once

#include <cstdio>
#include <cstdlib>
#include "assert.hh"

template< class T, int N>
char (&array_size_helper(T (&)[N]))[N];
#define ARRAY_SIZE(ary) (sizeof(array_size_helper((ary))))

template< class T > struct IsFloatType { enum { val = 0 }; };
template<> struct IsFloatType<float> { enum { val = 1 }; };
template<> struct IsFloatType<double> { enum { val = 1 }; };


