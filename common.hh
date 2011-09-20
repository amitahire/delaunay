#ifndef INCLUDED_common_HH
#define INCLUDED_common_HH

#include <cstdio>
#include <cstdlib>
#include "assert.hh"

template< class T, int N>
char (&array_size_helper(T (&)[N]))[N];
#define ARRAY_SIZE(ary) (sizeof(array_size_helper((ary))))

#endif


