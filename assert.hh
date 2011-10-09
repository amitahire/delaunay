#pragma once

#ifdef DEBUG
#include <cstdio>
#include <cstdlib>
#endif

#if defined(LINUX)
#define DEBUGBREAK() asm("int $3") ;
#elif defined(WIN32)
#define DEBUGBREAK() __debugbreak();
#endif

#define VERIFY(condition) do { if(!(condition)) { DEBUGBREAK() } } while(0)

#ifdef DEBUG
#define LABASSERT(condition,message) do { if(!(condition)) { fprintf(stderr, "%s\n", message); DEBUGBREAK(); } } while(0)
#define ASSERT(condition) do { if(!(condition)) { DEBUGBREAK() } } while(0)
#else
#define LABASSERT(condition,message) do { (void)sizeof(condition); } while(0)
#define ASSERT(condition) do { (void)sizeof(condition); } while(0)
#endif

namespace Assert
{
    template <bool> struct COMPILE_ASSERT_FAILURE; // undefined if not true
    template <> struct COMPILE_ASSERT_FAILURE<true> { enum { value = 1 } ; };
    template <int x> struct compile_assert_test{};
}
#define JOIN2(a,b) a##b
#define JOIN(a,b) JOIN2(a,b)
#define impl_COMPILE_ASSERT( condition, id ) \
    typedef ::Assert::compile_assert_test< sizeof( ::Assert::COMPILE_ASSERT_FAILURE< ((bool) (condition)) >::value ) > \
	JOIN(__compile_assert_typedef_, id)
#define COMPILE_ASSERT( condition ) impl_COMPILE_ASSERT(condition, JOIN(__COUNTER__, __LINE__))


