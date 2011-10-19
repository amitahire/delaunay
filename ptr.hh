#pragma once

////////////////////////////////////////////////////////////////////////////////
// Ptr Helpers
template< typename T, bool IsArray >
struct PtrPolicy
{
	static void Delete(T* ptr) {delete ptr;}
	static void At(T* ptr, int index) { ASSERT(false); }
};

template< typename T >
struct PtrPolicy<T, true>
{
	static void Delete(T* ptr) { delete[] ptr; }
	static T& At(T* ptr, int index) { return ptr[index]; }
};

////////////////////////////////////////////////////////////////////////////////
// ScopedPtrImpl
template<class T, bool IsArray = false>
class ScopedPtrImpl
{
protected:
	mutable T* m_ptr;
public:
	ScopedPtrImpl(T* ptr = 0) : m_ptr(ptr) {}
	~ScopedPtrImpl() { PtrPolicy<T, IsArray>::Delete(m_ptr); }

	operator T* () { return m_ptr; }
	operator const T* () const { return m_ptr; }

	T& operator *() { return *m_ptr; }
	const T& operator *() const { return *m_ptr; }

	T* operator ->() { return m_ptr; }
	const T* operator ->() const { return m_ptr; }

	T& operator[](int idx) {
		return PtrPolicy<T, IsArray>::At(m_ptr, idx);
	}

	const T& operator[](int idx) const {
		return PtrPolicy<T, IsArray>::At(m_ptr, idx);
	}
	ScopedPtrImpl& operator=(T* ptr);
	ScopedPtrImpl& operator=(const ScopedPtrImpl& other);
	ScopedPtrImpl(const ScopedPtrImpl&);
};

template<class T, bool IsArray>
ScopedPtrImpl<T, IsArray>& ScopedPtrImpl<T, IsArray>::operator=(T* ptr)
{
	if(ptr != m_ptr)
	{
		PtrPolicy<T, IsArray>::Delete(m_ptr);
		m_ptr = ptr;
	}
	return *this;
}

template<class T, bool IsArray>
ScopedPtrImpl<T, IsArray>& ScopedPtrImpl<T, IsArray>::operator=(const ScopedPtrImpl& other)
{
	ASSERT(m_ptr != other.m_ptr); // If this fires, then the object is probably going to be deleted twice anyways
	PtrPolicy<T, IsArray>::Delete(m_ptr);
	m_ptr = other.m_ptr;
	other.m_ptr = 0;
}

template<class T, bool IsArray>
ScopedPtrImpl<T, IsArray>::ScopedPtrImpl(const ScopedPtrImpl<T, IsArray> & other) : m_ptr(0)
{
	m_ptr = other.m_ptr;
	other.m_ptr = 0;
}
////////////////////////////////////////////////////////////////////////////////
// ScopedPtr
template< class T>
struct ScopedPtr
{
	typedef ScopedPtrImpl<T, false> Type;
};

////////////////////////////////////////////////////////////////////////////////
// ScopedPtrAry

template< class T >
struct ScopedPtrAry
{
	typedef ScopedPtrImpl<T, true> Type;
};

