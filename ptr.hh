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
// ScopedPtr
template<class T, bool IsArray = false>
class ScopedPtr
{
protected:
	mutable T* m_ptr;
public:
	ScopedPtr(T* ptr = 0) : m_ptr(ptr) {}
	~ScopedPtr() { PtrPolicy<T, IsArray>::Delete(m_ptr); }

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
	ScopedPtr& operator=(T* ptr);
	ScopedPtr& operator=(const ScopedPtr& other);
	ScopedPtr(const ScopedPtr&);
};

template<class T, bool IsArray>
ScopedPtr<T, IsArray>& ScopedPtr<T, IsArray>::operator=(T* ptr)
{
	if(ptr != m_ptr)
	{
		PtrPolicy<T, IsArray>::Delete(m_ptr);
		m_ptr = ptr;
	}
	return *this;
}

template<class T, bool IsArray>
ScopedPtr<T, IsArray>& ScopedPtr<T, IsArray>::operator=(const ScopedPtr& other)
{
	ASSERT(m_ptr != other.m_ptr); // If this fires, then the object is probably going to be deleted twice anyways
	PtrPolicy<T, IsArray>::Delete(m_ptr);
	m_ptr = other.m_ptr;
	other.m_ptr = 0;
}

template<class T, bool IsArray>
ScopedPtr<T, IsArray>::ScopedPtr(const ScopedPtr<T, IsArray> & other) : m_ptr(0)
{
	m_ptr = other.m_ptr;
	other.m_ptr = 0;
}

////////////////////////////////////////////////////////////////////////////////
// ScopedPtrAry

template< class T >
struct ScopedPtrAry
{
	typedef ScopedPtr<T, true> Type;
};

