#pragma once

enum PlyDataType
{
	PLYDATATYPE_CHAR = 0,
	PLYDATATYPE_UCHAR,
	PLYDATATYPE_SHORT,
	PLYDATATYPE_USHORT,
	PLYDATATYPE_INT,
	PLYDATATYPE_UINT,
	PLYDATATYPE_FLOAT,
	PLYDATATYPE_DOUBLE,
	PLYDATATYPE_LIST,

	PLYDATATYPE_NUM,
};

template< PlyDataType Val > struct PlyDataEnumToType {};
#define DEFINEPLYTYPE(enumval, tp) \
template<> struct PlyDataEnumToType<enumval> { typedef tp Type; };

DEFINEPLYTYPE(PLYDATATYPE_CHAR, char);
DEFINEPLYTYPE(PLYDATATYPE_SHORT, short);
DEFINEPLYTYPE(PLYDATATYPE_INT, int);
DEFINEPLYTYPE(PLYDATATYPE_UCHAR, unsigned char);
DEFINEPLYTYPE(PLYDATATYPE_USHORT, unsigned short);
DEFINEPLYTYPE(PLYDATATYPE_UINT, unsigned int);
DEFINEPLYTYPE(PLYDATATYPE_FLOAT, float);
DEFINEPLYTYPE(PLYDATATYPE_DOUBLE, double);

class PlyData
{
	class PlyDataValue
	{
		char m_data[sizeof(double)];
	public:
		PlyDataValue() : m_data() { }
		template< class T>
		PlyDataValue(const T& init) {
			COMPILE_ASSERT(sizeof(T) <= sizeof(double));
			T* ptr = reinterpret_cast<T*>(m_data);
			*ptr = init;
		}
		template< class T >
		T As() const {
			return *reinterpret_cast<const T*>(m_data);
		}	
	};
		
	class PlyPropertyBase
	{
		std::string m_name;
		PlyDataType m_type;
	public:
		PlyPropertyBase(const char* name, PlyDataType type) 
			: m_name(name)
			, m_type(type)
		{}

		virtual ~PlyPropertyBase() = 0 ;
		PlyDataType GetType() const { return m_type; }
		const char* GetName() const { return m_name.c_str(); }

		virtual bool Parse(const char* line, char *& next) = 0;

		template< class T >
		T GetValue(int index) const;

		template< class T >
		T GetListValue(int listId, int index) const;
		
		virtual int GetCount() const { return 0; }
		virtual int GetListSize(int listIndex) const { return 0; }
	protected:
		template<class ClassValueType, class T>
		static bool ValidateType(const T& value)
		{
			if(IsFloatType<ClassValueType>::val)
			{
				if(value < -std::numeric_limits<ClassValueType>::max() ||
						value > std::numeric_limits<ClassValueType>::max())
				{
					printf("Value out of range!\n");
					return false;
				}
			}
			else
			{
				if(value < std::numeric_limits<ClassValueType>::min() ||
						value > std::numeric_limits<ClassValueType>::max())
				{
					printf("Value out of range!\n");
					return false;
				}
			}
			return true;
		}

		virtual PlyDataValue GetValueRaw(int index) const { return PlyDataValue(); }
		virtual PlyDataValue GetListValueRaw(int listIndex, int itemIndex) const { return PlyDataValue(); }
	};

	template<class T>
	class PlyProperty : public PlyPropertyBase
	{
		std::vector<T> m_values;
	public:
		PlyProperty(const char *name, PlyDataType type) : PlyPropertyBase(name, type) {}
		virtual bool Parse(const char* line, char *& next) 
		{
			if(GetType() < PLYDATATYPE_FLOAT)
			{
				long value = strtol(line, &next, 10);
				if(!ValidateType<T>(value))
					return false;
				Append(value);
			}
			else
			{
				double value = strtod(line, &next);
				if(!ValidateType<T>(value))
					return false;
				Append(value);
			}
			
			while(*next && isspace(*next)) ++next;

			return true;
		}

		virtual int GetCount() const { return m_values.size(); }
	private:
		virtual PlyDataValue GetValueRaw(int index) const;
		void Append(const T& value) 
		{
			m_values.push_back(value); 
		}
	};

	template<class T>
	class PlyListProperty : public PlyPropertyBase
	{
		std::vector< std::vector<T>* > m_values;
		PlyDataType m_listType;
	public:
		PlyListProperty(const char* name, PlyDataType type, PlyDataType m_listType) : PlyPropertyBase(name, type) {}
		~PlyListProperty() {
			for(int i = 0, c = m_values.size(); i < c; ++i)
				delete m_values[i];
		}
	
		virtual bool Parse(const char* line, char *& next) 
		{
			long count = strtol(line, &next, 10);
			line = next;

			int listId = AppendList();
			
			if(m_listType < PLYDATATYPE_FLOAT)
			{
				for(int i = 0; i < int(count); ++i)
				{
					long value = strtol(line, &next, 10);
					if(!ValidateType<T>(value))
						return false;
					Append(listId, value);
					line = next;
				}
			}
			else
			{
				for(int i = 0; i < int(count); ++i)
				{
					double value = strtod(line, &next);
					if(!ValidateType<T>(value))
						return false;
					Append(listId, value);
					line = next;
				}
			}

			while(*next && isspace(*next)) ++next;
			return true;
		}
		
		virtual int GetCount() const ;
		virtual int GetListSize(int listIndex) const;
	protected:
		virtual PlyDataValue GetListValueRaw(int listIndex, int itemIndex) const ;
	private:
		int AppendList() {
			int list = m_values.size();
			std::vector<T>* newVal = new std::vector<T>;
			m_values.push_back(newVal);
			return list;
		}

		void Append(int list, const T& value)
		{
			m_values[list]->push_back(value);
		}
	};

	static PlyPropertyBase * AllocateProperty(const char * name, PlyDataType type, PlyDataType listType);

	class PlyElement
	{
	public:
		PlyElement(const char* name, int size) 
			: m_name(name)
			, m_count(size)
		{
			m_properties.reserve(size);
		}

		~PlyElement() {
			for(int i = 0, c = m_properties.size(); i < c; ++i)
				delete m_properties[i];
		}

		inline const char* GetName() const { return m_name.c_str(); }
		inline int GetCount() const { return m_count; }
		inline void AddProperty(PlyPropertyBase* property) { m_properties.push_back(property); }
		inline int NumProperties() const { return m_properties.size(); }
		PlyPropertyBase* GetProperty(int index) const { return m_properties[index]; }
	private:
		std::string m_name;
		int m_count;
		std::vector<PlyPropertyBase*> m_properties;
	};

	std::vector<PlyElement*> m_elements;

public:
	class ElementReader
	{
		PlyElement *m_element;
	public:
		ElementReader() : m_element(0) {}
		ElementReader(const ElementReader& o) : m_element(o.m_element) {}
		ElementReader(PlyElement* element) : m_element(element) {}
		int GetPropertyId(const char* name);

		bool Valid() const { return m_element != 0; }
		const char * GetName() const { return m_element->GetName(); }
		int GetCount() const { return m_element->GetCount(); }

		template<class T>
		T GetPropertyValue(int propId, int elementIndex) const
		{
			PlyPropertyBase * prop = m_element->GetProperty(propId);
			if(prop->GetType() == PLYDATATYPE_LIST)
				return T(0);
			else
				return prop->GetValue<T>(elementIndex);
		}

		template<class T>
		T GetPropertyListValue(int propId, int elementIndex, int listIndex) const
		{
			PlyPropertyBase * prop = m_element->GetProperty(propId);
			if(prop->GetType() == PLYDATATYPE_LIST)
				return prop->GetListValue<T>(elementIndex, listIndex);
			else
				return T(0);
		}

		int GetListSize(int propId, int elementIndex)
		{
			PlyPropertyBase * prop = m_element->GetProperty(propId);
			return prop->GetListSize(elementIndex);
		}

	};

	PlyData();
	~PlyData();

	void Clear();
	bool Read(FILE* fp);

	int NumElements() const { return m_elements.size(); }
	ElementReader GetElement(int i);
	ElementReader GetElement(const char* name);
private:

	bool MatchHeader(FILE* fp);
	bool MatchFormat(const char* line);
	bool MatchElement(FILE* fp, const char* line);
	bool MatchProperty(FILE* fp, const char* line);
	bool MatchData(FILE* fp);
};


template< class T >
inline T PlyData::PlyPropertyBase::GetValue(int index) const
{
	PlyDataValue val = GetValueRaw(index);
	return val.As<T>();
}

template< class T >
inline T PlyData::PlyPropertyBase::GetListValue(int listId, int index) const
{
	PlyDataValue val = GetListValueRaw(listId, index);
	return val.As<T>();
}

template< class T >
inline PlyData::PlyDataValue PlyData::PlyProperty<T>::GetValueRaw(int index) const
{
	return PlyDataValue(m_values[index]);
}

template< class T >
inline PlyData::PlyDataValue PlyData::PlyListProperty<T>::GetListValueRaw(int listIndex, int itemIndex) const
{
	std::vector<T> * pList = m_values[listIndex];
	return PlyDataValue(pList->at(itemIndex));
}

template< class T> 
inline int PlyData::PlyListProperty<T>::GetCount() const 
{
	return m_values.size();
}

template< class T >
inline int PlyData::PlyListProperty<T>::GetListSize(int listIndex) const
{
	std::vector<T> * pList = m_values[listIndex];
	return pList->size();
}

