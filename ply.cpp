#include "common.hh"
#include <strings.h>
#include "ply.hh"

////////////////////////////////////////////////////////////////////////////////
// Helpers

static PlyDataType DataTypeFromString(const char* str) 
{
	static const char* s_typeStr[] =
	{
		"char",
		"uchar",
		"short",
		"ushort",
		"int",
		"uint",
		"float",
		"double",
		"list",
	};
	COMPILE_ASSERT( ARRAY_SIZE(s_typeStr) == PLYDATATYPE_NUM );

	for(int i = 0, c = ARRAY_SIZE(s_typeStr); i < c; ++i)
	{
		if(strcasecmp(s_typeStr[i], str) == 0)
			return PlyDataType(i);
	}
	return PLYDATATYPE_NUM;
}

////////////////////////////////////////////////////////////////////////////////
// PlyProperty impl
PlyData::PlyPropertyBase::~PlyPropertyBase() {}

////////////////////////////////////////////////////////////////////////////////
// PlyData impl
	
PlyData::PlyPropertyBase * PlyData::AllocateProperty(const char * name, 
	PlyDataType type, PlyDataType listType) {
	switch(type)
	{
		case PLYDATATYPE_CHAR:
			return new PlyProperty<PlyDataEnumToType< PLYDATATYPE_CHAR >::Type >(name, type);
		case PLYDATATYPE_UCHAR:
			return new PlyProperty<PlyDataEnumToType< PLYDATATYPE_UCHAR >::Type >(name, type);
		case PLYDATATYPE_SHORT:
			return new PlyProperty<PlyDataEnumToType< PLYDATATYPE_SHORT >::Type >(name, type);
		case PLYDATATYPE_USHORT:
			return new PlyProperty<PlyDataEnumToType< PLYDATATYPE_USHORT >::Type >(name, type);
		case PLYDATATYPE_INT:
			return new PlyProperty<PlyDataEnumToType< PLYDATATYPE_INT >::Type >(name, type);
		case PLYDATATYPE_UINT:
			return new PlyProperty<PlyDataEnumToType< PLYDATATYPE_UINT >::Type >(name, type);
		case PLYDATATYPE_FLOAT:
			return new PlyProperty<PlyDataEnumToType< PLYDATATYPE_FLOAT >::Type >(name, type);
		case PLYDATATYPE_DOUBLE:
			return new PlyProperty<PlyDataEnumToType< PLYDATATYPE_DOUBLE >::Type >(name, type);
		case PLYDATATYPE_LIST:
			{
				switch(listType)
				{
					case PLYDATATYPE_CHAR:
						return new PlyListProperty<PlyDataEnumToType< PLYDATATYPE_CHAR >::Type >(name, type, listType);
					case PLYDATATYPE_UCHAR:
						return new PlyListProperty<PlyDataEnumToType< PLYDATATYPE_UCHAR >::Type >(name, type, listType);
					case PLYDATATYPE_SHORT:
						return new PlyListProperty<PlyDataEnumToType< PLYDATATYPE_SHORT >::Type >(name, type, listType);
					case PLYDATATYPE_USHORT:
						return new PlyListProperty<PlyDataEnumToType< PLYDATATYPE_USHORT >::Type >(name, type, listType);
					case PLYDATATYPE_INT:
						return new PlyListProperty<PlyDataEnumToType< PLYDATATYPE_INT >::Type >(name, type, listType);
					case PLYDATATYPE_UINT:
						return new PlyListProperty<PlyDataEnumToType< PLYDATATYPE_UINT >::Type >(name, type, listType);
					case PLYDATATYPE_FLOAT:
						return new PlyListProperty<PlyDataEnumToType< PLYDATATYPE_FLOAT >::Type >(name, type, listType);
					case PLYDATATYPE_DOUBLE:
						return new PlyListProperty<PlyDataEnumToType< PLYDATATYPE_DOUBLE >::Type >(name, type, listType);
					default:break;
				}
			}
			break;
		default:
			break;
	};
	return 0;
}

PlyData::PlyData()
	: m_elements()
{
}

PlyData::~PlyData()
{
	Clear();
}

void PlyData::Clear()
{
	for(int i = 0, c = m_elements.size(); i < c; ++i)
		delete m_elements[i];
	m_elements.clear();
}

bool PlyData::Read(FILE* fp)
{
	if(!MatchHeader(fp))
		return false;
	if(!MatchData(fp))
		return false;
	return true;
}

static bool NextToken(const char* line, const char*& curLine, char *tokenStr, int maxLen)
{
	while(*line && isspace(*line)) ++line;
	const char* cursor = line;
	int pos = 0;
	while(*cursor && !isspace(*cursor) && pos < maxLen - 1)
	{
		tokenStr[pos++] = *(cursor++);
	}
	tokenStr[pos] = '\0';
	curLine = cursor;
	return pos > 0;
}

static bool NextLine(FILE* fp, char* line, int maxLen, const char*& curLine)
{
	curLine = line;
	if(0 == fgets(line, maxLen - 1, fp))
		return false;
	return true;
}

bool PlyData::MatchHeader(FILE* fp)
{
	char token[64];
	char line[256];
	const char* cursor;

	if(!NextLine(fp, line, ARRAY_SIZE(line), cursor))
		return false;
	
	// Magic
	if(!NextToken(line, cursor, token, ARRAY_SIZE(token)))
		return false;
	if(strcasecmp(token, "ply") != 0)
		return false;

	// Read header
	do
	{
		if(!NextLine(fp, line, ARRAY_SIZE(line), cursor))
		{
			printf("Error reading header.\n");
			return false;
		}
	
		if(!NextToken(line, cursor, token, ARRAY_SIZE(token)))
			return false;

		if(strcasecmp(token, "format") == 0)
		{
			if(!MatchFormat(cursor))
				return false;
		}
		else if(strcasecmp(token, "comment") == 0)
		{
			// nothing
		}
		else if(strcasecmp(token, "element") == 0)
		{
			if(!MatchElement(cursor))
				return false;
		}
		else if(strcasecmp(token, "property") == 0)
		{
			if(!MatchProperty(cursor))
				return false;
		}
		else if(strcasecmp(token, "end_header") == 0)
		{
			break;
		}
		else
		{
			printf("Unrecognized header line: %s\n", token);
			return false;
		}
	} while(true);
	return true;
}

bool PlyData::MatchFormat(const char* line)
{
	const char* cursor;
	char token[64];
	if(!NextToken(line, cursor, token, ARRAY_SIZE(token)))
		return false;

	if(strcasecmp("binary", token) == 0)
	{
		printf("Binary PLY not supported\n");
		return false;
	} 
	else if(strcasecmp("ascii", token) != 0)
	{
		printf("Unsupported format: %s\n", token);
		return false;
	} 

	line = cursor;
	if(!NextToken(line, cursor, token, ARRAY_SIZE(token)))
		return false;

	char* major = token;
	char* minor = major;
	while(*minor != '.' && *minor) ++minor;
	if(*minor == '.') {
		*minor = '\0';
		++minor;
	}

	int majorNum = atoi(major);
	int minorNum = atoi(minor);

	if(majorNum != 1)
	{
		printf("File must be at least version 1\n");
		return false;
	}

	if(minorNum > 0)
	{
		printf("WARN: This loads version 1.0 files, but this is a version %d.%d file.\n", majorNum, minorNum);
	}

	return true;
}

bool PlyData::MatchElement(const char *line)
{
	const char *cursor = line;
	char token[64];
	if(!NextToken(line, cursor, token, ARRAY_SIZE(token)))
	{
		printf("Failed to match element name.\n");
		return false;
	}

	line = cursor;
	
	char countToken[64];
	if(!NextToken(line, cursor, countToken, ARRAY_SIZE(countToken)))
	{
		printf("Failed to match element count for %s\n", token);
		return false;
	}
	
	m_elements.push_back(new PlyElement(token, atoi(countToken)));

	return true;

}

bool PlyData::MatchProperty(const char *line)
{
	PlyElement *element = m_elements.back();
	const char* cursor = line;	
	char token[64];
	if(!NextToken(line, cursor, token, ARRAY_SIZE(token)))
	{
		printf("Could not property parse type.\n");
		return false;
	}

	line = cursor;

	PlyDataType dataType = DataTypeFromString(token);
	PlyDataType listType = PLYDATATYPE_NUM;
	if(dataType == PLYDATATYPE_LIST)
	{
		if(!NextToken(line, cursor, token, ARRAY_SIZE(token)))
		{
			printf("Could not parse list length type.\n");
			return false;
		}
		// Yeah i'm just going to ignore the list type for ascii.

		line = cursor;
		
		if(!NextToken(line, cursor, token, ARRAY_SIZE(token)))
		{
			printf("Could not parse list value type.\n");
			return false;
		}
		listType = DataTypeFromString(token);

		line = cursor;
	}
		
	if(!NextToken(line, cursor, token, ARRAY_SIZE(token)))
	{
		printf("Could not parse property name.\n");
		return false;
	}
		
	element->AddProperty(AllocateProperty(token, dataType, listType));

	return true;
}

bool PlyData::MatchData(FILE* fp)
{
	char line[256];
	char *cursor = 0, *next = 0;
	for(int iElement = 0, cElement = m_elements.size(); iElement < cElement; ++iElement)
	{
		PlyElement* element = m_elements[iElement];

		for(int iLine = 0, cLine = element->GetCount(); iLine < cLine; ++iLine)
		{
			if( fgets(line, ARRAY_SIZE(line) - 1, fp) == NULL)
			{
				printf("Failed to read next element line for %s.\n", element->GetName());
				return false;
			}
			cursor = next = line;

			for(int iProperty = 0, cProperty = element->NumProperties(); iProperty < cProperty; ++iProperty)
			{
				if(*cursor == '\0')
				{
					if(cursor != line)
					{
						if( fgets(line, ARRAY_SIZE(line) - 1, fp) == NULL)
						{
							printf("Failed to refresh line %s.\n", element->GetName());
							return false;
						}
						cursor = next = line;
					}
				}

				PlyPropertyBase* property = element->GetProperty(iProperty);
				if(!property->Parse(cursor, next))
				{
					printf("Failed to parse property value for %s\n", property->GetName());
					return false;
				}

				cursor = next;
			}
		}
	}
	return true;
}
	
PlyData::ElementReader PlyData::GetElement(int i)
{
	return ElementReader(m_elements[i]);
}

PlyData::ElementReader PlyData::GetElement(const char* name)
{
	for(int i = 0, c = m_elements.size(); i < c; ++i)
	{
		if(strcasecmp(name, m_elements[i]->GetName()) == 0)
			return ElementReader(m_elements[i]);
	}
	return ElementReader();
}

int PlyData::ElementReader::GetPropertyId(const char* name)
{
	for(int i = 0, c = m_element->NumProperties(); i < c; ++i)
	{
		PlyPropertyBase *base = m_element->GetProperty(i);
		if(strcasecmp(name, base->GetName()) == 0)
			return i;
	}
	return -1;
}

