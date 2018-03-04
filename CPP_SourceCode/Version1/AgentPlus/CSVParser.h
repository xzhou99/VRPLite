#pragma once
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <sstream>
using namespace std;
using std::string;
using std::ifstream;
using std::vector;
using std::map;
using std::istringstream;
template <typename T>

#pragma warning(disable: 4244)  // stop warning: "conversion from 'int' to 'float', possible loss of data"

string NumberToString(T Number)
{
	ostringstream ss;
	ss << Number;
	return ss.str();
}


template <typename T>
T StringToNumber(const string &Text)
{
	istringstream ss(Text);
	T result;
	return ss >> result ? result : 0;
}
class CCSVParser
{
public:
	char Delimiter;
	bool IsFirstLineHeader;
	ifstream inFile;
	vector<string> LineFieldsValue;
	vector<string> Headers;
	map<string, int> FieldsIndices;

	vector<int> LineIntegerVector;

public:
	void  ConvertLineStringValueToIntegers()
	{
		LineIntegerVector.clear();
		for (unsigned i = 0; i < LineFieldsValue.size(); i++)
		{
			std::string si = LineFieldsValue[i];
			int value = atoi(si.c_str());

			if (value >= 1)
				LineIntegerVector.push_back(value);

		}
	}
	vector<string> GetHeaderVector()
	{
		return Headers;
	}

	int m_EmptyLineCount;
	bool m_bDataHubSingleCSVFile;
	string m_DataHubSectionName;
	bool m_bLastSectionRead;

	bool m_bSkipFirstLine;  // for DataHub CSV files

	CCSVParser::CCSVParser(void)
	{
		Delimiter = ',';
		IsFirstLineHeader = true;
		m_bSkipFirstLine = false;
		m_bDataHubSingleCSVFile = false;
		m_bLastSectionRead = false;
		m_EmptyLineCount++;
	}

	CCSVParser::~CCSVParser(void)
	{
		if (inFile.is_open()) inFile.close();
	}


	bool CCSVParser::OpenCSVFile(string fileName, bool bIsFirstLineHeader)
	{
		inFile.clear();
		inFile.open(fileName.c_str());

		IsFirstLineHeader = bIsFirstLineHeader;
		if (inFile.is_open())
		{
			if (m_bSkipFirstLine)
			{
				string s;
				std::getline(inFile, s);
			}
			if (IsFirstLineHeader)
			{
				string s;
				std::getline(inFile, s);

				if (s.length() == 0)
					return true;

				vector<string> FieldNames = ParseLine(s);

				for (size_t i = 0; i<FieldNames.size(); i++)
				{
					string tmp_str = FieldNames.at(i);
					size_t start = tmp_str.find_first_not_of(" ");

					string name;
					if (start == string::npos)
					{
						name = "";
					}
					else
					{
						name = tmp_str.substr(start);
						TRACE("%s,", name.c_str());
					}
					Headers.push_back(name);
					FieldsIndices[name] = (int)i;
				}
			}

			return true;
		}
		else
		{
			return false;
		}
	}

	bool CCSVParser::ReadSectionHeader(string s)
	{
		//skip // data 

		Headers.clear();
		FieldsIndices.clear();

		if (s.length() == 0)
			return true;

		vector<string> FieldNames = ParseLine(s);

		for (size_t i = 0; i<FieldNames.size(); i++)
		{
			string tmp_str = FieldNames.at(i);
			size_t start = tmp_str.find_first_not_of(" ");

			string name;
			if (start == string::npos)
			{
				name = "";
			}
			else
			{
				name = tmp_str.substr(start);
				TRACE("%s,", name.c_str());
			}
			Headers.push_back(name);
			FieldsIndices[name] = (int)i;
		}


		return true;

	}
	void CCSVParser::CloseCSVFile(void)
	{
		inFile.close();
	}

	vector<string> CCSVParser::GetLineRecord()
	{
		return LineFieldsValue;
	}

	vector<string> CCSVParser::GetHeaderList()
	{
		return Headers;
	}

	bool CCSVParser::ReadRecord()
	{
		LineFieldsValue.clear();

		if (inFile.is_open())
		{
			string s;
			std::getline(inFile, s);
			if (s.length() > 0)
			{
				if (m_bDataHubSingleCSVFile && s.find("[") != string::npos)  // DataHub single csv file
				{
					LineFieldsValue = ParseLine(s);

					if (LineFieldsValue.size() >= 1 && LineFieldsValue[0].find("[") != string::npos)
					{
						m_DataHubSectionName = LineFieldsValue[0];

					}

					//re-read section header
					ReadSectionHeader(s);
					LineFieldsValue.clear();
					std::getline(inFile, s);

					LineFieldsValue = ParseLine(s);

				}
				else
				{
					LineFieldsValue = ParseLine(s);

				}
				return true;
			}
			else
			{

					return false;
			}
		}
		else
		{
			return false;
		}
	}

	vector<string> CCSVParser::ParseLine(string line)
	{
		vector<string> SeperatedStrings;
		string subStr;

		if (line.length() == 0)
			return SeperatedStrings;

		istringstream ss(line);


		if (line.find_first_of('"') == string::npos)
		{

			while (std::getline(ss, subStr, Delimiter))
			{
				SeperatedStrings.push_back(subStr);
			}

			if (line.at(line.length() - 1) == ',')
			{
				SeperatedStrings.push_back("");
			}
		}
		else
		{
			while (line.length() > 0)
			{
				size_t n1 = line.find_first_of(',');
				size_t n2 = line.find_first_of('"');

				if (n1 == string::npos && n2 == string::npos) //last field without double quotes
				{
					subStr = line;
					SeperatedStrings.push_back(subStr);
					break;
				}

				if (n1 == string::npos && n2 != string::npos) //last field with double quotes
				{
					size_t n3 = line.find_first_of('"', n2 + 1); // second double quote

					//extract content from double quotes
					subStr = line.substr(n2 + 1, n3 - n2 - 1);
					SeperatedStrings.push_back(subStr);

					break;
				}

				if (n1 != string::npos && (n1 < n2 || n2 == string::npos))
				{
					subStr = line.substr(0, n1);
					SeperatedStrings.push_back(subStr);
					if (n1 < line.length() - 1)
					{
						line = line.substr(n1 + 1);
					}
					else // comma is the last char in the line string, push an empty string to the back of vector
					{
						SeperatedStrings.push_back("");
						break;
					}
				}

				if (n1 != string::npos && n2 != string::npos && n2 < n1)
				{
					size_t n3 = line.find_first_of('"', n2 + 1); // second double quote
					subStr = line.substr(n2 + 1, n3 - n2 - 1);
					SeperatedStrings.push_back(subStr);
					size_t idx = line.find_first_of(',', n3 + 1);

					if (idx != string::npos)
					{
						line = line.substr(idx + 1);
					}
					else
					{
						break;
					}
				}
			}

		}

		return SeperatedStrings;
	}
	template <class T> bool GetValueBySectionKeyFieldName(string file_name, string section_name, string key_name, string field_name, T& value)
	{
		OpenCSVFile(file_name);
		while (ReadRecord())
		{
			if (LineFieldsValue[0] != section_name || LineFieldsValue[1] != key_name)
				continue;

			if (FieldsIndices.find(field_name) == FieldsIndices.end())
			{
				CloseCSVFile();
				return false;
			}
			else
			{
				if (LineFieldsValue.size() == 0)
				{
					CloseCSVFile();
					return false;
				}

				int size = (int)(LineFieldsValue.size());
				if (FieldsIndices[field_name] >= size)
				{
					CloseCSVFile();
					return false;
				}

				string str_value = LineFieldsValue[FieldsIndices[field_name]];

				if (str_value.length() <= 0)
				{
					CloseCSVFile();
					return false;
				}

				istringstream ss(str_value);

				T converted_value;
				ss >> converted_value;

				if (/*!ss.eof() || */ ss.fail())
				{

					CloseCSVFile();
					return false;
				}

				value = converted_value;
				CloseCSVFile();
				return true;
			}
		}
		CloseCSVFile();

		return false;
	}
	template <class T> bool GetValueByFieldName(string field_name, T& value, bool NonnegativeFlag = true)
	{
		if (FieldsIndices.find(field_name) == FieldsIndices.end())
		{
			return false;
		}
		else
		{
			if (LineFieldsValue.size() == 0)
			{
				return false;
			}

			int size = (int)(LineFieldsValue.size());
			if (FieldsIndices[field_name] >= size)
			{
				return false;
			}

			string str_value = LineFieldsValue[FieldsIndices[field_name]];

			if (str_value.length() <= 0)
			{
				return false;
			}

			istringstream ss(str_value);

			T converted_value;
			ss >> converted_value;

			if (/*!ss.eof() || */ ss.fail())
			{
				return false;
			}

			if (NonnegativeFlag && converted_value<0)
				converted_value = 0;

			value = converted_value;
			return true;
		}
	}


	bool GetValueByFieldName(string field_name, string& value)
	{
		if (FieldsIndices.find(field_name) == FieldsIndices.end())
		{
			return false;
		}
		else
		{
			if (LineFieldsValue.size() == 0)
			{
				return false;
			}

			unsigned int index = FieldsIndices[field_name];
			if (index >= LineFieldsValue.size())
			{
				return false;
			}
			string str_value = LineFieldsValue[index];

			if (str_value.length() <= 0)
			{
				return false;
			}

			value = str_value;
			return true;
		}
	}

};

class CCSVWriter
{
public:
	ofstream outFile;
	char Delimiter;
	int FieldIndex;
	bool IsFirstLineHeader;
	map<int, string> LineFieldsValue;
	vector<string> LineFieldsName;
	vector<string> LineFieldsCategoryName;
	map<string, int> FieldsIndices;

	bool row_title;

public:
	void SetRowTitle(bool flag)
	{
		row_title = flag;
	}

	bool OpenCSVFile(string fileName, bool b_required = true);
	void CloseCSVFile(void)
	{
		outFile.close();
	}
	template <class T> bool SetValueByFieldName(string field_name, T& value)  // by doing so, we do not need to exactly follow the sequence of field names
	{
		if (FieldsIndices.find(field_name) == FieldsIndices.end())
		{
			return false;
		}
		else
		{

			LineFieldsValue[FieldsIndices[field_name]] = NumberToString(value);

			return true;
		}
	}

	void Reset()
	{

		LineFieldsValue.clear();
		LineFieldsName.clear();
		LineFieldsCategoryName.clear();
		FieldsIndices.clear();

	}
	void SetFieldName(string field_name)
	{
		FieldsIndices[field_name] = (int)(LineFieldsName.size());
		LineFieldsName.push_back(field_name);
		LineFieldsCategoryName.push_back(" ");

	}

	template <class T>  void SetFieldNameAndValue(string field_name, T& value)
	{
		FieldsIndices[field_name] = (int)(LineFieldsName.size());
		LineFieldsName.push_back(field_name);
		LineFieldsCategoryName.push_back(" ");
		SetValueByFieldName(field_name, value);
	}

	void SetFieldNameWithCategoryName(string field_name, string category_name)
	{
		FieldsIndices[field_name] = (int)(LineFieldsName.size());
		LineFieldsName.push_back(field_name);
		LineFieldsCategoryName.push_back(category_name);

	}


	void WriteTextString(CString textString)
	{
		if (!outFile.is_open())
			return;
		outFile << textString << endl;

	}

	void WriteTextLabel(CString textString)
	{
		if (!outFile.is_open())
			return;
		outFile << textString;

	}

	template <class T>  void WriteNumber(T value)
	{
		if (!outFile.is_open())
			return;
		outFile << NumberToString(value) << endl;
	}

	template <class T>  void WriteParameterValue(CString textString, T value)
	{
		if (!outFile.is_open())
			return;

		outFile << textString << "=," << NumberToString(value) << endl;
	}

	void WriteNewEndofLine()
	{
		if (!outFile.is_open())
			return;
		outFile << endl;
	}


	void WriteHeader()
	{
		if (!outFile.is_open())
			return;


		//for(unsigned int i = 0; i< FieldsIndices.size(); i++)
		//{
		//outFile << LineFieldsCategoryName[i] << ",";
		//}
		//outFile << endl;

		//if(row_title == true)
		//	outFile << ",";

		for (unsigned int i = 0; i< FieldsIndices.size(); i++)
		{
			outFile << LineFieldsName[i] << ",";
		}

		outFile << endl;
	}
	void WriteRecord()
	{
		if (!outFile.is_open())
			return;

		for (unsigned int i = 0; i< FieldsIndices.size(); i++)
		{
			string str;
			if (LineFieldsValue.find(i) != LineFieldsValue.end() && LineFieldsValue[i].size() >= 1) // has been initialized
				outFile << LineFieldsValue[i].c_str() << ",";
			else
				outFile << ' ' << ",";
		}

		LineFieldsValue.clear();

		outFile << endl;
	}

	CCSVWriter::CCSVWriter()
	{
		row_title = false;
		FieldIndex = 0;
		Delimiter = ',';
		IsFirstLineHeader = true;
	}

	CCSVWriter::~CCSVWriter(void)
	{
		if (outFile.is_open()) outFile.close();
	}


	CCSVWriter::CCSVWriter(string fileName)
	{
		Open(fileName);

	};

	bool CCSVWriter::Open(string fileName)
	{
		outFile.open(fileName.c_str());

		if (outFile.is_open() == false)
		{

			cout << "File cannot be opened." << endl;
			return false;
		}

		return true;
	};

	void CCSVWriter::OpenAppend(string fileName)
	{
		outFile.open(fileName.c_str(), fstream::app);

		if (outFile.is_open() == false)
		{
			cout << "File " << fileName.c_str() << " cannot be opened." << endl;
			getchar();
			exit(0);
		}

	};
};


