#ifndef PARAMETER_READER_H
#define PARAMETER_READER_H

#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <memory>

using namespace std;

namespace easyrosbag
{
    class ParameterReader
    {
    public:
        typedef std::shared_ptr<ParameterReader> Ptr;

        inline static ParameterReader::Ptr create(string str) { return ParameterReader::Ptr(new ParameterReader(str)); }

        ParameterReader(string filename = "../data/parameters.txt"){
            std::ifstream fin(filename.c_str());
            if (!fin)
            {
                return;
            }
            while (!fin.eof())
            {
                string str;
                getline(fin, str);
                if (str[0] == '#')
                {
                    continue;
                }

                int pos = str.find("=");
                if (pos == -1)
                    continue;
                string key = str.substr(0, pos);
                string value = str.substr(pos + 1, str.length());
                data[key] = value;

                if (!fin.good())
                    break;
            }
        }

        string getData(string key)
        {
            map<string, string>::iterator iter = data.find(key);
            if (iter == data.end())
            {
                return string("NOT_FOUND");
            }
            return iter->second;
        }

    public:
        map<string, string> data;
    };
} // namespace nav

#endif
