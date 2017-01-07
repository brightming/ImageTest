
#include "str_common.h"
#include <string>
#include <vector>
#include <fstream>

using namespace std;

int main(int argc,char** argv){

    string input_dir="/home/gumh/qtcreator-workspace/robotsight/third-src/darknet/include";
    string output_dir="/home/gumh/qtcreator-workspace/robotsight/third-src/darknet/include_mod";

    vector<string> files=getAllFilesWithPathFromDir(input_dir);

    for(string f:files){
        string fileFullName=f;
        string filePath;
        string onlyName;
        string suffix;
        getFilePart( fileFullName, filePath, onlyName, suffix);

        string new_full_name=output_dir+"/"+onlyName+"."+suffix;
        ofstream fs(new_full_name);

        fs<<"#ifdef __cplusplus"<<endl;
        fs<<"extern \"C\" {"<<endl;
        fs<<"#endif"<<endl<<endl;

        vector<string> content;
        get_file_content(fileFullName,content);
        for(string cont:content){
            fs<<cont<<endl;
        }
        fs<<endl<<endl;
        fs<<"#ifdef __cplusplus"<<endl;
        fs<<"}"<<endl;
        fs<<"#endif"<<endl<<endl;

        fs.close();
    }

    return 0;
}
