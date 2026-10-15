#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
int main(int argc,char**argv){
 if(argc!=3)return 2;
 std::ifstream source(argv[1]);std::ofstream target(argv[2]);
 if(!source||!target)return 3;
 std::string line;while(std::getline(source,line))if(!line.empty())
  target<<nlohmann::json::parse(line).dump()<<'\n';
 return target.good()?0:4;
}
