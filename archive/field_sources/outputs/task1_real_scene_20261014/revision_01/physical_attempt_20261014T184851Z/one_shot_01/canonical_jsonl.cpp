#include <iostream>
#include <string>
#include <nlohmann/json.hpp>
int main(){
  std::string line;
  while(std::getline(std::cin,line))if(!line.empty())
    std::cout<<nlohmann::json::parse(line).dump()<<'\n';
}
