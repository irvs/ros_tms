//----------------------------------------------------------
// @file   : rps.cpp
// @author : Watanabe Yuuta 
// @version: Ver0.1.4 (since 2014.05.02)
// @date   : 2016.06.09
//----------------------------------------------------------
#include <ros/ros.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>
#include <iostream>
#include <ctime>
#include <time.h>
#include <vector>
#include <math.h>


int main() {
#if 1
  std::vector<double> before_x;
  std::vector<double> before_y;
  std::vector<double> after_x;
  std::vector<double> after_y;
  std::vector<double>::iterator it_x;
  std::vector<double>::iterator it_y;
  int size_x=0;

  before_x.push_back(0.0);
  before_y.push_back(1.0);

  before_x.push_back(1.0);  
  before_y.push_back(1.0);

  before_x.push_back(1.0);
  before_y.push_back(0.0);

///////////////////////////////////////////////////////////

  after_x.push_back(0.0);
  after_y.push_back(2.0);

  after_x.push_back(1.0);  
  after_y.push_back(2.0);

  after_x.push_back(2.0);
  after_y.push_back(1.0);

  after_x.push_back(2.0);
  after_y.push_back(0.0);
 
  after_x.push_back(100.0);
  after_y.push_back(100.0);

///////////////////////////////////////////////////////////
  
  it_x = before_x.end();
  it_y = before_y.end();
    
  for(int i=0; i < after_x.size() ; i++){
    for(it_x = before_x.end(), it_y = before_y.end(); it_x != before_x.begin()-1 ; --it_x,--it_y){
        double r = sqrt(pow(*it_x - after_x[i], 2) + pow(*it_y - after_y[i], 2));
        if(r < 1.2){
           std::cout << "r " << r << "x y"<< *it_x << *it_y << "x! y!" << after_x[i] << after_y[i] << std::endl;
           //std::cout << "r " << r << std::endl;
           before_x.insert(it_x+1,after_x[i]);
           before_y.insert(it_y+1,after_y[i]);
           break;
        }
        if(it_x == before_x.begin()){
          before_x.insert(it_x,after_x[i]);
          before_y.insert(it_y,after_y[i]);
        }
        //std::cout << "r " << r << "x y"<< before_x[i] << before_y[i] << "x! y!" << after_x[j] << after_y[j] << std::endl;
    }
  it_x = before_x.begin();
  it_y = before_y.begin();
  }
#endif
std::cout << "before_x.size() " << before_x.size() << std::endl;
for(int i=0; i < before_x.size() ; i++){
std::cout << "x y"<< before_x[i] << before_y[i] << std::endl;
}
#if 0
    // ファイル入力ストリームの初期化
    std::ifstream ifs("test3.txt");
    std::string line;
    while (std::getline(ifs, line)) {
        // ファイルの中身を一行づつ表示
        std::cout << line << std::endl;
    }

    // ファイル出力ストリームの初期化
    std::ofstream ofs("test3.txt");

    // ファイルに1行ずつ書き込み
    ofs << "ham" << std::endl;
    ofs << "egg" << std::endl;
    ofs << "spam" << std::endl;
    ofs << "spam1" << std::endl;
#endif
    return 0;
}
