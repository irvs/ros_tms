//----------------------------------------------------------
// @file   : pot_ctrl.cpp
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
#include <tms_ss_pot/Particle_filter.h>
#include <tms_ss_pot/define.h>
#include <tms_ss_pot/Target.h>
#include <tms_ss_pot/Laser.h>
#include <tms_ss_pot/Multiple_particle_filter.h>

typedef struct
{
    double x;
    double y;
} pos_cv;

typedef struct
{
    int id;
    pos_cv vector;
} pos_inf;

vector<long> v_list;
int i;
int main()
{
#if 0
    std::vector<pos_inf> posinf;
    std::vector<pos_inf>::iterator v;
    pos_inf tmp_inf;

    for (int i = 0; i < 10 ; i++)
    {
        tmp_inf.id = i;
        tmp_inf.vector.x = (double)i;
        tmp_inf.vector.y = (double)i;
        posinf.push_back(tmp_inf);
    }

    for (v = posinf.begin(); v != posinf.end(); ++v)
    {
        std::cout << " id " << v->id;
        std::cout << " x " <<  v->vector.x;
        std::cout << " y " <<  v->vector.y;
        std::cout << std::endl;
    }

#endif

#if 1

    v_list.push_back(100);
    v_list.push_back(200);
    v_list.push_back(300);
    vector< long >::iterator cIter;
    vector< long >::iterator v;
    cIter = find( v_list.begin(), v_list.end() , 200 );
    if ( cIter != v_list.end() )
    {
        std::cout << "200は存在してます" << std::endl;
    }
    v_list.erase (cIter);
    std::cout << "erase 200" << std::endl;

    cIter = find( cIter, v_list.end() , 300 );
    v_list.erase (cIter);
    std::cout << "erase 300" << std::endl;

    for (cIter = v_list.begin(); cIter != v_list.end(); ++cIter)
    {
        std::cout << *cIter ;
    }
    std::cout << std::endl;
#endif

#if 0
    pos_t titi;
    std::cout << "maxconnect" << MAX_CONNECT  << std::endl;
    titi.x = 9;
    titi.y = 8;
    std::cout << titi.y  << titi.x << std::endl;
#endif
#if 0
    std::vector<double> before_x;
    std::vector<double> before_y;
    std::vector<double> after_x;
    std::vector<double> after_y;
    std::vector<double>::iterator it_x;
    std::vector<double>::iterator it_y;
    int size_x = 0;

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

    for (int i = 0; i < after_x.size() ; i++)
    {
        for (it_x = before_x.end(), it_y = before_y.end(); it_x != before_x.begin() - 1 ; --it_x, --it_y)
        {
            double r = sqrt(pow(*it_x - after_x[i], 2) + pow(*it_y - after_y[i], 2));
            if (r < 1.2)
            {
                std::cout << "r " << r << "x y" << *it_x << *it_y << "x! y!" << after_x[i] << after_y[i] << std::endl;
                //std::cout << "r " << r << std::endl;
                before_x.insert(it_x + 1, after_x[i]);
                before_y.insert(it_y + 1, after_y[i]);
                break;
            }
            if (it_x == before_x.begin())
            {
                before_x.insert(it_x, after_x[i]);
                before_y.insert(it_y, after_y[i]);
            }
            //std::cout << "r " << r << "x y"<< before_x[i] << before_y[i] << "x! y!" << after_x[j] << after_y[j] << std::endl;
        }
        it_x = before_x.begin();
        it_y = before_y.begin();
    }

    std::cout << "before_x.size() " << before_x.size() << std::endl;
    for (int i = 0; i < before_x.size() ; i++)
    {
        std::cout << "x y" << before_x[i] << before_y[i] << std::endl;
    }
#endif
#if 0
    // ファイル入力ストリームの初期化
    std::ifstream ifs("test3.txt");
    std::string line;
    while (std::getline(ifs, line))
    {
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
