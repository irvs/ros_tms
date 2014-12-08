//----------------------------------------------------------
// @file   : pot_psen_manager.cpp
// @author : Watanabe Yuuta
// @version: Ver0.1.4 (since 2014.05.02)
// @date   : 2014.11.18
//----------------------------------------------------------

#include <ros/ros.h>
#include <pthread.h>
#include <std_msgs/String.h>
#include <tms_msg_ss/pot_tracking_psen.h>
#include <tms_msg_ss/pot_tracking_psens.h>

pthread_mutex_t mutex_laser  = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_target = PTHREAD_MUTEX_INITIALIZER;

#include "tms_ss_pot/define.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>
#include <iostream>
#include <ctime>
#include <time.h>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>

#define mode_s 0
#define mode_e 5
#define lrf_number_s 0
#define lrf_number_e 11
#define ini_value_s  0
#define ini_value_e  7
#define feature_s 0
#define feature_e 8

typedef struct
{
    int righter;
    int lefter;
} counter;

typedef struct
{
    int mode;
    std::vector<int>  LRF_number;
    std::vector<int>  init_value;
    int feature;
    int calc;
    int calc_righter;
    int calc_lefter;
    double time;
} composition;


double g_StartTime;
std::vector<counter> all_cnt;
std::vector<composition> all_cmp;
composition tmp_comp;
namespace fs = boost::filesystem;

void  lrf_display();

void *Management( void *ptr )
{
    ros::Rate r(30);
    while (ros::ok())
    {
        const fs::path path("/home/kurt/catkin_ws/position");
        all_cmp.clear();
        BOOST_FOREACH(const fs::path & p, std::make_pair(fs::directory_iterator(path), fs::directory_iterator()))
        {
            if (!fs::is_directory(p) && boost::algorithm::iends_with(p.string(), ".txt"))
            {
                std::ifstream ifs(p.c_str());
                std::string line;
                std::string tmp_line;
                int line_number = 0;
                while (getline(ifs, line))
                {
                    line_number++;
                    // ファイルの中身
                    //std::cout << line << std::endl;
                    if (line.substr(mode_s, mode_e) == "mode:")
                    {
                        int fs;
                        fs = line.find(",");
                        fs = fs - mode_e;
                        //std::cout << "fs " << fs << std::endl;
                        tmp_line = line.substr(mode_e, fs);
                        //std::cout << tmp_line << std::endl;
                        tmp_comp.mode = atoi(tmp_line.c_str());
                    }
                    else if (line.substr(lrf_number_s, lrf_number_e) == "LRF_number:")
                    {
                        //mode == 1 or mode == 2;
                        if (tmp_comp.mode == 1 || tmp_comp.mode == 2)
                        {
                            int fp = lrf_number_e;
                            int fn;
                            for (int i = 0; i < 2; i++)
                            {
                                fn = line.find("," , fp);
                                fn = fn - fp;
                                //std::cout << line.substr(fp, fn) << " ";
                                tmp_comp.LRF_number.push_back(atoi(line.substr(fp, fn).c_str()));
                                fp = fp + fn + 1;
                            }
                            //std::cout << std::endl;
                        }
                        else if (tmp_comp.mode == 3 || tmp_comp.mode == 4 || tmp_comp.mode == 5 || tmp_comp.mode == 6)
                        {
                            int fp = lrf_number_e;
                            int fn;
                            for (int i = 0; i < 3; i++)
                            {
                                fn = line.find("," , fp);
                                fn = fn - fp;
                                //std::cout << line.substr(fp, fn) << " ";
                                tmp_comp.LRF_number.push_back(atoi(line.substr(fp, fn).c_str()));
                                fp = fp + fn + 1;
                            }
                            //std::cout << std::endl;
                        }
                        else if (tmp_comp.mode == 7 || tmp_comp.mode == 8 || tmp_comp.mode == 9 || tmp_comp.mode == 10 || tmp_comp.mode == 11 || tmp_comp.mode == 12 || tmp_comp.mode == 13 || tmp_comp.mode == 14)
                        {
                            int fp = lrf_number_e;
                            int fn;
                            for (int i = 0; i < 4; i++)
                            {
                                fn = line.find("," , fp);
                                fn = fn - fp;
                                //std::cout << line.substr(fp, fn) << " ";
                                tmp_comp.LRF_number.push_back(atoi(line.substr(fp, fn).c_str()));
                                fp = fp + fn + 1;
                            }
                            //std::cout << std::endl;
                        }

                    }
                    else if (line.substr(ini_value_s, ini_value_e) == "inival:")
                    {
                        //mode == 1 or mode == 2;
                        if (tmp_comp.mode == 1 || tmp_comp.mode == 2)
                        {
                            int fp = ini_value_e;
                            int fn;
                            for (int i = 0; i < 4; i++)
                            {
                                fn = line.find("," , fp);
                                fn = fn - fp;
                                //std::cout << line.substr(fp, fn) << " ";
                                tmp_comp.init_value.push_back(atoi(line.substr(fp, fn).c_str()));
                                fp = fp + fn + 1;
                            }
                            //std::cout << std::endl;
                        }
                        else if (tmp_comp.mode == 3 || tmp_comp.mode == 4 || tmp_comp.mode == 5 || tmp_comp.mode == 6)
                        {
                            int fp = ini_value_e;
                            int fn;
                            for (int i = 0; i < 6; i++)
                            {
                                fn = line.find("," , fp);
                                fn = fn - fp;
                                //std::cout << line.substr(fp, fn) << " ";
                                tmp_comp.init_value.push_back(atoi(line.substr(fp, fn).c_str()));
                                fp = fp + fn + 1;
                            }
                            //std::cout << std::endl;
                        }
                        else if (tmp_comp.mode == 7 || tmp_comp.mode == 8 || tmp_comp.mode == 9 || tmp_comp.mode == 10 || tmp_comp.mode == 11 || tmp_comp.mode == 12 || tmp_comp.mode == 13 || tmp_comp.mode == 14)
                        {
                            int fp = ini_value_e;
                            int fn;
                            for (int i = 0; i < 8; i++)
                            {
                                fn = line.find("," , fp);
                                fn = fn - fp;
                                //std::cout << line.substr(fp, fn) << " ";
                                tmp_comp.init_value.push_back(atoi(line.substr(fp, fn).c_str()));
                                fp = fp + fn + 1;
                            }
                            //std::cout << std::endl;
                        }

                    }
                    else if (line.substr(0, 11) == "start_time:")
                    {
                        int fp = 11;
                        int fn;
                        fn = line.find("," , fp);
                        fn = fn - fp;
                        tmp_comp.time = atof(line.substr(fp, fn).c_str());
                    }
                    else if (line.substr(feature_s, feature_e) == "feature:")
                    {
                        int fp = feature_e;
                        int fn;
                        fn = line.find("," , fp);
                        fn = fn - fp;
                        tmp_comp.feature = atoi(line.substr(fp, fn).c_str());
                        all_cmp.push_back(tmp_comp);
                        tmp_comp.mode == 0;
                        tmp_comp.LRF_number.clear();
                        tmp_comp.init_value.clear();
                        tmp_comp.feature == 0;
                    }


                    if (line_number == 5)break;
                }
            }
        }
        int clsn = 0;
        for (clsn = 0 ; clsn < all_cmp.size() ; clsn++)
        {
            if (all_cmp[clsn].mode == 1 || all_cmp[clsn].mode == 2)
            {
                int f_righter = all_cnt[all_cmp[clsn].LRF_number[0] - 1].righter;
                int f_lefter  = all_cnt[all_cmp[clsn].LRF_number[0] - 1].lefter;
                int s_righter = all_cnt[all_cmp[clsn].LRF_number[1] - 1].righter;
                int s_lefter  = all_cnt[all_cmp[clsn].LRF_number[1] - 1].lefter;
                int f_righter_dose = all_cmp[clsn].init_value[0];
                int f_lefter_dose  = all_cmp[clsn].init_value[1];
                int s_righter_dose = all_cmp[clsn].init_value[2];
                int s_lefter_dose  = all_cmp[clsn].init_value[3];
                int calc;
                int calc_righter;
                int calc_lefter;
                if (all_cmp[clsn].mode == 1)
                {
                    calc = (f_lefter + s_righter - (f_lefter_dose + s_righter_dose)) - (f_righter + s_lefter - (f_righter_dose +  s_lefter_dose));
                    calc_righter = (f_righter - f_righter_dose) - (s_righter - s_righter_dose);
                    calc_lefter  = (f_lefter - f_lefter_dose) - (s_lefter - s_lefter_dose);
                    all_cmp[clsn].calc = calc;
                    all_cmp[clsn].calc_righter = calc_righter;
                    all_cmp[clsn].calc_lefter =  calc_lefter;
                }

                else if (all_cmp[clsn].mode == 2)
                {
                    calc = (f_lefter + s_lefter - (f_lefter_dose + s_lefter_dose)) - (f_righter + s_righter - (f_righter_dose +  s_righter_dose));
                    calc_righter = (f_righter - f_righter_dose) - (f_lefter - f_lefter_dose);
                    calc_lefter  = (s_righter - s_righter_dose) - (s_lefter - s_lefter_dose);
                    all_cmp[clsn].calc = calc;
                    all_cmp[clsn].calc_righter = calc_righter;
                    all_cmp[clsn].calc_lefter =  calc_lefter;
                }
            }
            else if (all_cmp[clsn].mode == 3 || all_cmp[clsn].mode == 4 || all_cmp[clsn].mode == 5 || all_cmp[clsn].mode == 6)
            {
                int f_righter = all_cnt[all_cmp[clsn].LRF_number[0] - 1].righter;
                int f_lefter  = all_cnt[all_cmp[clsn].LRF_number[0] - 1].lefter;
                int s_righter = all_cnt[all_cmp[clsn].LRF_number[1] - 1].righter;
                int s_lefter  = all_cnt[all_cmp[clsn].LRF_number[1] - 1].lefter;
                int t_righter = all_cnt[all_cmp[clsn].LRF_number[2] - 1].righter;
                int t_lefter  = all_cnt[all_cmp[clsn].LRF_number[2] - 1].lefter;
                int f_righter_dose = all_cmp[clsn].init_value[0];
                int f_lefter_dose  = all_cmp[clsn].init_value[1];
                int s_righter_dose = all_cmp[clsn].init_value[2];
                int s_lefter_dose  = all_cmp[clsn].init_value[3];
                int t_righter_dose = all_cmp[clsn].init_value[4];
                int t_lefter_dose  = all_cmp[clsn].init_value[5];
                int calc;
                if (all_cmp[clsn].mode == 3)
                {
                    calc = (f_lefter + s_lefter + t_righter - (f_lefter_dose + s_lefter_dose + t_righter_dose)) - (f_righter + s_righter + t_lefter - (f_righter_dose + s_righter_dose + t_lefter_dose));
                    all_cmp[clsn].calc = calc;
                    all_cmp[clsn].calc_righter = 10000;
                    all_cmp[clsn].calc_lefter =  10000;
                }
                else if (all_cmp[clsn].mode == 4)
                {
                    calc = (f_lefter + s_lefter + t_lefter - (f_lefter_dose + s_lefter_dose + t_lefter_dose)) - (f_righter + s_righter + t_righter - (f_righter_dose + s_righter_dose + t_righter_dose));
                    all_cmp[clsn].calc = calc;
                    all_cmp[clsn].calc_righter = 10000;
                    all_cmp[clsn].calc_lefter =  10000;
                }
                else if (all_cmp[clsn].mode == 5)
                {
                    calc = (f_lefter + s_righter + t_righter - (f_lefter_dose + s_righter_dose + t_righter_dose)) - (f_righter + s_lefter + t_lefter - (f_righter_dose + s_lefter_dose + t_lefter_dose));
                    all_cmp[clsn].calc = calc;
                    all_cmp[clsn].calc_righter = 10000;
                    all_cmp[clsn].calc_lefter =  10000;
                }
                else if (all_cmp[clsn].mode == 6)
                {
                    calc = (f_lefter + s_righter + t_lefter - (f_lefter_dose + s_righter_dose + t_lefter_dose)) - (f_righter + s_lefter + t_righter - (f_righter_dose + s_lefter_dose + t_righter_dose));
                    all_cmp[clsn].calc = calc;
                    all_cmp[clsn].calc_righter = 10000;
                    all_cmp[clsn].calc_lefter =  10000;
                }

            }
            else if (all_cmp[clsn].mode == 7 || all_cmp[clsn].mode == 8 || all_cmp[clsn].mode == 9 || all_cmp[clsn].mode == 10 || all_cmp[clsn].mode == 11 || all_cmp[clsn].mode == 12 || all_cmp[clsn].mode == 13 || all_cmp[clsn].mode == 14 )
            {
                int f_righter = all_cnt[all_cmp[clsn].LRF_number[0] - 1].righter;
                int f_lefter  = all_cnt[all_cmp[clsn].LRF_number[0] - 1].lefter;
                int s_righter = all_cnt[all_cmp[clsn].LRF_number[1] - 1].righter;
                int s_lefter  = all_cnt[all_cmp[clsn].LRF_number[1] - 1].lefter;
                int t_righter = all_cnt[all_cmp[clsn].LRF_number[2] - 1].righter;
                int t_lefter  = all_cnt[all_cmp[clsn].LRF_number[2] - 1].lefter;
                int tf_righter = all_cnt[all_cmp[clsn].LRF_number[3] - 1].righter;
                int tf_lefter = all_cnt[all_cmp[clsn].LRF_number[3] - 1].lefter;
                int f_righter_dose = all_cmp[clsn].init_value[0];
                int f_lefter_dose  = all_cmp[clsn].init_value[1];
                int s_righter_dose = all_cmp[clsn].init_value[2];
                int s_lefter_dose  = all_cmp[clsn].init_value[3];
                int t_righter_dose = all_cmp[clsn].init_value[4];
                int t_lefter_dose  = all_cmp[clsn].init_value[5];
                int tf_righter_dose = all_cmp[clsn].init_value[6];
                int tf_lefter_dose = all_cmp[clsn].init_value[7];
                int calc;
                if (all_cmp[clsn].mode == 7)
                {
                    calc = (f_lefter + s_lefter + t_righter + tf_righter - (f_lefter_dose + s_lefter_dose + t_righter_dose + tf_righter_dose)) - (f_righter + s_righter + t_lefter + tf_lefter - (f_righter_dose + s_righter_dose + t_lefter_dose + tf_lefter_dose));
                    all_cmp[clsn].calc = calc;
                    all_cmp[clsn].calc_righter = 10000;
                    all_cmp[clsn].calc_lefter =  10000;
                }
                else if (all_cmp[clsn].mode == 8)
                {
                    calc = (f_lefter + s_lefter + t_righter + tf_lefter - (f_lefter_dose + s_lefter_dose + t_righter_dose + tf_lefter_dose)) - (f_righter + s_righter + t_lefter + tf_righter - (f_righter_dose + s_righter_dose + t_lefter_dose + tf_righter_dose));
                    all_cmp[clsn].calc = calc;
                    all_cmp[clsn].calc_righter = 10000;
                    all_cmp[clsn].calc_lefter =  10000;
                }
                else if (all_cmp[clsn].mode == 9)
                {
                    calc = (f_lefter + s_lefter + t_lefter + tf_righter - (f_lefter_dose + s_lefter_dose + t_lefter_dose + tf_righter_dose)) - (f_righter + s_righter + t_righter + tf_lefter - (f_righter_dose + s_righter_dose + t_righter_dose + tf_lefter_dose));
                    all_cmp[clsn].calc = calc;
                    all_cmp[clsn].calc_righter = 10000;
                    all_cmp[clsn].calc_lefter =  10000;
                }
                else if (all_cmp[clsn].mode == 10)
                {
                    calc = (f_lefter + s_lefter + t_lefter + tf_lefter - (f_lefter_dose + s_lefter_dose + t_lefter_dose + tf_lefter_dose)) - (f_righter + s_righter + t_righter + tf_righter - (f_righter_dose + s_righter_dose + t_righter_dose + tf_righter_dose));
                    all_cmp[clsn].calc = calc;
                    all_cmp[clsn].calc_righter = 10000;
                    all_cmp[clsn].calc_lefter =  10000;
                }
                else if (all_cmp[clsn].mode == 11)
                {
                    calc = (f_lefter + s_righter + t_righter + tf_righter - (f_lefter_dose + s_righter_dose + t_righter_dose + tf_righter_dose)) - (f_righter + s_lefter + t_lefter + tf_lefter - (f_righter_dose + s_lefter_dose + t_lefter_dose + tf_lefter_dose));
                    all_cmp[clsn].calc = calc;
                    all_cmp[clsn].calc_righter = 10000;
                    all_cmp[clsn].calc_lefter =  10000;
                }
                else if (all_cmp[clsn].mode == 12)
                {
                    calc = (f_lefter + s_righter + t_righter + tf_lefter - (f_lefter_dose + s_righter_dose + t_righter_dose + tf_lefter_dose)) - (f_righter + s_lefter + t_lefter + tf_righter - (f_righter_dose + s_lefter_dose + t_lefter_dose + tf_righter_dose));
                    all_cmp[clsn].calc = calc;
                    all_cmp[clsn].calc_righter = 10000;
                    all_cmp[clsn].calc_lefter =  10000;
                }
                else if (all_cmp[clsn].mode == 13)
                {
                    calc = (f_lefter + s_righter + t_lefter + tf_righter - (f_lefter_dose + s_righter_dose + t_lefter_dose + tf_righter_dose)) - (f_righter + s_lefter + t_righter + tf_lefter - (f_righter_dose + s_lefter_dose + t_righter_dose + tf_lefter_dose));
                    all_cmp[clsn].calc = calc;
                    all_cmp[clsn].calc_righter = 10000;
                    all_cmp[clsn].calc_lefter =  10000;
                }
                else if (all_cmp[clsn].mode == 14)
                {
                    calc = (f_lefter + s_righter + t_lefter + tf_lefter - (f_lefter_dose + s_righter_dose + t_lefter_dose + tf_lefter_dose)) - (f_righter + s_lefter + t_righter + tf_righter - (f_righter_dose + s_lefter_dose + t_righter_dose + tf_righter_dose));
                    all_cmp[clsn].calc = calc;
                    all_cmp[clsn].calc_righter = 10000;
                    all_cmp[clsn].calc_lefter =  10000;
                }
            }
        }
        lrf_display();
        all_cmp.clear();
        r.sleep();

    }
}


void  lrf_display()
{
    int new_put_lrf_number;
    int replace_lrf_number;
    std::vector<int> predominance;
    predominance.resize(6);
    //predominance.clear();
    int cnt = 0;
    char str[20];
    for (int clsn = 0 ; clsn < all_cmp.size() ; clsn++)
    {
        for (int i = 0; i < all_cmp[clsn].LRF_number.size(); i++)
        {
            if (all_cmp[clsn].mode == 1 || all_cmp[clsn].mode == 2)
            {
                predominance[all_cmp[clsn].LRF_number[i] - 1] = predominance[all_cmp[clsn].LRF_number[i] - 1] + 1;
            }
            else if (all_cmp[clsn].mode == 3 || all_cmp[clsn].mode == 4 || all_cmp[clsn].mode == 5 || all_cmp[clsn].mode == 6 )
            {
                predominance[all_cmp[clsn].LRF_number[i] - 1] = predominance[all_cmp[clsn].LRF_number[i] - 1] + 2;
            }
            else if (all_cmp[clsn].mode == 7 || all_cmp[clsn].mode == 8 || all_cmp[clsn].mode == 9 || all_cmp[clsn].mode == 10 || all_cmp[clsn].mode == 11 || all_cmp[clsn].mode == 12 || all_cmp[clsn].mode == 13 )
            {
                predominance[all_cmp[clsn].LRF_number[i] - 1] = predominance[all_cmp[clsn].LRF_number[i] - 1] + 3;
            }
        }
    }
    for (int clsn = 0; clsn < all_cnt.size(); clsn++)
    {
        if (predominance[clsn] == 0)
        {
            new_put_lrf_number = clsn + 1;
            break;
        }
        else if (clsn == predominance.size() - 1 )
        {
            new_put_lrf_number = 100;
        }
    }
    std::cout << "\033[2J" << "\033[10A";
    std::cout << "---LRF status---" << new_put_lrf_number << std::endl;
    std::cout << "LRF    :";
    cnt = 16;
    for (int clsn = 0 ; clsn < all_cnt.size() ; clsn++)
    {
        std::cout << clsn + 1 << "[" << predominance[clsn] << "]" << "\r";
        sprintf(str, "\033[%dC", cnt);
        std::cout << str;
        cnt = cnt + 8;
    }
    pthread_mutex_lock(&mutex_target);
    std::cout << std::endl;
    std::cout << "righter:";
    cnt = 16;
    for (int clsn = 0 ; clsn < all_cnt.size() ; clsn++)
    {
        std::cout << all_cnt[clsn].righter << "\r";
        sprintf(str, "\033[%dC", cnt);
        std::cout << str;
        cnt = cnt + 8;
    }
    std::cout << std::endl;
    std::cout << "lefter :";
    cnt = 16;
    for (int clsn = 0 ; clsn < all_cnt.size() ; clsn++)
    {
        std::cout << all_cnt[clsn].lefter << "\r";
        sprintf(str, "\033[%dC", cnt);
        std::cout << str;
        cnt = cnt + 8;
    }
    std::cout << std::endl;
    double ExeTime = ros::Time::now().toSec() - g_StartTime;
    std::cout << "---mode setting--- Time " << ExeTime << std::endl;

    for (int clsn = 0 ; clsn < all_cmp.size() ; clsn++)
    {
        std::cout << clsn << ". mode " << all_cmp[clsn].mode << " LRF_number ";
        replace_lrf_number = all_cmp[clsn].LRF_number[0];
        for (int i = 0; i < all_cmp[clsn].LRF_number.size(); i++)
        {
            std::cout <<  all_cmp[clsn].LRF_number[i];
            if (i !=  all_cmp[clsn].LRF_number.size() - 1)
            {
                std::cout << ",";
            }
            if (predominance[all_cmp[clsn].LRF_number[i] - 1] < predominance[replace_lrf_number - 1])
            {
                replace_lrf_number = all_cmp[clsn].LRF_number[i];
            }
        }
        std::cout << "\r" << "\033[30C";
        std::cout << "calc " << abs(all_cmp[clsn].calc) << "\r" << "\033[39C";
        if (all_cmp[clsn].mode == 1 || all_cmp[clsn].mode == 2)
        {
            std::cout << "calc_r " << abs(all_cmp[clsn].calc_righter) << " " <<  "calc_l " << abs(all_cmp[clsn].calc_lefter);
        }
        std::cout <<  "\r" << "\033[60C";
        if (abs(all_cmp[clsn].calc) > 5 && new_put_lrf_number != 100)
        {
            std::cout << "[put a new portable" << new_put_lrf_number << " at f" << all_cmp[clsn].feature << "]" << std::endl;
        }
        else if (abs(all_cmp[clsn].calc) > 5 && new_put_lrf_number == 100)
        {
            std::cout << "[please wait]" << std::endl;
        }
        else if (abs(all_cmp[clsn].calc_righter) < 5 && ExeTime - all_cmp[clsn].time > 15.0 * 60.0)
        {
            std::cout << "[remove a portable" << replace_lrf_number << " at f" << all_cmp[clsn].feature << "]"  << std::endl;
        }
        else if (abs(all_cmp[clsn].calc_lefter) < 5 && ExeTime - all_cmp[clsn].time > 15.0 * 60.0)
        {
            std::cout << "[remove a portable" << replace_lrf_number << " at f" << all_cmp[clsn].feature << "]"  << std::endl;
        }
        else
        {
            std::cout << "[no comment]" << ExeTime - all_cmp[clsn].time << std::endl;
        }
    }
    pthread_mutex_unlock(&mutex_target);
    predominance.clear();
}


void PsenCallback0(const tms_msg_ss::pot_tracking_psens::ConstPtr &psens)
{
    int ID;
    int flag;
    float output_x;
    float output_y;
    pthread_mutex_lock(&mutex_target);
    for (int i = 0; i < psens->pot_tracking_psen.size(); i++)
    {
        char str[20];
        ID = psens->pot_tracking_psen[i].id;
        flag = psens->pot_tracking_psen[i].flag;
        output_x = psens->pot_tracking_psen[i].x;
        output_y = psens->pot_tracking_psen[i].y;
        all_cnt[0].righter = psens->pot_tracking_psen[i].righter;
        all_cnt[0].lefter = psens->pot_tracking_psen[i].lefter;
        sprintf(str, "psen/psen_txt0/ID%d.txt", ID);
        std::ofstream strw;
        strw.open(str, std::ofstream::out | std::ofstream::app);
        strw << output_x << " " << output_y  << " " << flag << std::endl;
    }

    pthread_mutex_unlock(&mutex_target);
}

void PsenCallback1(const tms_msg_ss::pot_tracking_psens::ConstPtr &psens)
{
    int ID;
    float output_x;
    float output_y;
    pthread_mutex_lock(&mutex_target);
    for (int i = 0; i < psens->pot_tracking_psen.size(); i++)
    {
        char str[20];
        ID = psens->pot_tracking_psen[i].id;
        output_x = psens->pot_tracking_psen[i].x;
        output_y = psens->pot_tracking_psen[i].y;
        all_cnt[1].righter = psens->pot_tracking_psen[i].righter;
        all_cnt[1].lefter  = psens->pot_tracking_psen[i].lefter;
        sprintf(str, "psen/psen_txt1/ID%d.txt", ID);
        std::ofstream strw;
        strw.open(str, std::ofstream::out | std::ofstream::app);
        strw << output_x << " " << output_y << std::endl;
    }

    pthread_mutex_unlock(&mutex_target);
}

void PsenCallback2(const tms_msg_ss::pot_tracking_psens::ConstPtr &psens)
{
    int ID;
    float output_x;
    float output_y;
    pthread_mutex_lock(&mutex_target);
    for (int i = 0; i < psens->pot_tracking_psen.size(); i++)
    {
        char str[20];
        ID = psens->pot_tracking_psen[i].id;
        output_x = psens->pot_tracking_psen[i].x;
        output_y = psens->pot_tracking_psen[i].y;
        all_cnt[2].righter = psens->pot_tracking_psen[i].righter;
        all_cnt[2].lefter  = psens->pot_tracking_psen[i].lefter;
        sprintf(str, "psen/psen_txt2/ID%d.txt", ID);
        std::ofstream strw;
        strw.open(str, std::ofstream::out | std::ofstream::app);
        strw << output_x << " " << output_y << std::endl;
    }

    pthread_mutex_unlock(&mutex_target);
}

void PsenCallback3(const tms_msg_ss::pot_tracking_psens::ConstPtr &psens)
{
    int ID;
    float output_x;
    float output_y;
    pthread_mutex_lock(&mutex_target);
    for (int i = 0; i < psens->pot_tracking_psen.size(); i++)
    {
        char str[20];
        ID = psens->pot_tracking_psen[i].id;
        output_x = psens->pot_tracking_psen[i].x;
        output_y = psens->pot_tracking_psen[i].y;
        all_cnt[3].righter = psens->pot_tracking_psen[i].righter;
        all_cnt[3].lefter  = psens->pot_tracking_psen[i].lefter;
        sprintf(str, "psen/psen_txt3/ID%d.txt", ID);
        std::ofstream strw;
        strw.open(str, std::ofstream::out | std::ofstream::app);
        strw << output_x << " " << output_y << std::endl;
    }

    pthread_mutex_unlock(&mutex_target);
}

void PsenCallback4(const tms_msg_ss::pot_tracking_psens::ConstPtr &psens)
{
    int ID;
    float output_x;
    float output_y;
    pthread_mutex_lock(&mutex_target);
    for (int i = 0; i < psens->pot_tracking_psen.size(); i++)
    {
        char str[20];
        ID = psens->pot_tracking_psen[i].id;
        output_x = psens->pot_tracking_psen[i].x;
        output_y = psens->pot_tracking_psen[i].y;
        all_cnt[4].righter = psens->pot_tracking_psen[i].righter;
        all_cnt[4].lefter  = psens->pot_tracking_psen[i].lefter;
        sprintf(str, "psen/psen_txt4/ID%d.txt", ID);
        std::ofstream strw;
        strw.open(str, std::ofstream::out | std::ofstream::app);
        strw << output_x << " " << output_y << std::endl;
    }

    pthread_mutex_unlock(&mutex_target);
}

void PsenCallback5(const tms_msg_ss::pot_tracking_psens::ConstPtr &psens)
{
    int ID;
    float output_x;
    float output_y;
    pthread_mutex_lock(&mutex_target);
    for (int i = 0; i < psens->pot_tracking_psen.size(); i++)
    {
        char str[20];
        ID = psens->pot_tracking_psen[i].id;
        output_x = psens->pot_tracking_psen[i].x;
        output_y = psens->pot_tracking_psen[i].y;
        all_cnt[5].righter = psens->pot_tracking_psen[i].righter;
        all_cnt[5].lefter  = psens->pot_tracking_psen[i].lefter;
        sprintf(str, "psen/psen_txt/ID%d.txt", ID);
        std::ofstream strw;
        strw.open(str, std::ofstream::out | std::ofstream::app);
        strw << output_x << " " << output_y << std::endl;
    }

    pthread_mutex_unlock(&mutex_target);
}

int main( int argc, char **argv )
{
    std::cout << "---pot_psen_manager start---" << std::endl;
    all_cnt.resize(6);
    pthread_t thread_p;
    ros::MultiThreadedSpinner spinner(4);

    ros::init(argc, argv, "pot_psen_manager");
    ros::NodeHandle n;
    ros::Subscriber sub0 = n.subscribe("tracking_psen0" , 100, PsenCallback0);
    ros::Subscriber sub1 = n.subscribe("tracking_psen1" , 100, PsenCallback1);
    ros::Subscriber sub2 = n.subscribe("tracking_psen2" , 100, PsenCallback2);
    ros::Subscriber sub3 = n.subscribe("tracking_psen3" , 100, PsenCallback3);
    ros::Subscriber sub4 = n.subscribe("tracking_psen4" , 100, PsenCallback4);
    ros::Subscriber sub5 = n.subscribe("tracking_psen5" , 100, PsenCallback5);

    if ( pthread_create( &thread_p, NULL, Management, NULL) )
    {
        printf("error creating thread.");
        abort();
    }
    g_StartTime = ros::Time::now().toSec();
    spinner.spin(); // spin() will not return until the node has been shutdown

    ros::waitForShutdown();

    if ( pthread_join( thread_p, NULL) )
    {
        printf("error joining thread.");
        abort();
    }

    return 0;
}
