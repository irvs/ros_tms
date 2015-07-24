#pragma once

#include <fstream>
#include  <yaml-cpp/yaml.h>


class Config{
	public:
		static Config* is(){
			static Config* instance = new Config;
			return instance;
		}
		Config(){
			// set default value
			n_of_particles = 100;
			max_lrf_range=   5.6;
      min_lrf_range=   0.2;
			target_area[0]= -4.5; 
			target_area[1]= -4.5; 
			target_area[2]=  4.5; 
			target_area[3]=  4.5; 
			m_sigma       =100.0; 
			pos_noise     = 0.05;   
			vel_noise     = 0.05;     
			particle_area = 0.1;
			update=10000;
			
			max_vel = 100.0;

			m_max_ID = 100;
			m_min_distance = 1000.0;	// 1000mm------------------------
			m_initial_dist = 500.0;		// 500mm-------------------------
			
			std::string filepath = "/home/kurt/catkin_ws/src/ros_tms/tms_ss/tms_ss_pot/config.yaml";
			std::cout <<  "filepath " << filepath << std::endl;
			try{
				YAML::Node config = YAML::LoadFile(filepath);

				if( config["n_of_particles"] ) n_of_particles = config["n_of_particles"].as<int>();
				if( config["update"] ) update = config["update"].as<int>();
				if( config["max_lrf_range"] ) max_lrf_range = config["max_lrf_range"].as<double>();
        if( config["min_lrf_range"] ) min_lrf_range = config["min_lrf_range"].as<double>();
				if( config["target_area"] ){
					if( config["target_area"].IsSequence() ){
						for (int i=0;i<4;i++) target_area[i] = config["target_area"][i].as<double>();
					}
					else{
						target_area[0] = -config["target_area"].as<double>();
						target_area[1] = -config["target_area"].as<double>();
						target_area[2] = config["target_area"].as<double>();
						target_area[3] = config["target_area"].as<double>();
					}
				}
				if( config["m_sigma"] ) m_sigma = config["m_sigma"].as<double>();
				if( config["pos_noise"] ) pos_noise = config["pos_noise"].as<double>();
				if( config["vel_noise"] ) vel_noise = config["vel_noise"].as<double>();
				if( config["particle_area"] ) particle_area = config["particle_area"].as<double>();
				if( config["max_vel"] ) max_vel = config["max_vel"].as<double>();

				if( config["m_max_ID"] ) m_max_ID = config["m_max_ID"].as<int>();
				if( config["m_min_distance"] ) m_min_distance = config["m_min_distance"].as<double>();
				if( config["m_initial_dist"] ) m_initial_dist = config["m_initial_dist"].as<double>();
				
				//std::cout << "max_vel" << max_vel << std::endl; 

			}
			catch(YAML::BadFile& e) {
				std::cerr <<  e.what() << " or Bad file path: /current/" <<  filepath << std::endl;
			}
			catch(YAML::KeyNotFound& e) {
				std::cerr <<  e.what() << std::endl;
			}
		}
		int n_of_particles;
		int update;
		double max_lrf_range;
    double min_lrf_range;
		double target_area[4];
		double m_sigma;
		double pos_noise;
		double vel_noise;
		double particle_area;
		
		double max_vel;
		int m_max_ID;
		double m_min_distance;	
		double m_initial_dist;
};
