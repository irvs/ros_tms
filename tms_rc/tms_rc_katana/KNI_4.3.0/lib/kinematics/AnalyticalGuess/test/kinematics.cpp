
#include "kinematics6M180.h"
#include "kinematics6M90G.h"
#include "kinematics6M90T.h"


#include <iostream>
#include <cstdio>
#include <stdlib.h>
#include <memory>
#include <vector>
#include <string>

#include "keyboard.h"

std::vector<std::string> explode( const std::string & in, const std::string & delim) {
	typedef std::string::size_type size_type;

	const size_type delim_len = delim.length();
	std::vector<std::string> result;

	size_type i = 0, j;
	for (;;) {
		j = in.find(delim, i);
		result.push_back(in.substr(i, j-i));
		if (j == std::string::npos)
			break;
		i = j + delim_len;
	}

	return result;
}

void DisplayHelp() {
	std::cout << "\n";
	std::cout << "-------------------------------------\n";
	std::cout << "?: Display this help\n";
	std::cout << "e: Enc2rad\n";
	std::cout << "r: Rad2enc\n";
	std::cout << "d: Direct kinematics\n";
	std::cout << "i: Inverse kinematics\n";
	std::cout << "q: Quit\n";
	std::cout << "Esc: Quit\n";
	std::cout << "-------------------------------------\n";
}


int main(int argc, char *argv[]) {

	using namespace AnaGuess;

	if (argc != 2) {
		std::cout << "-------------------------------------\n";
		std::cout << "usage: kinematics [6M180|6M90G|6M90T]\n";
		std::cout << "-------------------------------------\n";
		return 0;
	}

	std::cout << "-----------------------\n";
	std::cout << "KINEMATICS DEMO STARTED\n";
	std::cout << "-----------------------\n";

	//Kinematics objects
	Kinematics* kinematics;
	{
	char* t = argv[1];
	if ((t[0]=='6')&&(t[1]=='M')&&(t[2]=='1')&&(t[3]=='8')&&(t[4]=='0')) {
		kinematics = (Kinematics*) new Kinematics6M180();
	} else if ((t[0]=='6')&&(t[1]=='M')&&(t[2]=='9')&&(t[3]=='0')&&(t[4]=='G')) {
		kinematics = (Kinematics*) new Kinematics6M90G();
	} else if ((t[0]=='6')&&(t[1]=='M')&&(t[2]=='9')&&(t[3]=='0')&&(t[4]=='T')) {
		kinematics = (Kinematics*) new Kinematics6M90T();
	} else {
		std::cout << "-------------------------------------\n";
		std::cout << "unknown type: " << t << "\n";
		std::cout << "usage: kinematics [6M180|6M90G|6M90T]\n";
		std::cout << "-------------------------------------\n";
		return 0;
	}
	}

	std::cout << "--------------------------------------------\n";
	std::cout << "success: katana kinematics" << argv[1] << " initialized\n";
	std::cout << "--------------------------------------------\n";

	DisplayHelp();

	bool loop = true;
	int input;
	std::string strtemp;
	std::vector<std::string> strvectemp;
	double doubletemp;
	int size;
	std::vector<double> pose;
	pose.reserve(6);
	std::vector<int> encoder;
	encoder.reserve(6);
	std::vector<double> angle;
	angle.reserve(6);
	std::vector<double> start;
	start.reserve(6);

	while (loop) {
		input = _getch();
		try {
			switch (input) {
			case 27: //VK_ESCAPE
			case 'q': //VK_Q
				loop = false;
				continue;

			case '?':
				DisplayHelp();
				break;

			case 'e': //VK_E (enc2rad)
				encoder.clear();
				angle.clear();
				std::cout << "\nInsert encoder values (comma separated, without spaces): \n";
				std::cin >> strtemp;
std::cout << "> strtemp " << strtemp << "\n";
				strvectemp = explode(strtemp,",");
				size = strvectemp.size();
std::cout << "> strvectemp.size " << size << "\n";
				for (int i = 0; i < size; i++) {
					encoder.push_back(atoi(strvectemp.at(i).c_str())); // double: atof()
std::cout << "> encoder.at(" << i << ") " << encoder.at(i) << "\n";
				}
				if (!kinematics->enc2rad(angle, encoder)) {
					std::cout << "\nConverting encoders to angles failed.\n";
					break;
				}
				std::cout << "\nAngles:\n";
				size = angle.size();
std::cout << "> angle.size " << size << "\n"; // !!! is 0
				for (int i = 0; i < size; i++) {
					std::cout << angle.at(i);
					if (i != size - 1)
						std::cout << ",";
				}
				std::cout << "\n";
				break;

			case 'r': //VK_R (rad2enc)
				break;

			case 'd': //VK_D (direct kinematics)
				std::cout << "\nInsert angle values: \n";
				angle.clear();
				pose.clear();
				std::cout << "Axis 1: ";
				std::cin >> doubletemp;
				angle.push_back(doubletemp);
				std::cout << "Axis 2: ";
				std::cin >> doubletemp;
				angle.push_back(doubletemp);
				std::cout << "Axis 3: ";
				std::cin >> doubletemp;
				angle.push_back(doubletemp);
				std::cout << "Axis 4: ";
				std::cin >> doubletemp;
				angle.push_back(doubletemp);
				std::cout << "Axis 5: ";
				std::cin >> doubletemp;
				angle.push_back(doubletemp);
				std::cout << "Axis 6: ";
				std::cin >> doubletemp;
				angle.push_back(doubletemp);
				if (!kinematics->directKinematics(pose, angle)) {
					std::cout << "\nCalculating direct kinematics failed.\n";
					break;
				}
				std::cout.precision(6);
				std::cout << "\n------------------------------------\n";
				std::cout << "X:     " << pose[0] << "\n";
				std::cout << "Y:     " << pose[1] << "\n";
				std::cout << "Z:     " << pose[2] << "\n";
				std::cout << "phi:   " << pose[3] << "\n";
				std::cout << "theta: " << pose[4] << "\n";
				std::cout << "psi:   " << pose[5] << "\n";
				std::cout << "------------------------------------\n";
				break;

			case 'i':  //VK_I (inverse kinematics)
				std::cout << "\nInsert cartesian parameters: \n";
				angle.clear();
				pose.clear();
				std::cout << "X: ";
				std::cin >> doubletemp;
				pose.push_back(doubletemp);
				std::cout << "Y: ";
				std::cin >> doubletemp;
				pose.push_back(doubletemp);
				std::cout << "Z: ";
				std::cin >> doubletemp;
				pose.push_back(doubletemp);
				std::cout << "phi: ";
				std::cin >> doubletemp;
				pose.push_back(doubletemp);
				std::cout << "theta: ";
				std::cin >> doubletemp;
				pose.push_back(doubletemp);
				std::cout << "psi: ";
				std::cin >> doubletemp;
				pose.push_back(doubletemp);
				angle.reserve(6);
				for (int i = 0; i < 6; i++) {
					start[i] = 1.5;
				}
				try {
					if (!kinematics->inverseKinematics(angle, pose, start)) {
						std::cout << "\nCalculating inverse kinematics failed.\n";
						break;
					}
				} catch (NoSolutionException &nse) {
					std::cout << "\nNo solution found.\n";
					break;
				}
				std::cout << "\n------------------------------------\n";
				std::cout << "Axis 1: " << angle[0] << "\n";
				std::cout << "Axis 2: " << angle[1] << "\n";
				std::cout << "Axis 3: " << angle[2] << "\n";
				std::cout << "Axis 4: " << angle[3] << "\n";
				std::cout << "Axis 5: " << angle[4] << "\n";
				std::cout << "Axis 6: " << angle[5] << "\n";
				std::cout << "------------------------------------\n";
				break;

			default: //Error message
				std::cout << "\n'" << input << "' is not a valid command.\n" << std::endl;
				break;

			}
		} catch (Exception &e) {
			std::cout << "\nERROR: " << e.message() << "\n";
		}
	}
	delete kinematics;
	return 0;
}

