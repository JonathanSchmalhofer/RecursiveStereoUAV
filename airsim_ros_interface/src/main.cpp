#include "StereoImageGenerator.hpp"
#include "yuchen\mySocket.h"
#include "yuchen\myLog.h"
#include "yuchen\myException.h"
#include "yuchen\myHostInfo.h"

myLog winLog;


void runSteroImageGenerator(int num_samples, std::string storage_path)
{
    StereoImageGenerator gen(storage_path);
    gen.generate(num_samples);
}

void runSteroImageGenerator(int argc, const char *argv[])
{
    runSteroImageGenerator(argc < 2 ? 50000 : std::stoi(argv[1]), argc < 3 ? 
        common_utils::FileSystem::combine(
            common_utils::FileSystem::getAppDataFolder(), "stereo_gen")
        : std::string(argv[2]));
}

int main(int argc, const char *argv[])
{
    #ifdef WINDOWS_XP

	// Initialize the winsock library
	WSADATA wsaData;
    winLog << "system started ..." << endl;
	winLog << endl << "initialize the winsock library ... ";

	try 
	{
		if (WSAStartup(0x101, &wsaData))
		{
			myException* initializationException = new myException(0,"Error: calling WSAStartup()");
			throw initializationException;
        }
	}
	catch(myException* excp)
	{
		excp->response();
		delete excp;
		exit(1);
	}
	winLog << "successful" << endl;

#endif
        
	// get local information if neither the name or the address is given

	winLog << endl;
	winLog << "Retrieve the localHost [CLIENT] name and address:" << endl;
    
	std::cout << "Press Enter to start StereoImageGenerator" << std::endl; std::cin.get();

	runSteroImageGenerator(argc, argv);
	
    return 0;
}
