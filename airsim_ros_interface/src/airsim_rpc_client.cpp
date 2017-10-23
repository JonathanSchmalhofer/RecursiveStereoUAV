//#include "StereoImageGenerator.hpp"
#include "StereoImageRpcClient.hpp"



int main(int argc, const char *argv[])
{
	// get local information if neither the name or the address is given
    
	std::cout << "Press Enter to start StereoImageGenerator" << std::endl; std::cin.get();

	StereoImageRpcClient *client = new StereoImageRpcClient();
	
	std::cout << "Finished??" << std::endl; std::cin.get();
	
	client->connect();
	
	std::cout << "Connected??" << std::endl; std::cin.get();
	
	client->setPosition(0, 0, 1, 1, 0, 0 , 1);
	
	std::cout << "Collided? Answer: " << client->hasCollided() << std::endl;
	
	client->requestStereoImagesAndDepthImage();
	
	delete client;
	
    return 0;
}
