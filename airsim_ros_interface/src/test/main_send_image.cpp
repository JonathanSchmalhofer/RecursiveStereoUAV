#include <iostream>
#include "lodepng.h"

int showfile(const char* filename)
{
  std::cout << "showing " << filename << std::endl;

  std::vector<unsigned char> buffer, image;
  lodepng::load_file(buffer, filename); //load the image file with given filename
  unsigned w, h;
  unsigned error = lodepng::decode(image, w, h, buffer); //decode the png

  //stop if there is an error
  if(error)
  {
    std::cout << "decoder error " << error << ": " << lodepng_error_text(error) << std::endl;
    return 0;
  }

  return 0;
}


int main(int argc, const char *argv[])
{
    // argc should be 2 for correct execution
    if ( argc != 2 )
    {
        // We print argv[0] assuming it is the program name
        std::cout << "usage: " << argv[0] << " <filename>\n";
    }
    else
    {
        // We assume argv[1] is a filename to open
        for(int i = 1; i < argc; i++)
        {
            if(showfile(argv[i])) return 0;
        }
    }

    
    
    return 0;
}
