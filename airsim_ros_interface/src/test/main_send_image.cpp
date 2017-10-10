#include <iostream>

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
    }
    
    return 0;
}
