#include <iostream>
#include "livox.h"

int main(int argc, const char *argv[])
{
    int d = livox(argc, argv);
    std::cout << d << std::endl;

    return 0;
}
