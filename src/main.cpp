#include "BasicService.h"

int main(int argc, char** argv)
{
    if (!BasicServiceInst.init(argc, argv))
    {
        return -1;
    }

    BasicServiceInst.run();

    return 0;
}