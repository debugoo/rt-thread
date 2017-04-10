#include <iostream>

using namespace std;
#include "CppUTest/CommandLineTestRunner.h"

int main(int ac, char** av)
{
    cout << "Hello world!" << endl;
    return CommandLineTestRunner::RunAllTests(ac, av);

    return 0;
}
