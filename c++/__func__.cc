//demo.cpp

#include <iostream>

using namespace std;

const char* hello() {return __func__;}
const char* world() {return __func__;}
//====================1========================
struct TestStruct {
    TestStruct (): name(__func__){}
    const char* name;
};
//====================2========================
#define LOG(...) {\
                    fprintf(stderr, "%s: Line %d:\t", __FILE__, __LINE__);\
                    fprintf(stderr, __VA_ARGS__);\
                    fprintf(stderr, "\n");\
                }
//====================3========================
int main(int argc, char **args){
    cout << "Standard Clib: " << __STDC_HOSTED__ << endl;
    cout << "Standard C: "    << __STDC__ << endl;

    cout << "ISO/IEC " << __STDC_ISO_10646__ << endl;
    cout << hello() << ", " << world() << endl;
    //====================1========================
    TestStruct ts;
    cout << ts.name << endl;
    //====================2========================

    int x = 3;
    int y=4;
    LOG("x = %d,%d", x, y);

    //====================3========================




    return 0;
}

// gcc -std=c++11 demo.cpp -omain
