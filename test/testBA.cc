#include "generateData.h"

#include <iostream>

using namespace std;
int main(){

    // double d = gaussion(0, 1);

    for(int i=0; i<100; i++){
        cout << uniform(0, 100) << endl;
        cout << uniform(0.0, 100.0) << endl;
    }

    return 0;
}