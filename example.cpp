#include <iostream>
#include <fstream>

#include "PID.cpp"


using namespace std;

int main() {

    double u, y_d, y, kp, ki, kd, limit;

    y_d = 50;
    y = 40;
    kp = 0.1;
    ki = 0.1;
    kd = 0.1;
    limit = 50;


    // simple PID
    // initialize the pid
    PID p;

    int n = 0;
    while(1){
        // compute the pid output
    u = p.pid_simple(y_d,y,kp,ki,kd,limit);
    n++;
    // cout << p.error << endl;
    if (abs(p.error) < 1e-2) { // break if the error is in certain margin 
        // cout << "y_des achieved" << endl;
        break;
    }
    if(n==500) { // to control the infinite loop situations
    cout << "break"<<endl;
    break;
    }
    y = u;
    }

    cout << "With PID simple" << endl;
    cout << "desired_output = " << y_d << endl;
    cout << "current_output = " << y << endl;
    cout << "took " << n << " attempts to reach solutuion\n" << endl;

    // digital pid
    PID pd;

    y = 40;

    // to plot the response
    ofstream writeToFile;
    writeToFile.open("test.csv", ios_base::out | ios_base::trunc);

    if (writeToFile.is_open()) {
        writeToFile << "time" << "," << "output" << endl;
    }

    n = 0;
    while(1){
        // compute the pid output
    u = pd.pid_digital(y_d,y,kp,ki,kd,limit);

    if (writeToFile.is_open()) {
        writeToFile << n << "," << u << endl;
    }

    n++;

    // cout << pd.error << endl;
    if (abs(pd.error) < 1e-2) { // break if the error is in certain margin 
        // cout << "y_des achieved" << endl;
        break;
    }
    if(n==500) { // to control the infinite loop situations
        cout << "break"<<endl;
        break;
    }
    
    y = u;
    }

    writeToFile.close();

    cout << "With PID Digital" << endl;
    cout << "desired_output = " << y_d << endl;
    cout << "current_output = " << y << endl;
    cout << "took " << n << " attempts to reach solutuion" << endl;

    return 0;
}
