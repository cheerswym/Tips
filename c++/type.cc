#include<iostream>
//#include<cstdint>
using namespace std;
 
int main(){
	int8_t a = 'a';
	int16_t b = 100;
	cout << sizeof(long)*8 << endl;
	cout << sizeof(long long)*8 << endl;
	cout << sizeof(int)*8 << endl;
	cout << sizeof(int8_t)*8 << endl;
	cout << sizeof(int16_t)*8 << endl;
	cout << sizeof(int32_t)*8 << endl;
	cout << sizeof(int64_t)*8 << endl;
 
	cout << a << endl;
	cout << b << endl;
 
	cout << "over" << endl;
}
