// endl example
#include <iostream>     // std::cout, std::end
#include <fstream>      // std::filebuf
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */

int main () {

  int a=100;
  double b=3.14;

  std::cout << a;
  std::cout << std::endl;              // manipulator inserted alone
  std::cout << b << std::endl << a*b;  // manipulator in concatenated insertion
  std::endl (std::cout);               // endl called as a regular function



  std::filebuf fb;
  fb.open ("test.txt",std::ios::out);
  std::ostream os(&fb);
  os << RED;
  //std::cout << os.str();
  std::endl(os);
  
  fb.close();

  std::cout <<"1111111111111111"<< std::endl<< RED << RESET;
  return 0;
}
