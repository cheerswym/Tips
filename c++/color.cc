#include <ostream>
#include <iostream>     // std::cout, std::end
using namespace std;

namespace Color {
    enum Code {
        FG_RED      = 31,
        FG_GREEN    = 32,
        FG_BLUE     = 34,
        FG_DEFAULT  = 39,
        BG_RED      = 41,
        BG_GREEN    = 42,
        BG_BLUE     = 44,
        BG_DEFAULT  = 0
    };
    class Modifier {
        Code code;
    public:
        Modifier(Code pCode) : code(pCode) {}
        
        friend std::ostream& operator<<(std::ostream& os, const Modifier& mod) {
            return os << "\033[" << mod.code << "m";
        }
    };
}


inline std::ostream &bg_red(std::ostream &os) { 
	return os << "\033[41m"; }

int main() {
    Color::Modifier red(Color::FG_RED);
    Color::Modifier def(Color::FG_DEFAULT);
    Color::Modifier bg_red(Color::BG_RED);
    Color::Modifier bg_def(Color::BG_DEFAULT);
    std::cout << "This ->" << red << "word" << def << "<- is red." << std::endl;
    //std::cout << "This ->" << bg_red << "word" << bg_def << "<- is red." << std::endl;
    std::cout << bg_red << "testing..." << "\033[0m"<< std::endl;
}
