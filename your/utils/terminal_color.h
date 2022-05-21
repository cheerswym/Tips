#ifndef ONBOARD_UTILS_TERMINAL_COLOR_H_
#define ONBOARD_UTILS_TERMINAL_COLOR_H_

namespace ansi {
inline std::ostream &reset(std::ostream &os) { return os << "\033[0m"; }

inline std::ostream &grey(std::ostream &os) { return os << "\033[30m"; }

inline std::ostream &red(std::ostream &os) { return os << "\033[31m"; }

inline std::ostream &green(std::ostream &os) { return os << "\033[32m"; }

inline std::ostream &yellow(std::ostream &os) { return os << "\033[33m"; }

inline std::ostream &blue(std::ostream &os) { return os << "\033[34m"; }

inline std::ostream &magenta(std::ostream &os) { return os << "\033[35m"; }

inline std::ostream &cyan(std::ostream &os) { return os << "\033[36m"; }

inline std::ostream &white(std::ostream &os) { return os << "\033[37m"; }

inline std::ostream &bg_grey(std::ostream &os) { return os << "\033[40m"; }

inline std::ostream &bg_red(std::ostream &os) { return os << "\033[41m"; }

inline std::ostream &bg_green(std::ostream &os) { return os << "\033[42m"; }

inline std::ostream &bg_yellow(std::ostream &os) { return os << "\033[43m"; }

inline std::ostream &bg_blue(std::ostream &os) { return os << "\033[44m"; }

inline std::ostream &bg_magenta(std::ostream &os) { return os << "\033[45m"; }

inline std::ostream &bg_cyan(std::ostream &os) { return os << "\033[46m"; }

inline std::ostream &bg_white(std::ostream &os) { return os << "\033[47m"; }

inline std::ostream &bold(std::ostream &os) { return os << "\033[1m"; }

inline std::ostream &dark(std::ostream &os) { return os << "\033[2m"; }

inline std::ostream &italic(std::ostream &os) { return os << "\033[3m"; }

inline std::ostream &underline(std::ostream &os) { return os << "\033[4m"; }

inline std::ostream &blink(std::ostream &os) { return os << "\033[5m"; }

inline std::ostream &reverse(std::ostream &os) { return os << "\033[7m"; }

inline std::ostream &crossed(std::ostream &os) { return os << "\033[9m"; }

inline std::ostream &concealed(std::ostream &os) { return os << "\033[8m"; }
}  // namespace ansi

#endif  // ONBOARD_UTILS_TERMINAL_COLOR_H_
