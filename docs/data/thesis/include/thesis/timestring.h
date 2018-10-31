/**
 * @author Carsten Könemann
 */
#ifndef __TIMESTRING__
#define __TIMESTRING__

#include <ctime>

namespace timestring
{
  inline std::string get_timestring()
  {
    // Create logfile name
    std::stringstream stream;
    // Current time and date
    time_t now = time(0);
    tm* ltm = localtime(&now);
    stream << 1900 + ltm->tm_year << "_";
    stream << 1    + ltm->tm_mon  << "_";
    stream <<        ltm->tm_mday << "_";
    stream << 1    + ltm->tm_hour << "_";
    stream << 1    + ltm->tm_min  << "_";
    stream << 1    + ltm->tm_sec;
    // Done
    return stream.str();
  }
} // namespace timestring

#endif //__TIMESTRING__
