#ifndef __PAIS_LOG_MANAGER_H__
#define __PAIS_LOG_MANAGER_H__

#include <iostream>
#include <fstream>
#include <stdarg.h>

using namespace std;

namespace PAIS {
	class LogManager {
	private:
		static ofstream *instance;
		static bool create();
	public:
		// push log message
		static void log(const char *message, ...);
		// push warning message
		static void warning(const char *message, ...);
		// push error message
		static void error(const char *message, ...);
		// close log file
		static void close();
	};
}

#endif