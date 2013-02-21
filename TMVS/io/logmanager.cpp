#define STRING_BUFFER_LENGTH 1024

#include "logmanager.h"

using namespace PAIS;

ofstream* LogManager::instance;

bool LogManager::create() {
	if (instance == NULL) {
		instance = new ofstream("log.txt", ofstream::out);
	}
	return instance->is_open();
}

void LogManager::log(const char *message, ...) {
	if ( !create() ) return;

    // put formatted string
	va_list argptr;
    va_start(argptr, message);
	char buffer [STRING_BUFFER_LENGTH];
	vsnprintf (buffer, STRING_BUFFER_LENGTH-1, message, argptr);
	va_end(argptr);

	//(*instance) << "[Log]  " << buffer << endl;
	(*instance) << buffer << endl;
}

void LogManager::warning(const char *message, ...) {
	if ( !create() ) return;

    // put formatted string
	va_list argptr;
    va_start(argptr, message);
	char buffer [STRING_BUFFER_LENGTH];
	vsnprintf (buffer, STRING_BUFFER_LENGTH-1, message, argptr);
	va_end(argptr);

	(*instance) << "[Warning] " << buffer << endl;
}

void LogManager::error(const char *message, ...) {
	if ( !create() ) return;

    // put formatted string
	va_list argptr;
    va_start(argptr, message);
	char buffer [STRING_BUFFER_LENGTH];
	vsnprintf (buffer, STRING_BUFFER_LENGTH-1, message, argptr);
	va_end(argptr);

	(*instance) << "[Error]   " << buffer << endl;
}

void LogManager::close() {
	if (instance != NULL) {
		if (instance->is_open()) instance->close();
	}
}

#ifdef STRING_BUFFER_LENGTH
	#undef STRING_BUFFER_LENGTH
#endif