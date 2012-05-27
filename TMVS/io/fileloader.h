#ifndef __PAIS_FILE_LOADER_H__
#define __PAIS_FILE_LOADER_H__

#define STRING_BUFFER_LENGTH 1024
#define DELIMITER " \t"

#include <map>
#include <fstream>

#include "../mvs/headers.h"
#include "../mvs/camera.h"
#include "../mvs/patch.h"

using namespace PAIS;

namespace PAIS {
	class FileLoader {
	private: 
		FileLoader(void);
		~FileLoader(void);

		static void   getDir(const char *fileName, char *path);
		static Camera loadNvmCamera(ifstream &file, const char* path);
		static Patch  loadNvmPatch(ifstream &file, const MVS &mvs);
		
	public:
		static void loadNVM(const char *fileName, MVS &mvs);
	};
};

#endif