#ifndef __PAIS_FILE_LOADER_H__
#define __PAIS_FILE_LOADER_H__

#define STRING_BUFFER_LENGTH 1024
#define DELIMITER " \t"

#include <map>
#include <fstream>

#include <opencv2\opencv.hpp>

#include "../mvs/camera.h"
#include "../mvs/patch.h"
#include "../mvs/mvs.h"

using namespace PAIS;
using namespace cv;

namespace PAIS {
	class MVS;
	class Patch;

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