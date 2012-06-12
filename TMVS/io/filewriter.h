#ifndef __PAIS_FILE_WRITER_H__
#define __PAIS_FILE_WRITER_H__

#include <fstream>
#include <opencv2\opencv.hpp>

#include "../mvs/patch.h"
#include "../mvs/mvs.h"

using namespace PAIS;
using namespace cv;

namespace PAIS {
	class MVS;
	class Patch;

	class FileWriter {
	private:
		static void writeCamera(fstream &file, const Camera &camera);
		static void writePatch(fstream &file, const Patch &patch);
		static void writeVec(fstream &file, const Vec4d &vec);
		static void writeVec(fstream &file, const Vec3d &vec);
		static void writeVec(fstream &file, const Vec2d &vec);

	public:
		static void writeMVS(const char *fileName, const MVS &mvs);
	};
};

#endif