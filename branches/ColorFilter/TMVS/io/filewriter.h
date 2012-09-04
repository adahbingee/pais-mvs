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
	class Camera;
	class Patch;

	class FileWriter {
	private:
		static void writeMvsConfig(fstream &file, const MVS &mvs);
		static void writeCamera(fstream &file, const Camera &camera);
		static void writePatch(fstream &file, const Patch &patch);
		static void writeColorDistPatch(fstream &file, char *fileNameGT, const Patch &patch);
		static void writeVec(fstream &file, const Vec4d &vec);
		static void writeVec(fstream &file, const Vec3d &vec);
		static void writeVec(fstream &file, const Vec2d &vec);

	public:
		static void writeMVS(const char *fileName, const MVS &mvs);
		static void writeColorDistMVS(const char *fileName, char *fileNameGT, const MVS &mvs); // added by Chaody, 2012.Sep.04
		static void writePLY(const char *fileName, const MVS &mvs);
		static void wirtePSR(const char *fileName, const MVS &mvs);
		static void writeDeletedPatchMVS(const char *fileName, const MVS &mvs);
		static void writeDeletedPatchPLY(const char *fileName, const MVS &mvs);
	};
};

#endif