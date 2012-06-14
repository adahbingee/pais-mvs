#include "filewriter.h"

void FileWriter::writeVec(fstream &file, const Vec4d &vec) {
	file.write((char*) &vec[0], sizeof(double));
	file.write((char*) &vec[1], sizeof(double));
	file.write((char*) &vec[2], sizeof(double));
	file.write((char*) &vec[3], sizeof(double));
}

void FileWriter::writeVec(fstream &file, const Vec3d &vec) {
	file.write((char*) &vec[0], sizeof(double));
	file.write((char*) &vec[1], sizeof(double));
	file.write((char*) &vec[2], sizeof(double));
}

void FileWriter::writeVec(fstream &file, const Vec2d &vec) {
	file.write((char*) &vec[0], sizeof(double));
	file.write((char*) &vec[1], sizeof(double));
}

void FileWriter::writeCamera(fstream &file, const Camera &camera) {
	const int fileNameLength      = (int) string(camera.getFileName()).size();
	const double focal            = camera.getFocalLength();
	const Vec4d &quaternion       = camera.getQuaternion();
	const Vec3d &center           = camera.getCenter();
	const double radialDistortion = camera.getRadialDistortion();
	// write image file name length (int)
	file.write((char*) &fileNameLength, sizeof(int));
	// write image file name (char * length)
	file.write(camera.getFileName(), fileNameLength);
	// write camera center 
	writeVec(file, center);
	// wirte camera focal length (double)
	file.write((char*) &focal, sizeof(double));
	// write rotation quaternion
	writeVec(file, quaternion);
	// write radial distortion
	file.write((char*) &radialDistortion, sizeof(double));
}

void FileWriter::writePatch(fstream &file, const Patch &patch) {
	const vector<int> &camIdx = patch.getCameraIndices();
	const int camNum = (int) camIdx.size();
	double fitness = patch.getFitness();
	double correaltion = patch.getCorrelation();

	// write patch center
	writeVec(file, patch.getCenter());
	// write patch spherical normal
	writeVec(file, patch.getSphericalNormal());
	// write visible camera number
	file.write((char*) &camNum, sizeof(int));
	// write visible camera index
	for (int i = 0; i < camNum; ++i) {
		file.write((char*) &camIdx[i], sizeof(int));
	}
	// write fitness
	file.write((char*) &fitness, sizeof(double));
	// write correlation
	file.write((char*) &correaltion, sizeof(double));
}

void FileWriter::writeMVS(const char *fileName, const MVS &mvs) {
	fstream file;
	file.open(fileName, fstream::out | fstream::binary);
	if ( !file.is_open() ) {
		printf("Can't write file %s\n", fileName);
	}

	// write MVS header
	file << "MVS_V2" << endl;

	// write cameras
	const int camNum = (int) mvs.getCameras().size();
	file << "CAMERAS " << camNum << endl;
	const vector<Camera> &cameras = mvs.getCameras();
	for (int i = 0; i < camNum; ++i) {
		writeCamera(file, cameras[i]);
	}

	// write patches
	const int patchNum = (int) mvs.getPatches().size();
	file << "PATCHES " << patchNum << endl;
	const map<int, Patch> &patches = mvs.getPatches();
	map<int, Patch>::const_iterator it;
	for (it = patches.begin(); it != patches.end(); ++it) {
		writePatch(file, (*it).second);
	}

	file.close();
}

void FileWriter::writePLY(const char *fileName, const MVS &mvs) {
	
}