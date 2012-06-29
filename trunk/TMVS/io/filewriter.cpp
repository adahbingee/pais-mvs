#include "filewriter.h"

void FileWriter::writeMvsConfig(fstream &file, const MVS &mvs) {
	MvsConfig config = (MvsConfig) mvs;
	file.write((char*) &config, sizeof(MvsConfig));
	/*
	// write cell size
	file.write((char*) &mvs.cellSize, sizeof(int));
	// write patch radius
	file.write((char*) &mvs.patchRadius, sizeof(int));
	// write minimum visible camera number
	file.write((char*) &mvs.minCamNum, sizeof(int));
	// write intensity variation in patch for LOD
	file.write((char*) &mvs.textureVariation, sizeof(double));
	// write visible camera correlation for expand visible camera
	file.write((char*) &mvs.visibleCorrelation, sizeof(double));
	// write minimum patch correlation when filtering patch visible camera
	file.write((char*) &mvs.minCorrelation, sizeof(double));
	// write LOD ratio
	file.write((char*) &mvs.lodRatio, sizeof(double));
	// write minimum LOD
	file.write((char*) &mvs.minLOD, sizeof(int));
	// write maximum LOD
	file.write((char*) &mvs.maxLOD, sizeof(int));
	// write maximum cell patch number
	file.write((char*) &mvs.maxCellPatchNum, sizeof(int));
	// write patch distance wieghting
	file.write((char*) &mvs.distWeighting, sizeof(double));
	// wirte patch difference weighting
	file.write((char*) &mvs.diffWeighting, sizeof(double));
	// write neighbor radius
	file.write((char*) &mvs.neighborRadius, sizeof(double));
	// write minimum region ratio
	file.write((char*) &mvs.minRegionRatio, sizeof(double));
	// write depth range scalar (pixel)
	file.write((char*) &mvs.depthRangeScalar, sizeof(double));
	// PSO parameter
	// particle number
	file.write((char*) &mvs.particleNum, sizeof(int));
	// maximum iteration number
	file.write(((char*) &mvs.maxIteration), sizeof(int));
	*/
}

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

void FileWriter::writeCamera(fstream &file, const PAIS::Camera &camera) {
	const int fileNameLength      = (int) string(camera.getFileName()).size();
	const Vec2d &focal            = camera.getFocalLength();
	const Vec4d &quaternion       = camera.getQuaternion();
	const Vec3d &center           = camera.getCenter();
	const double radialDistortion = camera.getRadialDistortion();
	const Vec2d &principle        = camera.getPrinciplePoint();
	// write image file name length (int)
	file.write((char*) &fileNameLength, sizeof(int));
	// write image file name (char * length)
	file.write(camera.getFileName(), fileNameLength);
	// write camera center 
	writeVec(file, center);
	// wirte camera focal length
	writeVec(file, focal);
	// write principle point
	writeVec(file, principle);
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
	file << "MVS_V3" << endl;

	// write MVS config
	writeMvsConfig(file, mvs);

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
	const map<int, Patch> &patches = mvs.getPatches();
	map<int, Patch>::const_iterator it;
	ofstream file;
	file.open(fileName, ofstream::out);
	if ( !file.is_open() ) {
		printf("Can't open file %s\n", fileName);
		return;
	}

	file << "ply" << endl;
	file << "format ascii 1.0"             << endl;
	file << "element vertex " << patches.size() << endl;
	file << "property float x"             << endl;
	file << "property float y"             << endl;
	file << "property float z"             << endl;
	file << "property float nx"             << endl;
	file << "property float ny"             << endl;
	file << "property float nz"             << endl;
	file << "property uchar diffuse_red"   << endl;
	file << "property uchar diffuse_green" << endl;
	file << "property uchar diffuse_blue"  << endl;
	file << "end_header"                   << endl;

	for (it = patches.begin(); it != patches.end(); ++it) {
		const Patch &pth = it->second;
		const Vec3d &p   = pth.getCenter();
		const Vec3d &n   = pth.getNormal();
		const Vec3b &c   = pth.getColor();
		file << p[0] << " " << p[1] << " " << p[2] << " ";
		file << n[0] << " " << n[1] << " " << n[2] << " ";
		file << int(c[2]) << " " << int(c[1]) << " " << int(c[0]) << endl;
	}

	file.close();
}

void FileWriter::wirtePSR(const char *fileName, const MVS &mvs) {
	const map<int, Patch> &patches = mvs.getPatches();
	map<int, Patch>::const_iterator it;
	ofstream file;
	file.open(fileName, ofstream::binary);
	if ( !file.is_open() ) {
		printf("Can't open file %s\n", fileName);
		return;
	}

	for (it = patches.begin(); it != patches.end(); ++it) {
		const Patch &pth = it->second;
		const Vec3d &p   = pth.getCenter();
		const Vec3d &n   = pth.getNormal();
		float num;
		num = p[0];
		file.write((char*) &num, sizeof(float));
		num = p[1];
		file.write((char*) &num, sizeof(float));
		num = p[2];
		file.write((char*) &num, sizeof(float));
		num = n[0];
		file.write((char*) &num, sizeof(float));
		num = n[1];
		file.write((char*) &num, sizeof(float));
		num = n[2];
		file.write((char*) &num, sizeof(float));
	}

	file.close();
}