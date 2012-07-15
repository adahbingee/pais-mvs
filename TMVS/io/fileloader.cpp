#include "fileloader.h"

FileLoader::FileLoader(void) {}
FileLoader::~FileLoader(void) {}

void FileLoader::getDir(const char *fileName, char *path) {
	size_t found = string(fileName).find_last_of("/\\");
	string(fileName).copy(path, found+1, 0);
    path[found+1] = '\0';
}

PAIS::Camera FileLoader::loadNvmCamera(ifstream &file, const char* path) {
	// camera information
	string fileName = path;
	Vec2d focal;
	Vec4d quaternion;
	Vec3d center;
	double radialDistortion;

	char strbuf[STRING_BUFFER_LENGTH];
	char *strip;

	file.getline(strbuf, STRING_BUFFER_LENGTH);
	// image file name
    strip = strtok(strbuf, DELIMITER);
	fileName.append(strip);

	// focal length
	strip = strtok(NULL, DELIMITER);
	focal[0] = atof(strip);
	focal[1] = atof(strip);

	// quaternion rotation
    strip = strtok(NULL, DELIMITER); // k
    quaternion[0] = atof(strip);
    strip = strtok(NULL, DELIMITER); // wx                  
    quaternion[1] = atof(strip);
    strip = strtok(NULL, DELIMITER); // wy
    quaternion[2] = atof(strip);
    strip = strtok(NULL, DELIMITER); // wz
    quaternion[3] = atof(strip);

	// 3D camera center in world coordinate
    Vec3d cameraCenter;
    strip = strtok(NULL, DELIMITER); // cx
    center[0] = atof(strip);
    strip = strtok(NULL, DELIMITER); // cy
    center[1] = atof(strip);
    strip = strtok(NULL, DELIMITER); // cz
    center[2] = atof(strip);

	// radial distortion parameters
    strip = strtok(NULL, DELIMITER);
	radialDistortion = atof(strip);

	return Camera(fileName.c_str(), focal, Vec2d(-1, -1), quaternion, center, radialDistortion);
}

PAIS::Camera FileLoader::loadNvm2Camera(ifstream &file, const char* path) {
	// camera information
	string fileName = path;
	Vec2d focal;
	Vec2d principlePoint;
	Vec4d quaternion;
	Vec3d center;

	char strbuf[STRING_BUFFER_LENGTH];
	char *strip;

	file.getline(strbuf, STRING_BUFFER_LENGTH);
	// image file name
    strip = strtok(strbuf, DELIMITER);
	fileName.append(strip);

	// focal length
	strip = strtok(NULL, DELIMITER);
	focal[0] = atof(strip);
	strip = strtok(NULL, DELIMITER);
	focal[1] = atof(strip);

	// priciple point
	strip = strtok(NULL, DELIMITER);
	principlePoint[0] = atof(strip);
	strip = strtok(NULL, DELIMITER);
	principlePoint[1] = atof(strip);

	// quaternion rotation
    strip = strtok(NULL, DELIMITER); // k
    quaternion[0] = atof(strip);
    strip = strtok(NULL, DELIMITER); // wx                  
    quaternion[1] = atof(strip);
    strip = strtok(NULL, DELIMITER); // wy
    quaternion[2] = atof(strip);
    strip = strtok(NULL, DELIMITER); // wz
    quaternion[3] = atof(strip);

	// 3D camera center in world coordinate
    Vec3d cameraCenter;
    strip = strtok(NULL, DELIMITER); // cx
    center[0] = atof(strip);
    strip = strtok(NULL, DELIMITER); // cy
    center[1] = atof(strip);
    strip = strtok(NULL, DELIMITER); // cz
    center[2] = atof(strip);

	return Camera(fileName.c_str(), focal, principlePoint, quaternion, center, 0);
}

Patch FileLoader::loadNvmPatch(ifstream &file, const MVS &mvs) {
	Vec3d center;
	Vec3b color;
	int camNum;
	vector<int> camIdx;
	vector<Vec2d> imgPoint;

	char strbuf[STRING_BUFFER_LENGTH];
	char *strip;

	file.getline(strbuf, STRING_BUFFER_LENGTH);

	// 3D point
    strip = strtok(strbuf, DELIMITER); // x
    center[0] = atof(strip); 
    strip = strtok(NULL, DELIMITER);   // y
    center[1] = atof(strip); 
    strip = strtok(NULL, DELIMITER);   // z
    center[2] = atof(strip); 

	// RGB color
    strip = strtok(NULL, DELIMITER);
	color[2] = atoi(strip);            // r
    strip = strtok(NULL, DELIMITER);
	color[1] = atoi(strip);            // g
    strip = strtok(NULL, DELIMITER);
	color[0] = atoi(strip);            // b

	// number of measurements ( how many cameras see this point)
    strip = strtok(NULL, DELIMITER);
    camNum = atoi(strip);

	for (int i = 0; i < camNum; i++) {
		// camera index
		strip = strtok(NULL, DELIMITER);
		int idx = atoi(strip);
        camIdx.push_back(idx);

		const Camera &cam = mvs.getCameras()[idx];

		// feature index
		strip = strtok(NULL, DELIMITER);

		// 2D image point
		Vec2d p;
        strip  = strtok(NULL, DELIMITER);
		p[0] = atof(strip) + cam.getRgbImage().cols / 2;
        strip  = strtok(NULL, DELIMITER);
		p[1] = atof(strip) + cam.getRgbImage().rows / 2;
		imgPoint.push_back(p);
	}

	return Patch(center, color, camIdx, imgPoint);
}

MvsConfig FileLoader::loadMvsConfig(ifstream &file) {
	MvsConfig config;
	file.read((char*) &config, sizeof(MvsConfig));
	return config;
}

PAIS::Camera FileLoader::loadMvsCamera(ifstream &file) {
	int fileNameLength;
	char *fileName;
	Vec3d center;
	Vec2d focal;
	Vec4d quaternion;
	Vec2d principle;
	double radialDistortion;

	// read image file name length
	file.read( (char*) &fileNameLength, sizeof(int) );
	// read image file name
	fileName = new char [fileNameLength+1];
	file.read(fileName, fileNameLength);
	fileName[fileNameLength] = '\0';
	// read camera center
	loadMvsVec(file, center);
	// read camera focal length
	loadMvsVec(file, focal);
	// read camera principle point
	loadMvsVec(file, principle);
	// read rotation quaternion
	loadMvsVec(file, quaternion);
	// read radial distortion
	file.read((char*) &radialDistortion, sizeof(double));

	Camera cam(fileName, focal, principle, quaternion, center, radialDistortion);

	delete [] fileName;

	return cam;
}

Patch FileLoader::loadMvsPatch(ifstream &file) {
	vector<int> camIdx;
	int camNum;
	Vec3d center;
	Vec2d sphericalNormal;
	double fitness;
	double correlation;

	// read patch center
	loadMvsVec(file, center);
	// read patch spherical normal
	loadMvsVec(file, sphericalNormal);
	// load visible camera number
	file.read((char*) &camNum, sizeof(int));
	// load visible camera index
	for (int i = 0; i < camNum; ++i) {
		int idx;
		file.read((char*) &idx, sizeof(int));
		camIdx.push_back(idx);
	}
	// load fitness
	file.read((char*) &fitness, sizeof(double));
	// load correlation
	file.read((char*) &correlation, sizeof(double));
	return Patch(center, sphericalNormal, camIdx, fitness, correlation);
}

void FileLoader::loadMvsVec(ifstream &file, Vec2d &v) {
	file.read( (char*) &v[0], sizeof(double));
	file.read( (char*) &v[1], sizeof(double));
}

void FileLoader::loadMvsVec(ifstream &file, Vec3d &v) {
	file.read( (char*) &v[0], sizeof(double));
	file.read( (char*) &v[1], sizeof(double));
	file.read( (char*) &v[2], sizeof(double));
}

void FileLoader::loadMvsVec(ifstream &file, Vec4d &v) {
	file.read( (char*) &v[0], sizeof(double));
	file.read( (char*) &v[1], sizeof(double));
	file.read( (char*) &v[2], sizeof(double));
	file.read( (char*) &v[3], sizeof(double));
}

void FileLoader::loadNVM(const char *fileName, MVS &mvs) {
	vector<Camera>  &cameras = mvs.cameras;
	map<int, Patch> &patches = mvs.patches;

	// reset container
	cameras.clear();
	patches.clear();

	// open nvm file
	ifstream file(fileName, ifstream::in);

	if ( !file.is_open() ) {
		printf("Can't open NVM file: %s\n", fileName);
		return;
	}

	// get file path
	char filePath [MAX_FILE_NAME_LENGTH];
	getDir(fileName, filePath);

	char *strip = NULL;
	char strbuf[STRING_BUFFER_LENGTH];
	int num;
	bool loadCamera = false;
	bool loadPatch  = false;

	while ( !file.eof() ) {

		file.getline(strbuf, STRING_BUFFER_LENGTH);
		strip = strtok(strbuf, DELIMITER);

		if (strip == NULL) continue; // skip blank line

		// start load camera
		if (strcmp(strip, "NVM_V3") == 0) {
			loadCamera = true;
			continue;
		}

		if (loadCamera) {
			// get number of cameras
			strip = strtok(strbuf, DELIMITER);
			num = atoi(strip);

			cameras.reserve(num);
			for (int i = 0; i < num; i++) {
				printf("\rloading cameras: %d / %d", i+1, num);
				cameras.push_back( loadNvmCamera(file, filePath) );
			}
			printf("\n");
			loadCamera = false;
			loadPatch  = true;

			continue;
		}

		if (loadPatch) {
			// get number of points
			strip = strtok(strbuf, DELIMITER);
			num = atoi(strip);

			for (int i = 0; i < num; i++) {
				printf("\rloading patches: %d / %d", i+1, num);
				Patch p = loadNvmPatch(file, mvs);
				patches.insert( pair<int, Patch>(p.getId(), p) );
			}
			printf("\n");
			loadPatch = false;

			break;
		}
	}

	file.close();
}

void FileLoader::loadNVM2(const char *fileName, MVS &mvs) {
	vector<Camera>  &cameras = mvs.cameras;
	map<int, Patch> &patches = mvs.patches;

	// reset container
	cameras.clear();
	patches.clear();

	// open nvm file
	ifstream file(fileName, ifstream::in);

	if ( !file.is_open() ) {
		printf("Can't open NVM file: %s\n", fileName);
		return;
	}

	// get file path
	char filePath [MAX_FILE_NAME_LENGTH];
	getDir(fileName, filePath);

	char *strip = NULL;
	char strbuf[STRING_BUFFER_LENGTH];
	int num;
	bool loadCamera = false;
	bool loadPatch  = false;

	while ( !file.eof() ) {

		file.getline(strbuf, STRING_BUFFER_LENGTH);
		strip = strtok(strbuf, DELIMITER);

		if (strip == NULL) continue; // skip blank line

		// start load camera
		if (strcmp(strip, "NVM_V3") == 0) {
			loadCamera = true;
			continue;
		}

		if (loadCamera) {
			// get number of cameras
			strip = strtok(strbuf, DELIMITER);
			num = atoi(strip);

			cameras.reserve(num);
			for (int i = 0; i < num; i++) {
				printf("\rloading cameras: %d / %d", i+1, num);
				cameras.push_back( loadNvm2Camera(file, filePath) );
			}
			printf("\n");
			loadCamera = false;
			loadPatch  = true;

			continue;
		}

		if (loadPatch) {
			// get number of points
			strip = strtok(strbuf, DELIMITER);
			num = atoi(strip);

			for (int i = 0; i < num; i++) {
				printf("\rloading patches: %d / %d", i+1, num);
				Patch p = loadNvmPatch(file, mvs);
				patches.insert( pair<int, Patch>(p.getId(), p) );
			}
			printf("\n");
			loadPatch = false;

			break;
		}
	}

	file.close();
}

void FileLoader::loadMVS(const char *fileName, MVS &mvs) {
	vector<Camera>  &cameras = mvs.cameras;
	map<int, Patch> &patches = mvs.patches;

	// reset container
	cameras.clear();
	patches.clear();

	// open mvs file
	ifstream file(fileName, ifstream::in | ifstream::binary);

	if ( !file.is_open() ) {
		printf("Can't open MVS file: %s\n", fileName);
		return;
	}

	char *strip = NULL;
	char strbuf[STRING_BUFFER_LENGTH];
	int num;
	bool loadCamera = false;
	bool loadPatch  = false;
	while ( !file.eof() ) {

		file.getline(strbuf, STRING_BUFFER_LENGTH);
		strip = strtok(strbuf, DELIMITER);

		if (strip == NULL) continue; // skip blank line

		// start load camera
		if (strcmp(strip, "MVS_V2") == 0) {
			loadCamera = true;
			continue;
		}

		// set config and start load camera
		if (strcmp(strip, "MVS_V3") == 0) {
			MvsConfig config = loadMvsConfig(file);
			mvs.setConfig(config);
			loadCamera = true;
			continue;
		}

		if (loadCamera) {
			strip = strtok(NULL, DELIMITER);
			num = atoi(strip);
			for (int i = 0; i < num; ++i) {
				printf("\rloading cameras: %d / %d", i+1, num);
				cameras.push_back( loadMvsCamera(file) );
			}
			printf("\n");
			loadCamera = false;
			loadPatch  = true;
			continue;
		}

		if (loadPatch) {
			strip = strtok(NULL, DELIMITER);
			num = atoi(strip);
			for (int i = 0; i < num; ++i) {
				printf("\rloading patches: %d / %d", i+1, num);
				Patch pth = loadMvsPatch(file);
				patches.insert( pair<int, Patch>(pth.getId(), pth) );
			}
			printf("\n");
			loadPatch = false;
		}
	}

	file.close();
}