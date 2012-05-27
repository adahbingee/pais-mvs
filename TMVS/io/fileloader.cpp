#include "fileloader.h"

FileLoader::FileLoader(void) {}
FileLoader::~FileLoader(void) {}

void FileLoader::getDir(const char *fileName, char *path) {
	size_t found = string(fileName).find_last_of("/\\");
	string(fileName).copy(path, found+1, 0);
    path[found+1] = '\0';
}

Camera FileLoader::loadNvmCamera(ifstream &file, const char* path) {
	// camera information
	string fileName = path;
	double focal;
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
	focal = atof(strip);

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

	return Camera(fileName.c_str(), focal, quaternion, center, radialDistortion);
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
	color[0] = atoi(strip);            // r
    strip = strtok(NULL, DELIMITER);
	color[1] = atoi(strip);            // g
    strip = strtok(NULL, DELIMITER);
	color[2] = atoi(strip);            // b

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
		p[0] = atof(strip) + cam.getImageWidth()/2.0;
        strip  = strtok(NULL, DELIMITER);
		p[1] = atof(strip) + cam.getImageHeight()/2.0;
		imgPoint.push_back(p);
	}

	return Patch(&mvs, center, color, camIdx, imgPoint);
}

void FileLoader::loadNVM(const char *fileName, MVS &mvs) {
	vector<Camera>  &cameras = mvs.getCameras();
	map<int, Patch> &patches = mvs.getPatches();

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
				cameras.push_back( loadNvmCamera(file, filePath) );
			}

			loadCamera = false;
			loadPatch  = true;

			continue;
		}

		if (loadPatch) {
			// get number of points
			strip = strtok(strbuf, DELIMITER);
			num = atoi(strip);

			for (int i = 0; i < num; i++) {
				Patch p = loadNvmPatch(file, mvs);
				patches.insert( pair<int, Patch>(p.getId(), p) );
			}

			loadPatch = false;

			break;
		}
	}

	file.close();
}