#include "filewriter.h"


// compute distance from point to point
double DistPoint2Point(obj_vector *, obj_vector *);

bool bIntersectTriangle(const obj_vector& orig, const obj_vector& dir, obj_vector& v0, obj_vector& v1, obj_vector& v2)
{
	obj_vector E1, E2, P, T;

	E1.e[0]=v1.e[0]-v0.e[0];	E1.e[1]=v1.e[1]-v0.e[1];	E1.e[2]=v1.e[2]-v0.e[2];
	E2.e[0]=v2.e[0]-v0.e[0];	E2.e[1]=v2.e[1]-v0.e[1];	E2.e[2]=v2.e[2]-v0.e[2];
	
	// cross
	P.e[0] = (dir.e[1] * E2.e[2]) - (dir.e[2] * E2.e[1]);
	P.e[1] = (dir.e[2] * E2.e[0]) - (dir.e[0] * E2.e[2]);
	P.e[2] = (dir.e[0] * E2.e[1]) - (dir.e[1] * E2.e[0]);

	// determinant
	float det = E1.e[0]*P.e[0] + E1.e[1]*P.e[1] + E1.e[2]*P.e[2];

	// keep det > 0, modify T accordingly
	if( det >0 )
	{
		T.e[0] = orig.e[0] - v0.e[0];
		T.e[1] = orig.e[1] - v0.e[1];
		T.e[2] = orig.e[2] - v0.e[2];
	}
	else
	{
		T.e[0] = v0.e[0] - orig.e[0];
		T.e[1] = v0.e[1] - orig.e[1];
		T.e[2] = v0.e[2] - orig.e[2];
		det =-det;
	}
	
	// If determinant is near zero, ray lies in plane of triangle
	if( det <0.0001f )
		return false;

	// Calculate u and make sure u <= 1
	double u, v;
	u = T.e[0]*P.e[0] + T.e[1]*P.e[1] + T.e[2]*P.e[2];
	if( u <0.0f||u > det )
		return false;

	// Q
	obj_vector Q;
	// cross
	Q.e[0] = (T.e[1] * E1.e[2]) - (T.e[2] * E1.e[1]);
	Q.e[1] = (T.e[2] * E1.e[0]) - (T.e[0] * E1.e[2]);
	Q.e[2] = (T.e[0] * E1.e[1]) - (T.e[1] * E1.e[0]);
	
	// Calculate v and make sure u + v <= 1
	v = Q.e[0]*dir.e[0] + Q.e[1]*dir.e[1] + Q.e[2]*dir.e[2];
	if( v <0.0f||u +v > det )
		return false;

	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Jet color map, low:blue, high:red
void JetColorMap(unsigned char *rgb,float value,float min,float max)
{
	  unsigned char c1=144;
	  float max4=(max-min)/4;
	  value-=min;
	  if(value==HUGE_VAL)
		{rgb[0]=rgb[1]=rgb[2]=255;}
	  else if(value<0)
		{rgb[0]=0;rgb[1]=0;rgb[2]=255;}  //{rgb[0]=rgb[1]=rgb[2]=0;}
	  else if(value<max4)
		{rgb[0]=0;rgb[1]=0;rgb[2]=c1+(unsigned char)((255-c1)*value/max4);}
	  else if(value<2*max4)
		{rgb[0]=0;rgb[1]=(unsigned char)(255*(value-max4)/max4);rgb[2]=255;}
	  else if(value<3*max4)
		{rgb[0]=(unsigned char)(255*(value-2*max4)/max4);rgb[1]=255;rgb[2]=255-rgb[0];}
	  else if(value<max)
		{rgb[0]=255;rgb[1]=(unsigned char)(255-255*(value-3*max4)/max4);rgb[2]=0;}
	  else {rgb[0]=255;rgb[1]=rgb[2]=0;}
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// compute distance from a vertex p to a plane that constructed by three points (p1, p2, p3)
double DistPoint2Point(obj_vector *p1, obj_vector *p2)
{
	double a, b, c;
	a = p1->e[0] - p2->e[0];
	b = p1->e[1] - p2->e[1];
	c = p1->e[2] - p2->e[2];

	return sqrt(a*a+b*b+c*c);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// compute distance from a vertex p to a plane that constructed by three points (p1, p2, p3)
float DistToPlane(obj_vector *p, obj_vector *p1, obj_vector *p2, obj_vector *p3)
{
	obj_vector v1, v2, normal, normal_inv;
	bool bInt;

	v1.e[0] = p1->e[0] - p2->e[0];
	v1.e[1] = p1->e[1] - p2->e[1];
	v1.e[2] = p1->e[2] - p2->e[2];
	v2.e[0] = p1->e[0] - p3->e[0];
	v2.e[1] = p1->e[1] - p3->e[1];
	v2.e[2] = p1->e[2] - p3->e[2];

	// cross
	normal.e[0] = (v1.e[1] * v2.e[2]) - (v1.e[2] * v2.e[1]);
	normal_inv.e[0] = normal.e[0] * -1;
	normal.e[1] = (v1.e[2] * v2.e[0]) - (v1.e[0] * v2.e[2]);
	normal_inv.e[1] = normal.e[1] * -1;
	normal.e[2] = (v1.e[0] * v2.e[1]) - (v1.e[1] * v2.e[0]);
	normal_inv.e[2] = normal.e[2] * -1;
	
	// -D
	float D = float(-(p1->e[0]*normal.e[0] + p1->e[1]*normal.e[1] + p1->e[2]*normal.e[2]));

	// intersection test
	bInt = bIntersectTriangle(*p, normal_inv, *p1, *p2, *p3);

	if (bInt)
	{
		// Dist
		float fOutDist = float( abs(p->e[0]*normal.e[0] + p->e[1]*normal.e[1] + p->e[2]*normal.e[2] + D));

		fOutDist /= float(sqrt(normal.e[0]*normal.e[0]+normal.e[1]*normal.e[1]+normal.e[2]*normal.e[2]));
	
		return fOutDist;
	}
	else
	{
		// output the distance to the nearest vertex of the face
		float fTmpDist = DistPoint2Point(p, p1);
		if (fTmpDist > DistPoint2Point(p,p2))
			fTmpDist = DistPoint2Point(p, p2);
		if (fTmpDist > DistPoint2Point(p,p3))
			fTmpDist = DistPoint2Point(p, p3);

		return fTmpDist;
	}
}



void FileWriter::writeMvsConfig(fstream &file, const MVS &mvs) {
	const MvsConfig *config = static_cast<const MvsConfig*> (&mvs);
	file.write((char*) config, sizeof(MvsConfig));
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

// compare with ground truth model, output in color map
// added by Chaody, 2012.Sep.04
void FileWriter::writeColorDistPatch(fstream &file, objLoader *objGT, const Patch &patch, float fColorDistMin, float fColorDistMax) {
	const vector<int> &camIdx = patch.getCameraIndices();
	const int camNum = (int) camIdx.size();
	double fitness = patch.getFitness();
	double correaltion = patch.getCorrelation();

	////////////////////////////////////////////////////////
	// compute distance
	////////////////////////////////////////////////////////
	obj_vector objPatchCenter;
	Vec3d vPatchCenter = patch.getCenter();
	objPatchCenter.e[0] = vPatchCenter[0];
	objPatchCenter.e[1] = vPatchCenter[1];
	objPatchCenter.e[2] = vPatchCenter[2];
	
	//objLoader *objGT = new objLoader(); // ground truth model
	//objGT->load(fileNameGT);

	//printf("[Ground truth 3D model] %s\n", fileNameGT);
	//printf("Number of vertices: %i\n", objGT->vertexCount);

	float fMinDist = FLT_MAX ;
	int j;
	unsigned char *rgb;
	rgb = new unsigned char[3];

	// for all faces in GT model
	#pragma omp parallel for
	for (j=0; j<objGT->faceCount; j++)
	{
		// compute distance
		obj_face *o = objGT->faceList[j];
		
		float fDist = DistToPlane(&objPatchCenter, objGT->vertexList[ o->vertex_index[0]], objGT->vertexList[ o->vertex_index[1]], objGT->vertexList[ o->vertex_index[2]]);
			
		// store min. dist
		if (fDist < fMinDist)
			fMinDist = fDist;
	}
		
	// output vertex in color map
	JetColorMap(rgb, fMinDist, fColorDistMin, fColorDistMax);

	////////////////////////////////////////////////////////

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
	// write color
	file.write((char*) &rgb[0], sizeof(unsigned char));
	file.write((char*) &rgb[1], sizeof(unsigned char));
	file.write((char*) &rgb[2], sizeof(unsigned char));

	delete [] rgb;

}

// compare with ground truth model, output in color map
// added by Chaody, 2012.Sep.04
void FileWriter::writeColorDistMVS(const char *fileName, char *fileNameGT, float fColorDistMin, float fColorDistMax, const MVS &mvs) {
	fstream file;
	file.open(fileName, fstream::out | fstream::binary);
	if ( !file.is_open() ) {
		printf("Can't write file %s\n", fileName);
	}

	// write MVSC header
	file << "MVSC_V1" << endl;

	// write MVS config
	writeMvsConfig(file, mvs);

	// write cameras
	const vector<Camera> &cameras = mvs.getCameras();
	const int camNum = (int) cameras.size();
	file << "CAMERAS " << camNum << endl;
	for (int i = 0; i < camNum; ++i) {
		writeCamera(file, cameras[i]);
	}

	// write patches
	const map<int, Patch> &patches = mvs.getPatches();
	const int patchNum = (int) patches.size();
	file << "PATCHES " << patchNum << endl;
	map<int, Patch>::const_iterator it;
	
	printf("\n"); int i = 0;

	objLoader *objGT = new objLoader(); // ground truth model
	objGT->load(fileNameGT);

	for (it = patches.begin(); it != patches.end(); ++it) {
		writeColorDistPatch(file, objGT, (*it).second, fColorDistMin, fColorDistMax);
		printf("\rprocessing ...... %d / %d", i++, patchNum);
	}
	printf("End of writing.\n");

	delete objGT;

	file.close();
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
	const vector<Camera> &cameras = mvs.getCameras();
	const int camNum = (int) cameras.size();
	file << "CAMERAS " << camNum << endl;
	for (int i = 0; i < camNum; ++i) {
		writeCamera(file, cameras[i]);
	}

	// write patches
	const map<int, Patch> &patches = mvs.getPatches();
	const int patchNum = (int) patches.size();
	file << "PATCHES " << patchNum << endl;
	map<int, Patch>::const_iterator it;
	for (it = patches.begin(); it != patches.end(); ++it) {
		writePatch(file, (*it).second);
	}

	file.close();
}

// write MVS in ascii mode, added by Chaody, 2012.Sep.13
void FileWriter::writeMVSascii(const char *fileName, const MVS &mvs) {
	const map<int, Patch> &patches = mvs.getPatches();
	map<int, Patch>::const_iterator it;
	ofstream file;
	file.open(fileName, ofstream::out);
	if ( !file.is_open() ) {
		printf("Can't open file %s\n", fileName);
		return;
	}

	int i=0;

	for (it = patches.begin(); it != patches.end(); ++it) {
		const Patch &pth = it->second;
		const Vec3d &p   = pth.getCenter();
		const vector<Vec2d> &imgPoints = pth.getImagePoints();
		const double &fitness = pth.getFitness();

		const int camNum               = pth.getCameraNumber();


		file << "ID:" << i++ << endl;
		file << "(x, y, z): " << p[0] << " " << p[1] << " " << p[2] << endl;
		for (int j=0; j<camNum; j++){
			file << "(u, v) on Cam" << j << ": " << imgPoints[j][0] << " " << imgPoints[j][0] << endl;
		}
		file << "fitness: " << fitness << endl;
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

void FileWriter::writeDeletedPatchMVS(const char *fileName, const MVS &mvs) {
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
	const vector<Patch> &patches = mvs.getDeletedPatches();
	const int patchNum = (int) patches.size();
	file << "PATCHES " << patchNum << endl;
	vector<Patch>::const_iterator it;
	for (it = patches.begin(); it != patches.end(); ++it) {
		writePatch(file, *it);
	}

	file.close();
}

void FileWriter::writeDeletedPatchPLY(const char *fileName, const MVS &mvs) {
	const vector<Patch> &patches = mvs.getDeletedPatches();
	vector<Patch>::const_iterator it;
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
		const Patch &pth = *it;
		const Vec3d &p   = pth.getCenter();
		const Vec3d &n   = pth.getNormal();
		const Vec3b &c   = pth.getColor();
		file << p[0] << " " << p[1] << " " << p[2] << " ";
		file << n[0] << " " << n[1] << " " << n[2] << " ";
		file << int(c[2]) << " " << int(c[1]) << " " << int(c[0]) << endl;
	}

	file.close();
}