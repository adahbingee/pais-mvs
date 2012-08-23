#include "stdafx.h"

#include <time.h>

#include <opencv2\opencv.hpp>

#include "mvs\mvs.h"
#include "view\mvsviewer.h"
#include "mvs\featuremanager.h"

#define CONFIG_FILE_NAME "config.txt"

using namespace cv;
using namespace PAIS;

ofstream debugFile("debug.txt", ofstream::out);

MvsViewer *viewer;
MvsConfig config;

void addPatchView(const Patch &pth) {
	if (viewer != NULL) {
		viewer->addPatch(pth);
	}
}

void setInitConfig() {
	config.cellSize                 = 4;
	config.patchRadius              = 15;
	config.reduceNormalRange        = 2;
	config.adaptiveDistanceEnable   = true;
	config.adaptiveDifferenceEnable = true;
	config.adaptiveGradientEnable   = false;
	config.distWeighting            = config.patchRadius / 3.0;
	config.diffWeighting            = 128*128;
	config.gradientWeighting        = 10.0;
	config.minCamNum                = 3;
	config.textureVariation         = 36;
	config.visibleCorrelation       = 0.7;
	config.minCorrelation           = 0.7;
	config.maxFitness               = 10.0;
	config.minLOD                   = 0;
	config.maxLOD                   = 15;
	config.lodRatio                 = 0.8;
	config.maxCellPatchNum          = 3;
	config.neighborRadius           = 0.005;
	config.neighborRadiusScalar     = 0.0025;
	config.minRegionRatio           = 0.55;
	config.depthRangeScalar         = 1;
	config.particleNum              = 5;
	config.maxIteration             = 10;
	config.expansionStrategy        = MVS::EXPANSION_BEST_FIRST;
}

void runViewer(MVS &mvs, const char *fileName) {
	mvs.loadMVS(fileName);
	viewer = new MvsViewer(mvs, true, true, false);
	viewer->open();
}

void runAnimate(MVS &mvs, const char *fileName) {
	mvs.loadMVS(fileName);
	viewer = new MvsViewer(mvs, true, true, true);
	viewer->open();
}

void runReconstruct(MVS &mvs, const char *fileName) {
	string fileNameStr(fileName);
	size_t found = fileNameStr.find_last_of(".");
	string fileExt = fileNameStr.substr(found+1);

	// load file
	if ( fileExt.compare("nvm") == 0 ) {
		mvs.loadNVM(fileName);
	} else if ( fileExt.compare("nvm2") == 0 ) {
		mvs.loadNVM2(fileName);
	} else if ( fileExt.compare("mvs") == 0 ) {
		mvs.loadMVS(fileName);
	}

	// load config
	FileLoader::loadConfig(CONFIG_FILE_NAME, config);
	mvs.setConfig(config);

	printf("patches: %d\n", mvs.getPatches().size());

	// get seed point if no initial matching
	if ( mvs.getPatches().empty() ) {
		FeatureManager::setSeedPatches(mvs.getCameras(), 3.0, &mvs);
		if ( mvs.getPatches().empty() ) {
			printf("try less minCamNum\n");
		}
	}

	// run reconstruction
	clock_t start_t, end_t;
	start_t = clock();
	mvs.writeMVS("init.mvs"); // from seed traingulation
	mvs.refineSeedPatches();
	mvs.writeMVS("seed.mvs"); // after optimization and runtime filtering
	mvs.expansionPatches();
	mvs.writeMVS("exp.mvs");
	mvs.writePLY("exp.ply");
	mvs.writePSR("exp.psr");
	end_t = clock();

	// show runtime
	double totime = (double)(end_t - start_t) / CLOCKS_PER_SEC;
	printf("time1\t%f\n", totime);
	debugFile << "total time: " << totime << endl;
	debugFile.close();
	system("pause");
}

void runFiltering(MVS &mvs, const char *fileName) {
	string fileNameStr(fileName);
	size_t found = fileNameStr.find_last_of(".");
	string fileExt = fileNameStr.substr(found+1);

	// load file
	if ( fileExt.compare("mvs") == 0 ) {
		mvs.loadMVS(fileName);
	} else {
		printf("filtering only mvs file\n");
		return;
	}

	// load config
	FileLoader::loadConfig(CONFIG_FILE_NAME, config);
	mvs.setConfig(config);

	printf("patches: %d\n", mvs.getPatches().size());

	clock_t start_t, end_t;
	start_t = clock();
	// PMVS filtering
	mvs.cellFiltering();
	mvs.visibilityFiltering();
	mvs.neighborCellFiltering(0.25);
	mvs.writeMVS("PMVS_filter.mvs");
	mvs.writePLY("PMVS_filter.ply");
	// PCMVS filtering
	mvs.neighborPatchFiltering(0.25);
	end_t = clock();
	mvs.writeMVS("PCMVS_filter.mvs");
	mvs.writePLY("PCMVS_filter.ply");
			
	double totime = (double)(end_t - start_t) / CLOCKS_PER_SEC;
	printf("time1\t%f\n", totime);
	debugFile << "total time: " << totime << endl;
	debugFile.close();
	system("pause");
}

int main(int argc, char* argv[])
{
	// MVS configures
	setInitConfig();
	FileLoader::loadConfig(CONFIG_FILE_NAME, config);

	// set MVS instance
	MVS &mvs = MVS::getInstance(config);

	if (argc >= 3) {
		if ( strcmp(argv[1], "-v") == 0 ) {         // viewer
			runViewer(mvs, argv[2]);
		} else if ( strcmp(argv[1], "-a") == 0 ) {  // animate
			runAnimate(mvs, argv[2]);
		} else if ( strcmp(argv[1], "-r") == 0 ) {  // reconstruction
			runReconstruct(mvs, argv[2]);
		} else if ( strcmp(argv[1], "-f") == 0 ) {  // filtering
			runFiltering(mvs, argv[2]);
		}
	} else {
		char *msg = "-v [filename.mvs]: viewer\n-a [filename.mvs]: animate\n-r {[filename.mvs], [filename.nvm], [filename.nvm2]}: reconstruction\n-f [filename.mvs]";
		printf(msg);
		return 1;
	}

	return 0;
}