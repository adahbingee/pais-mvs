#include "stdafx.h"

#include <time.h>

#include <opencv2\opencv.hpp>

#include "mvs\mvs.h"
#include "view\mvsviewer.h"
#include "mvs\featuremanager.h"

using namespace cv;
using namespace PAIS;

ofstream debugFile("debug.txt", ofstream::out);

MvsViewer *viewer;

void addPatchView(const Patch &pth) {
	if (viewer != NULL) {
		viewer->addPatch(pth);
	}
}

int main(int argc, char* argv[])
{
	// MVS configures
	MvsConfig config;
	config.cellSize           = 4;
	config.patchRadius        = 15;
	config.reduceNormalRange  = 2;
	config.adaptiveEnable     = true;
	config.distWeighting      = config.patchRadius / 3.0;
	config.diffWeighting      = 128*128;
	config.minCamNum          = 3;
	config.textureVariation   = 36;
	config.visibleCorrelation = 0.7;
	config.minCorrelation     = 0.7;
	config.maxFitness         = 10.0;
	config.minLOD             = 0;
	config.maxLOD             = 15;
	config.lodRatio           = 0.8;
	config.maxCellPatchNum    = 3;
	config.neighborRadius     = 0.005;
	config.minRegionRatio     = 0.55;
	config.depthRangeScalar   = 1;
	config.particleNum        = 5;
	config.maxIteration       = 10;
	config.expansionStrategy  = MVS::EXPANSION_BEST_FIRST;

	FileLoader::loadConfig("config.txt", config);

	// set MVS instance
	MVS &mvs = MVS::getInstance(config);

	//mvs.loadNVM2("D:/workspace/TMVS_data/boxball/boxball.nvm2");
	//FeatureManager::getFeatureDescriptor(mvs.getCameras(), 1.0);

	if (argc >= 3) {
		if ( strcmp(argv[1], "-v") == 0 ) {         // viewer
			mvs.loadMVS((char*)argv[2]);
			viewer = new MvsViewer(mvs, true, true, false);
			viewer->open();

		} else if ( strcmp(argv[1], "-a") == 0 ) {  // animate
			mvs.loadMVS((char*)argv[2]);
			viewer = new MvsViewer(mvs, true, true, true);
			viewer->open();

		} else if ( strcmp(argv[1], "-r") == 0 ) {  // reconstruction
			string fileName(argv[2]);
			size_t found = fileName.find_last_of(".");
			string fileExt = fileName.substr(found+1);

			// load file
			if ( fileExt.compare("nvm") == 0 ) {
				mvs.loadNVM(argv[2]);
			} else if ( fileExt.compare("nvm2") == 0 ) {
				mvs.loadNVM2(argv[2]);
			} else if ( fileExt.compare("mvs") == 0 ) {
				mvs.loadMVS(argv[2]);
			}

			printf("patches: %d\n", mvs.getPatches().size());

			// run reconstruction
			clock_t start_t, end_t;
			start_t = clock();
			mvs.writeMVS("init.mvs");
			mvs.refineSeedPatches();
			mvs.writeMVS("seed.mvs");
			mvs.expansionPatches();
			mvs.writeMVS("exp.mvs");
			mvs.writePLY("exp.ply");
			mvs.writePSR("exp.psr");
			end_t = clock();

			double totime = (double)(end_t - start_t) / CLOCKS_PER_SEC;
			printf("time1\t%f\n", totime);
			debugFile << "total time: " << totime << endl;
			debugFile.close();
			system("pause");
		} else if ( strcmp(argv[1], "-f") == 0 ) { // filtering
			string fileName(argv[2]);
			size_t found = fileName.find_last_of(".");
			string fileExt = fileName.substr(found+1);

			// load file
			if ( fileExt.compare("mvs") == 0 ) {
				mvs.loadMVS(argv[2]);
			} else {
				printf("filtering only mvs file\n");
				return 1;
			}

			// load config
			FileLoader::loadConfig("config.txt", config);
			mvs.setConfig(config);

			printf("patches: %d\n", mvs.getPatches().size());

			clock_t start_t, end_t;
			start_t = clock();
			mvs.cellFiltering();
			mvs.visibilityFiltering();
			mvs.neighborCellFiltering(0.25);
			mvs.neighborPatchFiltering(0.25);
			end_t = clock();
			mvs.writeMVS("filter.mvs");
			mvs.writePLY("filter.ply");
			
			double totime = (double)(end_t - start_t) / CLOCKS_PER_SEC;
			printf("time1\t%f\n", totime);
			debugFile << "total time: " << totime << endl;
			debugFile.close();
			system("pause");
		}
	} else {
		// Todo: useage message
		char *msg = "-v [filename.mvs]: viewer\n-a [filename.mvs]: animate\n-r {[filename.mvs], [filename.nvm], [filename.nvm2]}: reconstruction\n";
		printf(msg);
		return 1;
	}
	return 0;
}