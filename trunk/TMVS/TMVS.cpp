#include "stdafx.h"

#include <time.h>

#include <opencv2\opencv.hpp>

#include "mvs\mvs.h"
#include "view\mvsviewer.h"

using namespace cv;
using namespace PAIS;

//ofstream debugFile("debug.txt", ofstream::out);

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
	config.cellSize           = 1;
	config.patchRadius        = 15;
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
	config.depthRangeScalar   = 2;
	config.particleNum        = 10;
	config.maxIteration       = 40;
	config.expansionStrategy  = MVS::EXPANSION_BEST_FIRST;

	// set MVS instance
	MVS &mvs = MVS::getInstance(config);

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
			printf("time1\t%f\n", (double)(end_t - start_t) / CLOCKS_PER_SEC);
		}
	} else {
		// Todo: useage message
	}

	/*
	mvs.cellFiltering();
	printf("patches: %d\n", mvs.getPatches().size());
	mvs.visibilityFiltering();
	printf("patches: %d\n", mvs.getPatches().size());
	mvs.neighborCellFiltering(0.25);
	printf("patches: %d\n", mvs.getPatches().size());
	mvs.writeMVS("filter.mvs");
	mvs.neighborPatchFiltering();
	mvs.writeMVS("filter2.mvs");
	mvs.writePLY("cloud.ply");
	mvs.writePSR("cloud.psr");
	mvs.patchQuantization(24, 24, 100);
	mvs.writePLY("cloud_quantized.ply");
	mvs.writePSR("cloud_quantized.psr");
	*/
	
	//debugFile.close();
	system("pause");
	return 0;
}