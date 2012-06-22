#include "stdafx.h"

#include <time.h>

#include <opencv2\opencv.hpp>

#include "mvs\mvs.h"
#include "view\mvsviewer.h"

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
	config.cellSize           = 10;
	config.patchRadius        = 30;
	config.distWeighting      = config.patchRadius / 3.0;
	config.diffWeighting      = 128*128;
	config.minCamNum          = 2;
	config.textureVariation   = 300;
	config.visibleCorrelation = 0.87;
	config.minCorrelation     = 0.9;
	config.minLOD             = 0;
	config.maxCellPatchNum    = 5;
	config.particleNum        = 15;
	config.maxIteration       = 60;

	// set MVS instance
	MVS &mvs = MVS::getInstance(config);

	// LOAD MVS file
	mvs.loadNVM("../../../TMVS_data/wc/wc.nvm");
	//mvs.loadNVM((char*)argv[1]);
	//mvs.loadMVS((char*)argv[1]);
	//mvs.loadMVS("exp.mvs");
	// printf("patches: %d\n", mvs.getPatches().size());

	// start MVS process
	//clock_t start_t, end_t;
	//start_t = clock();
	//end_t = clock();
	//printf("time1\t%f\n", (double)(end_t - start_t) / CLOCKS_PER_SEC);

	//viewer = new MvsViewer(mvs, true, true);

	/*
	mvs.refineSeedPatches();
	mvs.writeMVS("seed.mvs");
	mvs.setCellMaps();
	mvs.expansionPatches();
	mvs.writeMVS("exp.mvs");
	mvs.cellFiltering();
	printf("patches: %d\n", mvs.getPatches().size());
	mvs.visibilityFiltering();
	printf("patches: %d\n", mvs.getPatches().size());
	mvs.neighborCellFiltering(0.1, 0.25);
	printf("patches: %d\n", mvs.getPatches().size());
	mvs.writeMVS("filter.mvs");
	*/
	debugFile.close();
	return 0;
}