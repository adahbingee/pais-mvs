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
	config.cellSize           = 2;
	config.patchRadius        = 15;
	config.distWeighting      = config.patchRadius / 3.0;
	config.diffWeighting      = 128*128;
	config.minCamNum          = 10;
	config.textureVariation   = 36;
	config.visibleCorrelation = 0.87;
	config.minCorrelation     = 0.95;
	config.minLOD             = 0;
	config.maxLOD             = 10;
	config.lodRatio           = 0.8;
	config.maxCellPatchNum    = 3;
	config.neighborRadius     = 0.005;
	config.minRegionRatio     = 0.55;
	config.depthRangeScalar   = 1.5;
	config.particleNum        = 15;
	config.maxIteration       = 60;

	// set MVS instance
	MVS &mvs = MVS::getInstance(config);

	// LOAD MVS file
	//mvs.loadNVM2("../../../TMVS_data/temple/temple.nvm2");
	//mvs.loadNVM((char*)argv[1]);
	mvs.loadMVS((char*)argv[1]);
	//mvs.loadMVS("init.mvs");
	// printf("patches: %d\n", mvs.getPatches().size());

	// start MVS process
	//clock_t start_t, end_t;
	//start_t = clock();
	//end_t = clock();
	//printf("time1\t%f\n", (double)(end_t - start_t) / CLOCKS_PER_SEC);
	
	viewer = new MvsViewer(mvs, true, true, true);
	
	/*
	mvs.writeMVS("init.mvs");
	mvs.refineSeedPatches();
	mvs.writeMVS("seed.mvs");
	mvs.expansionPatches();
	mvs.writeMVS("exp.mvs");
	mvs.cellFiltering();
	printf("patches: %d\n", mvs.getPatches().size());
	mvs.visibilityFiltering();
	printf("patches: %d\n", mvs.getPatches().size());
	mvs.neighborCellFiltering(0.1, 0.25);
	printf("patches: %d\n", mvs.getPatches().size());
	mvs.writeMVS("filter.mvs");
	mvs.writePLY("cloud.ply");
	mvs.writePSR("cloud.psr");
	*/
	
	//debugFile.close();
	return 0;
}