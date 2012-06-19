#include "stdafx.h"

#include <time.h>

#include <opencv2\opencv.hpp>

#include "mvs\mvs.h"
#include "view\mvsviewer.h"

using namespace cv;
using namespace PAIS;

int main(int argc, char* argv[])
{
	clock_t start_t, end_t;

	MvsConfig config;
	config.cellSize           = 8;
	config.patchRadius        = 30;
	config.distWeighting      = config.patchRadius / 3.0;
	config.diffWeighting      = 128*128;
	config.minCamNum          = 3;
	config.textureVariation   = 1000;
	config.visibleCorrelation = 0.87;
	config.minCorrelation     = 0.99;
	config.minLOD             = 1;
	config.maxCellPatchNum    = 5;
	config.particleNum        = 15;
	config.maxIteration       = 60;

	
	MVS &mvs = MVS::getInstance(config);
	//mvs.loadNVM("../../../TMVS_data/mailbox/mailbox.nvm");
	//mvs.loadNVM((char*)argv[1]);
	mvs.loadMVS((char*)argv[1]);
	//mvs.loadMVS("exp.mvs");

	
	printf("patches: %d\n", mvs.getPatches().size());
	
	start_t = clock();
	//mvs.refineSeedPatches();
	//mvs.writeMVS("seed.mvs");
	//mvs.expansionPatches();
	//mvs.writeMVS("exp.mvs");
	//mvs.patchQuantization(20, 90, 200);
	//mvs.writeMVS("quan.mvs");
	mvs.setCellMaps();
	mvs.visibilityFiltering();
	printf("patches: %d\n", mvs.getPatches().size());
	end_t = clock();

	printf("time1\t%f\n", (double)(end_t - start_t) / CLOCKS_PER_SEC);
	

	MvsViewer viewer(mvs, true);

	return 0;
}