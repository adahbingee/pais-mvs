#include "stdafx.h"

#include <time.h>

#include <opencv2\opencv.hpp>

#include "mvs\mvs.h"
//#include "view\mvsviewer.h"

using namespace cv;
using namespace PAIS;

int main(int argc, char* argv[])
{
	clock_t start_t, end_t;

	MvsConfig config;
	config.cellSize           = 8;
	config.patchRadius        = 25;
	config.distWeighting      = config.patchRadius / 3.0;
	config.diffWeighting      = 128*128;
	config.minCamNum          = 2;
	config.textureVariation   = 36;
	config.visibleCorrelation = 0.87;
	config.minCorrelation     = 0.99;
	config.minLOD             = 0;
	config.maxCellPatchNum    = 5;
	config.particleNum        = 10;
	config.maxIteration       = 50;

	
	MVS &mvs = MVS::getInstance(config);
	mvs.loadNVM("../../../TMVS_data/wc/wc.nvm");
	//mvs.loadNVM((char*)argv[1]);
	//mvs.loadMVS((char*)argv[1]);
	//mvs.loadMVS("pmvs.mvs");

	start_t = clock();
	mvs.refineSeedPatches();
	mvs.writeMVS("seed.mvs");
	mvs.expansionPatches();
	end_t = clock();

	//mvs.patchQuantization(20, 90, 200);

	printf("time1\t%f\n", (double)(end_t - start_t) / CLOCKS_PER_SEC);
	mvs.writeMVS("exp.mvs");

	//MvsViewer viewer(mvs, true);

	return 0;
}