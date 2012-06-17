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
	config.cellSize           = 15;
	config.patchRadius        = 15;
	config.minCamNum          = 3;
	config.textureVariation   = 36;
	config.visibleCorrelation = 0.87;
	config.minCorrelation     = 0.95;
	config.minLOD             = 2;
	config.maxCellPatchNum    = 5;
	config.particleNum        = 15;
	config.maxIteration       = 60;

	
	MVS &mvs = MVS::getInstance(config);
	mvs.loadNVM("../../../TMVS_data/castle/castle.nvm");
	//mvs.loadNVM((char*)argv[1]);
	//mvs.loadMVS((char*)argv[1]);
	//mvs.loadMVS("pmvs.mvs");

	start_t = clock();
	mvs.refineSeedPatches();
	mvs.writeMVS("seed.mvs");
	//mvs.expansionPatches();
	end_t = clock();

	printf("time1\t%f\n", (double)(end_t - start_t) / CLOCKS_PER_SEC);
	//mvs.writeMVS("exp.mvs");

	MvsViewer viewer(mvs, true);

	return 0;
}