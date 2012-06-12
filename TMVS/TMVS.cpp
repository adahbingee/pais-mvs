#include "stdafx.h"

#include <time.h>

#include <opencv2\opencv.hpp>

#include "mvs\mvs.h"
//#include "view\mvsviewer.h"

using namespace cv;
using namespace PAIS;

int _tmain(int argc, _TCHAR* argv[])
{
	clock_t start_t, end_t;
	
	MVS &mvs = MVS::getInstance(5, 15, 3, 36, 0.8, 30, 1000);
	//mvs.loadNVM("../../../TMVS_data/pawn/pawn.nvm");
	mvs.loadMVS("pawn.mvs");

	start_t = clock();
	// mvs.refineSeedPatches();
	mvs.expansionPatches();
	end_t = clock();

	printf("time1\t%f\n", (double)(end_t - start_t) / CLOCKS_PER_SEC);
	mvs.wirteMVS("pawn_exp.mvs");

	//MvsViewer viewer(mvs, true);

	return 0;
}