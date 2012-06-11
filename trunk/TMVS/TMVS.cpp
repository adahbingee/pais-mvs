#include "stdafx.h"

#include <time.h>

#include <opencv2\opencv.hpp>

#include "mvs\mvs.h"

using namespace cv;
using namespace PAIS;

int _tmain(int argc, _TCHAR* argv[])
{
	clock_t start_t, end_t;
	
	MVS &mvs = MVS::getInstance();
	mvs.loadNVM("../../TMVS_data/pawn/pawn.nvm");

	start_t = clock();
	mvs.refineSeedPatches();
	end_t = clock();

	printf("time1\t%f\n", (double)(end_t - start_t) / CLOCKS_PER_SEC);

	return 0;
}