#include "particle.h"

using namespace PAIS;

Particle::Particle(int dim) {
    this->dim  = dim;
    pBest      = new double [dim];
	nBest      = new double [dim];
    pos        = new double [dim];
    vec        = new double [dim];
    fitness      = 1.7976931348623158e+308;
	pBestFitness = 1.7976931348623158e+308;
	//fitness      = 99999;
    //pBestFitness = 99999;

    for (int i = 0; i < dim; i++) {
        pBest[i] = 0;
		nBest[i] = 0;
        pos[i]   = 0;
        vec[i]   = 0;
    }
}

Particle::Particle(const Particle &p) {
    dim        = p.dim;
    pBest      = new double [p.dim];
	nBest      = new double [p.dim];
    pos        = new double [p.dim];
    vec        = new double [p.dim];
    fitness      = p.fitness;
	pBestFitness = p.pBestFitness;

    for (int i = 0; i < dim; i++) {
        pBest[i] = p.pBest[i];
		nBest[i] = p.nBest[i];
        pos[i]   = p.pos[i];
        vec[i]   = p.vec[i];
    }
}

Particle& Particle::operator=(const Particle &p){

    if (dim != p.dim) {
        dim = p.dim;
        if (pBest) delete [] pBest;
		if (nBest) delete [] nBest;
        if (pos)   delete [] pos;
        if (vec)   delete [] vec;
        pBest = new double [dim];
		nBest = new double [dim];
        pos   = new double [dim];
        vec   = new double [dim];
    }
            
    for (int i = 0; i < dim; i++) {
        pBest[i] = p.pBest[i];
		nBest[i] = p.nBest[i];
        pos[i]   = p.pos[i];
        vec[i]   = p.vec[i];
    }

	fitness      = p.fitness;
	pBestFitness = p.pBestFitness;

    return *this;
}

Particle::~Particle(void) {
    if (pBest) {
        delete [] pBest;
        pBest = 0;
    }
	if (nBest) {
		delete [] nBest;
		nBest = 0;
	}
    if (pos) {
        delete [] pos;
        pos = 0;
    }
    if (vec) {
        delete [] vec;
        vec = 0;
    }
}