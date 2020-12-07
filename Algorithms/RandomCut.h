#ifndef SLICE_RANDOMCUT_H
#define SLICE_RANDOMCUT_H

#include "CutAlgorithm.h"

class RandomCut : public CutAlgorithm {
public:
    RandomCut(SliceApp *app, Polyhedron *shape, Vector3 cut_norm);

    virtual std::string name();

protected:
    virtual void optimize();

    virtual void preprocess();

    Vector3 _norm;
};

class RandomCutFactory : public AlgorithmFactory {
public:
    RandomCutFactory(SliceApp *app, Polyhedron *shape);
    //virtual CutAlgorithm<S>* pick();
protected:
    virtual CutAlgorithm *construct(int idx);

private:
    std::uniform_real_distribution<double> _distribution;
    std::minstd_rand _generator;
};


#endif //SLICE_RANDOMCUT_H
