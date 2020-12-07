#ifndef SLICE_STOCHASTITIP_H
#define SLICE_STOCHASTITIP_H


#include "../3D/Polyhedron.h"
#include "BinarySearchHough.h"
#include "../SliceApp.h"

class StochasTip : public BinarySearchHough {
public:
    StochasTip(SliceApp *app, Polyhedron *poly, Face reference_face);

    StochasTip(SliceApp *app, Polyhedron *poly, Face reference_face, Vector3 cut_normal);

    virtual std::string name() override;

protected:
    virtual void preprocess() override;

    virtual void optimize() override;

    virtual bool optimize_iterate() override;

    K::FT printability_score(shared_ptr<CutOption> cut) override;

    shared_ptr<CutOption> choose_intersect(InfinityFace infinity_face) override;

    virtual void perturb_orientation() override;

    virtual set<Polyhedron *> cut() override;

    Plane3 find_plane(Polyhedron *poly, int num_faces);
};

class StochasTipFactory : public AlgorithmFactory {
    typedef SliceApp SSliceApp;
public:
    StochasTipFactory(SliceApp *app, Polyhedron *shape);

protected:
    virtual CutAlgorithm *construct(int idx);

    std::vector<int> _tips;
};


#endif //SLICE_STOCHASTITIP_H
