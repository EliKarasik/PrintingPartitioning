#ifndef SLICE_EXTERNALBASEALGORITHM_H
#define SLICE_EXTERNALBASEALGORITHM_H


#include "BinarySearchHough.h"
#include "../3D/Polyhedron.h"

class ExternalBaseAlgorithm : public BinarySearchHough {
public:
    ExternalBaseAlgorithm(SliceApp *app, Polyhedron *shape, Face reference_face);

    ExternalBaseAlgorithm(SliceApp *app, Polyhedron *shape, Plane3 base, Face reference_face);

    virtual std::string name() override;

protected:
    virtual bool optimize_iterate() override;

    virtual void preprocess() override;

    virtual set<Polyhedron *> cut() override;

    virtual void perturb_orientation() override;

    shared_ptr<CutOption> choose_intersect(InfinityFace infinity_face) override;

    K::FT printability_score(shared_ptr<CutOption> cut) override;

    Plane3 _base;
    bool _fully_printable;
};

class ExternalBaseFactory : public AlgorithmFactory {
public:
    ExternalBaseFactory(SliceApp *app, Polyhedron *shape);

protected:
    virtual CutAlgorithm *construct(int idx);

private:
};


#endif //SLICE_EXTERNALBASEALGORITHM_H
