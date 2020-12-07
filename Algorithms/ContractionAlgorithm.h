#ifndef SLICE_SEPERATIONALGORITHM_H
#define SLICE_SEPERATIONALGORITHM_H

#include "BinarySearchHough.h"
#include "ExternalBaseAlgorithm.h"
#include "../3D/Polyhedron.h"

/* T-Junction splitting */
class ContractionAlgorithm : public ExternalBaseAlgorithm {
public:
    ContractionAlgorithm(SliceApp *app, Polyhedron *poly, Face reference_face);

    ContractionAlgorithm(SliceApp *app, Polyhedron *poly, shared_ptr<JunctionDescriptor> junction);

    virtual std::string name();

protected:
    shared_ptr<JunctionDescriptor> _junction;

    virtual void preprocess();

    virtual void optimize();

    virtual set<Polyhedron *> cut();
};

class ContractionFactory : public AlgorithmFactory {
public:
    ContractionFactory(SliceApp *app, Polyhedron *shape);

protected:
    virtual CutAlgorithm *construct(int idx);
};

#endif //SLICE_SEPERATIONALGORITHM_H
