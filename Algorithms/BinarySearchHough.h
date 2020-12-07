#ifndef SLICE_BINARYSEARCHHOUGH_H
#define SLICE_BINARYSEARCHHOUGH_H

#include "HoughAlgorithm.h"
#include "../3D/Polyhedron.h"

class BinarySearchHough : public HoughAlgorithm {

protected:
    K::FT _best_score;

    Vector3 _cut_normal;
    Plane3 _best_cut;
    GrowingCutState _last_cut_state;
    double _alpha;
    int _total_iterations;

public:
    BinarySearchHough(SliceApp *app, Polyhedron *shape, Face reference_face);

    virtual void visualize();

    virtual void optimize();

    virtual bool optimize_iterate() = 0;

    virtual void perturb_orientation();

    //pair<HoughCoords, K::FT> binary_search(Orientation orientation, double left, double right, bool to_left);
    virtual shared_ptr<CutOption> choose_intersect(InfinityFace infinity_base) = 0;
};


#endif //SLICE_BINARYSEARCHHOUGH_H
