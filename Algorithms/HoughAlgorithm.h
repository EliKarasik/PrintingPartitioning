#ifndef SLICE_HOUGHALGORITHM_H
#define SLICE_HOUGHALGORITHM_H


#include "CutAlgorithm.h"
#include "../HoughSpace.h"

class HoughAlgorithm : public CutAlgorithm {
public:
    HoughAlgorithm(SliceApp *app, Polyhedron *shape, Face reference_face);

    virtual void close_visualize();

    void update_result();

protected:
    HoughSpace *_hough;

    virtual K::FT printability_score(shared_ptr<CutOption> base) = 0;

    virtual shared_ptr<CutOption> choose_intersect(InfinityFace infinity_base) = 0;

    void allow_manual_changes();
};


#endif //SLICE_HOUGHALGORITHM_H
