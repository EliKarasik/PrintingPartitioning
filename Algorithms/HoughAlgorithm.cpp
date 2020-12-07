#include "HoughAlgorithm.h"

#include "../3D/Polyhedron.h"
#include "../HoughSpace.h"

HoughAlgorithm::HoughAlgorithm(SliceApp *app, Polyhedron *shape, Face reference_face) :
        CutAlgorithm(app, shape, reference_face), _hough(nullptr) {

}

void HoughAlgorithm::close_visualize() {
    if (_hough != nullptr)
        _hough->close_visualize();
}

void HoughAlgorithm::update_result() {
    auto selected = _hough->get_selected();
    if (selected.distance == 0)
        return;
    auto plane = _hough->inverse_hough(selected);
    auto choice = choose_intersect(plane);
    if (this->_result) {
        delete this->_result;
        this->_result = nullptr;
    }
    if (choice != nullptr) {
        this->_result = new CutDescription(choice);
        try {
            this->_score = this->printability_score(choice);
            this->_hough->set_value(selected, this->_score);
        } catch (std::runtime_error &error) {
            cout << "Error calculating score:" << error.what() << endl;
        }
    }
    emit this->request_redraw();
}

void HoughAlgorithm::allow_manual_changes() {

}