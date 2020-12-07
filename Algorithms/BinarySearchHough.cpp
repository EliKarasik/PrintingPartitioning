#include "BinarySearchHough.h"

BinarySearchHough::BinarySearchHough(SliceApp *app, Polyhedron *shape, Face reference_face)
        : HoughAlgorithm(app, shape, reference_face), _best_score(-1), _alpha(0.2), _total_iterations(5) {}

void BinarySearchHough::visualize() {
    this->_hough = new HoughSpace(this->_epsilon_reference_point);
    this->_hough->reset_middle(this->_epsilon_reference_point);
    this->_hough->set_selected(this->_hough->approximate(this->_best_cut));
    this->_hough->visualize();
    QObject::connect(this->_hough, &HoughSpace::selection_changed, this, &HoughAlgorithm::update_result);
}


void BinarySearchHough::optimize() {
    if (not this->_reference_face.is_valid())
        return;
    int num_iterations = 10;
    for (int i = 1; i <= num_iterations; i++) {
        while (not this->_do_step)
            usleep(10000);
        //this->_do_step = false;
        if (this->_canceled)
            return;
        if (not optimize_iterate()) {
            break;
        }
        perturb_orientation();
        emit this->progress_updated(i / (double) _total_iterations);
    }
    if (_best_score > 0) {
        auto cut_option = choose_intersect(this->_best_cut);
        this->_result = new CutDescription(cut_option);
    }
    //this->_hough.set_selected(_best_coords);
    //this->update_result();
    this->allow_manual_changes();
    if (_best_score != 1)
            emit this->progress_updated(1);
}

//
//bool BinarySearchHough::optimize_iterate() {
//    static const double minimal_distance = 2;
//    HoughCoords best_coords;
//    K::FT best_score = -1;
//    auto cand = this->binary_search(_orientation, -MAX_DIST, - minimal_distance, true);
//    if (best_score < cand.second) {
//        best_score = cand.second;
//        best_coords = cand.first;
//    }
//    auto cand2 = this->binary_search(_orientation, minimal_distance, MAX_DIST, false);
//    if (best_score < cand2.second) {
//        best_score = cand2.second;
//        best_coords = cand2.first;
//    }
//    //cout << best_score / _best_score << "%" << endl;
//    if (best_score == -1 or best_score < _best_score * 0.7) //precentage maybe?
//        return false;
//    if (best_score > _best_score) {
//        _best_score = best_score;
//        _best_coords = best_coords;
//        //cout << "Best score:" << _best_score << endl;
//    }
//    this->_hough.set_selected(best_coords);
//    this->update_result();
//    emit this->request_redraw();
//    return true;
//}


void BinarySearchHough::perturb_orientation() {
}

//
//pair<HoughCoords, K::FT> BinarySearchHough::binary_search(Orientation orientation, double left, double right, bool to_left) {
//    auto test_plane = this->_hough.inverse_hough(HoughCoords(orientation, to_left ? right : left));
//    auto projected_point = test_plane.projection(this->_reference_point);
//    auto face = this->_shape->infinity_face(this->_reference_face);
//    if (face.oriented_side(projected_point) != CGAL::NEGATIVE) //not a relevant search
//        return pair<HoughCoords, K::FT>(HoughCoords(orientation, 0), -1);
//    auto test_cut = choose_intersect(test_plane);
//    if (test_cut == nullptr or this->printability_score(test_cut) == -1) // closest is bad, don't search
//        return pair<HoughCoords, K::FT>(HoughCoords(orientation, 0), -1);
//
//    double search_resolution = 0.25; //0.25;
//    while (right - left > search_resolution) {
//        double mid = ((right + left) / 2);
//        InfinityFace infinity_cut = this->_hough.inverse_hough(HoughCoords(orientation, mid));
//        auto cut = choose_intersect(infinity_cut);
//        if (cut == nullptr) {
//            if (to_left) left = mid + search_resolution;
//            else right = mid - search_resolution;
//            continue;
//        }
//        K::FT printability = this->printability_score(cut);
//        this->_hough.set_value(HoughCoords(orientation, mid), printability);
//        if (printability == -1) {
//            if (to_left) left = mid + search_resolution;
//            else right = mid - search_resolution;
//        } else {
//            if (to_left) right = mid;
//            else left = mid;
//        }
//    }
//
//    auto left_coords = HoughCoords(orientation, left), right_coords = HoughCoords(orientation, right);
//    K::FT left_score = this->printability_score(choose_intersect(this->_hough.inverse_hough(left_coords)));
//    K::FT right_score = this->printability_score(choose_intersect(this->_hough.inverse_hough(right_coords)));
//    if (left_score < right_score) {
//        return pair<HoughCoords, K::FT>(right_coords, right_score);
//    } else {
//        return pair<HoughCoords, K::FT>(left_coords, left_score);
//    }
//}