#include "Polyhedron.h"

#include <fstream>
#include <sys/stat.h>
#include <boost/graph/adj_list_serialize.hpp>
#include <CGAL/extract_mean_curvature_flow_skeleton.h>
#include <CGAL/Polygon_mesh_processing/bbox.h>

typedef CGAL::Mean_curvature_flow_skeletonization<Mesh> Skeletonization;

void Polyhedron::skeletonize() {
    Skeletonization::Skeleton initial_skeleton;
    CGAL::Mean_curvature_flow_skeletonization<Mesh> mcs(*_poly);
    mcs.set_medially_centered_speed_tradeoff(1);
    mcs.set_quality_speed_tradeoff(1);
    mcs(initial_skeleton);
    //CGAL::extract_mean_curvature_flow_skeleton(*_poly, initial_skeleton);
    size_t n = boost::num_vertices(initial_skeleton);
    for (uint i = 0; i < n; i++) {
        auto point = initial_skeleton[i].point;
        boost::add_vertex(point, *_skeleton);
        _skeleton_mapping[point] = std::vector<Mesh::Vertex_index>();
        for (auto vertex : initial_skeleton[i].vertices) {
            _skeleton_mapping[point].push_back(vertex);
        }
    }
    boost::graph_traits<Skeleton>::edge_iterator edges_iter, edges_end;
    for (boost::tie(edges_iter, edges_end) = edges(initial_skeleton); edges_iter != edges_end; ++edges_iter) {
        auto src = boost::source(*edges_iter, *_skeleton);
        auto dst = boost::target(*edges_iter, *_skeleton);
        boost::add_edge(src, dst, *_skeleton);
    }
}

void Polyhedron::load_skeleton() {
    auto last_dot = _filename.rfind(".");
    std::string without_suffix = _filename.substr(0, last_dot);
    std::string skeleton_file = without_suffix + ".skel";

    struct stat buffer;
    _skeleton = make_shared<Skeleton>();
    if (stat(skeleton_file.c_str(), &buffer) == 0) {
        std::ifstream input_stream(skeleton_file);
        boost::archive::text_iarchive archive(input_stream);
        archive >> *_skeleton;
        cout << "Loaded skeleton from file" << endl;
    } else {
        skeletonize();
        std::ofstream output_stream(skeleton_file);
        boost::archive::text_oarchive archive(output_stream);
        archive << *_skeleton;
        cout << "wrote skeleton to file" << endl;
    }


}

shared_ptr<Skeleton> Polyhedron::get_skeleton() {
    return _skeleton;
}

void Polyhedron::trim_skeleton(shared_ptr<Skeleton> skeleton) {
    if (skeleton == nullptr)
        return;
    _skeleton = shared_ptr<Skeleton>(new Skeleton(*skeleton));
    boost::graph_traits<Skeleton>::vertex_iterator vertices_iter, vertices_end;
    auto checker = CGAL::Side_of_triangle_mesh<Mesh, K>(*_poly);
    for (boost::tie(vertices_iter, vertices_end) = vertices(*_skeleton);
         vertices_iter != vertices_end; ++vertices_iter) {
        auto vertex = *vertices_iter;
        if (not(checker((*_skeleton)[vertex].point) == CGAL::ON_BOUNDED_SIDE))
            boost::clear_vertex(vertex, *_skeleton);
    }
}