#include <boost/graph/connected_components.hpp>
#include "Polyhedron.h"
#include <sys/stat.h>
#include "boost/tuple/tuple_io.hpp"
#include "../HoughSpace.h"

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, IntPoint> SphereGraph;

#define CUBE_IDX(x, y, z, radius) (((2*(radius)+1)*(2*(radius)+1)*(x+(radius))+(2*(radius)+1)*(y+(radius))+(z+(radius))))
#define CUBE_IDX_POINT(point, radius) (CUBE_IDX((point).x(), (point).y(), (point).z(), radius))

void Polyhedron::load_tips() {
    auto last_dot = _filename.rfind('.');
    std::string without_suffix = _filename.substr(0, last_dot);
    std::string tips_file = without_suffix + ".tips";

    struct stat buffer;
    bool reading = (stat(tips_file.c_str(), &buffer) == 0);
    //reading = false;

    if (reading) {
        cout << "Loading tips from file" << endl;
        ifstream tips_stream(tips_file);
        uint tip_count;
        tips_stream >> tip_count;
        double x, y, z;
        double nx, ny, nz;
        for (uint i = 0; i < tip_count; i++) {
            tips_stream >> x >> y >> z;
            tips_stream >> nx >> ny >> nz;
            auto tip = make_shared<TipDescriptor>();
            tip->point = Point(x, y, z);
            tip->direction = Vector3(nx, ny, nz);
            tip->original = true;
            _tips.push_back(tip);
        }
        tips_stream.close();
    } else {
        cout << "Generating and saving tips" << endl;
        recognize_tips();
        ofstream output_stream(tips_file);
        output_stream << _tips.size() << endl;
        for (auto tip : _tips) {
            output_stream << tip->point << endl;
            output_stream << tip->direction << endl;
        }
        output_stream.close();
    }


}

void Polyhedron::load_voxelization() {
    auto last_dot = _filename.rfind('.');
    std::string without_suffix = _filename.substr(0, last_dot);
    std::string voxels_file = without_suffix + ".vox";
    std::string tips_file = without_suffix + ".tips";

    struct stat buffer;
    bool reading = (stat(voxels_file.c_str(), &buffer) == 0);

    if (reading) {
        cout << "Loading voxels from file" << endl;
        ifstream input_stream(voxels_file);
        auto total_size = (uint) pow(_resolution, 3);
        _voxels.reserve(total_size);
        _voxels.assign(total_size, false);
        for (int x = 0; x < _resolution; x++) {
            for (int y = 0; y < _resolution; y++) {
                for (int z = 0; z < _resolution; z++) {
                    bool bit;
                    input_stream >> bit;
                    _voxels[voxel_idx(IntPoint(x, y, z))] = bit;
                }
            }
        }
        input_stream.close();
    } else {
        cout << "Generating and saving voxels" << endl;
        ofstream output_stream(voxels_file);
        auto total_size = (uint) pow(_resolution, 3);
        _voxels.reserve(total_size);
        _voxels.assign(total_size, false);
        voxelize();
        for (int x = 0; x < _resolution; x++) {
            for (int y = 0; y < _resolution; y++) {
                for (int z = 0; z < _resolution; z++) {
                    bool bit = _voxels[voxel_idx(IntPoint(x, y, z))];
                    output_stream << bit << " ";
                }
                output_stream << endl;
            }
        }
        output_stream.close();
    }
}

int Polyhedron::voxel_idx(IntPoint p) {
    for (int i = 0; i < p.dimension(); i++) {
        if (p[i] < 0 or p[i] >= _resolution)
            return -1;
    }
    return p.x() * _resolution * _resolution + p.y() * _resolution + p.z();
}

void Polyhedron::voxelize() {
    auto checker = CGAL::Side_of_triangle_mesh<Mesh, K>(*_poly);
    for (int x = 0; x < _resolution; x++) {
        for (int y = 0; y < _resolution; y++) {
            for (int z = 0; z < _resolution; z++) {
                auto point = Point3((x + 0.5), (y + 0.5), (z + 0.5));
                _voxels[voxel_idx(IntPoint(x, y, z))] = (checker(point) == CGAL::ON_BOUNDED_SIDE);
            }
        }
    }
}

void Polyhedron::calculate_distances() {
    cout << "calculating boundary distances..." << endl;
    auto total_size = (uint) pow(_resolution, 3);
    _distances.reserve(total_size);
    _distances.assign(total_size, -1);
    set<IntPoint> current;
    int dist = 0;
    for (int x = 0; x < _resolution; x++) {
        for (int y = 0; y < _resolution; y++) {
            for (int z = 0; z < _resolution; z++) {
                _distances[voxel_idx(IntPoint(x, y, z))] = -1;
                if (not _voxels[voxel_idx(IntPoint(x, y, z))])
                    current.insert(IntPoint(x, y, z));
            }
        }
    }
    set<IntPoint> next;
    while (not current.empty()) {
        for (IntPoint p : current) {
            _distances[voxel_idx(p)] = dist;
        }
        for (IntPoint p : current) {
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dz = -1; dz <= 1; dz++) {
                        IntVector dir = IntVector(dx, dy, dz);
                        auto offset = p + dir;
                        bool bad = false;
                        for (int coord = 0; coord < 3; coord++)
                            if (offset[coord] < 0 or offset[coord] >= _resolution)
                                bad = true;
                        if (bad)
                            continue;
                        if (_distances[voxel_idx(offset)] == -1)
                            next.insert(offset);
                    }
                }
            }
        }
        dist++;
        current = next;
        next.clear();
    }
}

IntPoint Polyhedron::closest_point(IntPoint point) {
    auto vidx = voxel_idx(point);
    if (vidx < 0 or _voxels[vidx])
        return point;
    set<IntPoint> visited;
    visited.insert(point);
    queue<IntPoint> bfs;
    bfs.push(point);
    while (not bfs.empty()) {
        IntPoint current = bfs.front();
        bfs.pop();
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                for (int dz = -1; dz <= 1; dz++) {
                    auto next = current + IntVector(dx, dy, dz);
                    auto idx = voxel_idx(next);
                    if (idx == -1 or visited.find(next) != visited.end())
                        continue;
                    if (_voxels[idx]) {
                        return next;
                        cout << "found closest" << endl;
                    }
                    visited.insert(next);
                    bfs.push(next);
                }
            }
        }
    }
    return point; //won't ever happen
}


shared_ptr<JunctionDescriptor> Polyhedron::junctions_in_cube(IntPoint center, int max_radius) {
    if (voxel_idx(center) == -1) {
        return nullptr;
    }
    vector<bool> object;
    object.insert(object.begin(), (uint) pow(2 * max_radius + 1, 3), false);
    object[CUBE_IDX(0, 0, 0, max_radius)] = true;
    queue<IntPoint> last_level;
    last_level.push(IntPoint(0, 0, 0));
    queue<IntPoint> next_level;

    int radius;
    int min_element_idx;
    Vector3 min_element_direction;
    std::vector<int> component;
    SphereGraph graph;

    bool last_layer_good = false;
    for (radius = 1; radius <= max_radius; radius++) {
        while (not last_level.empty()) {
            auto current = last_level.front();
            last_level.pop();
            IntPoint current_point = center + IntVector(IntPoint(CGAL::ORIGIN), current);
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dz = -1; dz <= 1; dz++) {
                        auto next_point = current_point + IntVector(dx, dy, dz);
                        if (voxel_idx(next_point) == -1)
                            continue;
                        auto relative = current + IntVector(dx, dy, dz);
                        if (not(abs(relative.x()) <= radius and abs(relative.y()) <= radius
                                and abs(relative.z()) <= radius))
                            continue;
                        if (_voxels[voxel_idx(next_point)] and not object[CUBE_IDX_POINT(relative, max_radius)]) {
                            next_level.push(relative);
                            object[CUBE_IDX_POINT(relative, max_radius)] = true;
                        }
                    }
                }
            }
        }
        //cout << "next level size:" << next_level.size() << endl;
        last_level = next_level;
        //build sphere
        graph.clear();
        map<IntPoint, SphereGraph::vertex_descriptor> sphere;
        while (not next_level.empty()) {
            IntPoint current = next_level.front();
            next_level.pop();
            sphere[current] = boost::add_vertex(current, graph);
        }
        //build edges
        for (auto point_and_idx : sphere) {
            auto point = point_and_idx.first;
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dz = -1; dz <= 1; dz++) {
                        if (dx == 0 and dy == 0 and dz == 0)
                            continue;
                        IntVector dir = IntVector(dx, dy, dz);
                        auto offset = point + dir;
                        if (sphere.count(offset) == 0)
                            continue;
                        boost::add_edge(sphere[point], sphere[offset], graph);
                    }
                }
            }
        }

        //calculate CCs
        component.assign(boost::num_vertices(graph), -1);
        auto num_comp = boost::connected_components(graph, &component[0]);
        if (num_comp == 0)
            continue;
        std::vector<int> sizes;
        sizes.assign(num_comp, 0);
        std::vector<Vector3> mids;
        mids.assign(num_comp, Vector3(0, 0, 0));
        boost::graph_traits<SphereGraph>::vertex_iterator vertices_iter, vertices_end;
        for (boost::tie(vertices_iter, vertices_end) = boost::vertices(graph);
             vertices_iter != vertices_end; ++vertices_iter) {
            sizes[component[*vertices_iter]] += 1;
            //cout << graph[*vertices_iter] << endl;
            IntPoint p = graph[*vertices_iter];
            mids[component[*vertices_iter]] += Vector3(p.x(), p.y(), p.z());
        }
        for (int i = 0; i < num_comp; i++) {
            mids[i] /= sizes[i];
        }
        auto min_element_iterator = std::min_element(sizes.begin(), sizes.end());
        int min_element = *min_element_iterator;
        min_element_idx = (int) (min_element_iterator - sizes.begin());
        min_element_direction = mids[min_element_idx];
        auto max_element = *std::max_element(sizes.begin(), sizes.end());
        bool condition = (max_element / min_element) > 3;
        if (condition and last_layer_good) {
            //double ratio = ((double)max_element) / min_element;
            //cout << "seperation at radius " << radius << endl;
            //cout << "ratio: " << ratio << endl;
            //cout << "smaller idx: " << min_element_iterator - sizes.begin() << endl;
            //cout << "min direction " << min_element_direction << endl;
            break;
        }
        last_layer_good = condition;
    }
    if (radius > max_radius) {
        return nullptr;
    }
    Vector3 move_to_min = min_element_direction / 2;
    IntVector move_int = IntVector(move_to_min.x(), move_to_min.y(), move_to_min.z());

    IntPoint inner_center = center + move_int;
    inner_center = closest_point(inner_center);
    //cout << "Inner center: " <<  inner_center << endl;
    auto inner_junction = junctions_in_cube(inner_center, radius - 1);

    IntPoint outer_center = center - move_int;
    outer_center = closest_point(outer_center);
    //cout << "Outer center: " <<  outer_center << endl;
    auto outer_junction = junctions_in_cube(outer_center, radius - 1);
    if (inner_junction and (outer_junction == nullptr or inner_junction->radius <= outer_junction->radius)) {
        return inner_junction;
    }

    if (outer_junction and (inner_junction == nullptr or outer_junction->radius < inner_junction->radius)) {
        return outer_junction;
    }
    auto res = make_shared<JunctionDescriptor>();
    res->radius = radius;
    res->cc_center = from_int_point(center) + min_element_direction;
    res->center = center;
    boost::graph_traits<SphereGraph>::vertex_iterator vertices_iter, vertices_end;
    for (boost::tie(vertices_iter, vertices_end) = boost::vertices(graph);
         vertices_iter != vertices_end; ++vertices_iter) {
        if (component[*vertices_iter] == min_element_idx)
            res->small_cc.push_back(from_int_point(center + IntVector(CGAL::ORIGIN, graph[*vertices_iter])));
    }
    res->plane = Plane3(res->small_cc.front(), min_element_direction);
    return res;
}

void Polyhedron::recognize_junctions() {
    cout << "recognizing junctions..." << endl;
    int initial_cube_size = 20;

    set<IntPoint> workset;
    for (int x = 0; x < _resolution; x++) {
        for (int y = 0; y < _resolution; y++) {
            for (int z = 0; z < _resolution; z++) {
                auto point = IntPoint(x, y, z);
                if (_voxels[voxel_idx(point)]) {
                    workset.insert(point);
                }
            }
        }
    }
    vector<shared_ptr<JunctionDescriptor>> temp_junctions;
    while (not workset.empty()) {
        IntPoint start = *workset.begin();
        workset.erase(start);
        //cout << "searching " << start << endl;
        auto junction = junctions_in_cube(start, initial_cube_size);
        int travel_distance;
        if (junction == nullptr) {
            travel_distance = initial_cube_size;
        } else {
            temp_junctions.push_back(junction);
            auto diff = junction->cc_center - from_int_point(start);
            travel_distance = (int) max(abs(diff.x()), max(abs(diff.y()), abs(diff.z())));
        }
        //cout << "travel: " << travel_distance << endl;
        queue<pair<IntPoint, int> > bfs;
        bfs.push(pair<IntPoint, int>(start, travel_distance));
        while (not bfs.empty()) {
            auto current = bfs.front();
            bfs.pop();
            //_voxels[voxel_idx(current.first)] = false;
            if (current.second < 1)
                continue;
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dz = -1; dz <= 1; dz++) {
                        auto next = current.first + IntVector(dx, dy, dz);
                        if (workset.find(next) != workset.end()) {
                            workset.erase(next);
                            bfs.push(pair<IntPoint, int>(next, current.second - 1));
                        }
                    }
                }
            }
        }
    }


    for (auto &junction : temp_junctions) {
        _junctions.push_back(junction);
//        Plane3 plane = junction->plane;
//        auto point_on_cut = junction->small_cc.front();
//        auto choice = intersect_single_line(plane, point_on_cut);
//        try {
//            auto cut_result = cut(choice);
//            if (cut_result.size() == 1)
//                _junctions.push_back(junction);
//        } catch (std::runtime_error& e) {
//            cout << "Crashed.." << endl;
//        }
    }

}

void Polyhedron::tips_from_plane(Plane3 plane) {
    auto normal = plane.orthogonal_vector();
    normal /= sqrt(CGAL::to_double(normal.squared_length()));
    queue<IntPoint> worklist;
    vector<int> visit_time;
    vector<Point3> planedots;
    auto total_size = (uint) pow(_resolution, 3);
    visit_time.reserve(total_size);
    visit_time.assign(total_size, 0);
    int next_time = 1;

    for (int x = 0; x < _resolution; x++) {
        for (int y = 0; y < _resolution; y++) {
            for (int z = 0; z < _resolution; z++) {
                auto ipoint = IntPoint(x, y, z);
                if (CGAL::squared_distance(from_int_point(ipoint), plane) < 1) {
                    worklist.push(ipoint);
                    planedots.push_back(from_int_point(ipoint));
                    visit_time[voxel_idx(ipoint)] = 1;
                }
            }
        }
    }

    while (not worklist.empty()) {
        IntPoint current_point = worklist.front();
        worklist.pop();
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                for (int dz = -1; dz <= 1; dz++) {
                    auto next_point = current_point + IntVector(dx, dy, dz);
                    auto vidx = voxel_idx(next_point);
                    if (vidx == -1)
                        continue;
                    if (_voxels[vidx] and visit_time[vidx] == 0) {
                        worklist.push(next_point);
                        visit_time[vidx] = next_time++;
                    }
                }
            }
        }
    }

//    if (planedots.size() == 0)
//        cout << "Bad: " << plane << endl;
//    auto junction = make_shared<JunctionDescriptor>();
//    junction->center = IntPoint(0,0,0);
//    junction->small_cc = planedots;
//    this->_junctions.push_back(junction);


    for (int x = 0; x < _resolution; x++) {
        for (int y = 0; y < _resolution; y++) {
            for (int z = 0; z < _resolution; z++) {
                auto current_point = IntPoint(x, y, z);
                int vidx = voxel_idx(current_point);
                if (not _voxels[vidx]) //or visit_time[vidx] == 0)
                    continue;
                bool highest = true;
                int dist = 3;
                for (int dx = -dist; dx <= dist && highest; dx++) {
                    for (int dy = -dist; dy <= dist && highest; dy++) {
                        for (int dz = -dist; dz <= dist && highest; dz++) {
                            auto next_point = current_point + IntVector(dx, dy, dz);
                            int nidx = voxel_idx(next_point);
                            if (nidx != -1 and visit_time[nidx] > visit_time[vidx])
                                highest = false;
                        }
                    }
                }
                if (highest) {
                    auto tip = make_shared<TipDescriptor>();
                    tip->point = from_int_point(current_point);
                    tip->direction = normal;
                    this->_tips.push_back(tip);
                }
            }
        }
    }
}


void Polyhedron::max_suppress(vector<double> &values) {
    SphereGraph graph;
    map<IntPoint, SphereGraph::vertex_descriptor> sphere;
    for (uint x = 0; x < _resolution; x++) {
        for (uint y = 0; y < _resolution; y++) {
            for (uint z = 0; z < _resolution; z++) {
                auto ipoint = IntPoint(x, y, z);
                auto idx = voxel_idx(ipoint);
                if (values[idx] >= 40) {
                    sphere[ipoint] = boost::add_vertex(ipoint, graph);
                }
            }
        }
    }

    for (uint x = 0; x < _resolution; x++) {
        for (uint y = 0; y < _resolution; y++) {
            for (uint z = 0; z < _resolution; z++) {
                auto ipoint = IntPoint(x, y, z);
                auto idx = voxel_idx(ipoint);
                if (sphere.count(ipoint) == 0)
                    continue;
                for (int dx = -1; dx <= 1; dx++) {
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dz = -1; dz <= 1; dz++) {
                            if (dx == 0 and dy == 0 and dz == 0)
                                continue;
                            IntVector dir = IntVector(dx, dy, dz);
                            auto offset = ipoint + dir;
                            if (sphere.count(offset) == 0)
                                continue;
                            boost::add_edge(sphere[ipoint], sphere[offset], graph);
                        }
                    }
                }
            }
        }
    }

    //calculate CCs
    std::vector<int> component;
    component.assign(boost::num_vertices(graph), -1);
    auto num_comp = boost::connected_components(graph, &component[0]);
    //cout << num_comp << endl;

    vector<IntPoint> maximal;
    maximal.assign(num_comp, IntPoint(0, 0, 0));
    vector<double> max_value;
    max_value.assign(num_comp, 0);
    for (uint x = 0; x < _resolution; x++) {
        for (uint y = 0; y < _resolution; y++) {
            for (uint z = 0; z < _resolution; z++) {
                auto ipoint = IntPoint(x, y, z);
                auto vidx = voxel_idx(ipoint);
                if (sphere.count(ipoint) != 0) {
                    auto c = component[sphere[ipoint]];
                    if (max_value[c] < values[vidx]) {
                        max_value[c] = values[vidx];
                        maximal[c] = ipoint;
                    }
                }
                values[vidx] = 0;
            }
        }
    }

    for (auto c = 0; c < num_comp; c++) {
        values[voxel_idx(maximal[c])] = max_value[c];
    }



//    vector<bool> supp;
//    supp.reserve(values.size());
//    supp.assign(values.size(), false);
//    auto res = _resolution;
//    for (int x = 0; x < res; x++) {
//        for (int y = 0; y < res; y++) {
//            for (int z = 0; z < res; z++) {
//                auto p = IntPoint(x,y,z);
//                auto pidx = voxel_idx(p);
//                for (int dx = -1; dx <= 1; dx++) {
//                    for (int dy = -1; dy <= 1; dy++) {
//                        for (int dz = -1; dz <= 1; dz++) {
//                            if (dx == 0 and dy == 0 and dz == 0)
//                                continue;
//                            auto didx = voxel_idx(p + IntVector(dx,dy,dz));
//                            if (didx == -1)
//                                continue;
//                            if (values[didx] >= values[pidx])
//                                supp[pidx] = true;
//                        }
//                    }
//                }
//            }
//        }
//    }
//    for (int x = 0; x < res; x++) {
//        for (int y = 0; y < res; y++) {
//            for (int z = 0; z < res; z++) {
//                auto p = IntPoint(x,y,z);
//                auto pidx = voxel_idx(p);
//                if (supp[pidx])
//                    values[pidx] = 0;
//            }
//        }
//    }
}

void Polyhedron::recognize_tips() {
    cout << "recognizing tips" << endl;
    vector<IntVector> directions;
    vector<Vector3> normal_directions;
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            for (int dz = -1; dz <= 1; dz++) {
                if (dx == 0 and dy == 0 and dz == 0)
                    continue;
                directions.emplace_back(dx, dy, dz);
                normal_directions.push_back(-normalize(Vector3(dx, dy, dz)));
            }
        }
    }
    for (int level = 0; level <= 5; level++) {
        for (uint positive = 0; positive < directions.size(); positive++) {
            vector<double> values;
            vector<int> direction;
            auto total_size = (uint) pow(_resolution, 3);
            values.reserve(total_size);
            values.assign(total_size, 0);
            direction.reserve(total_size);
            direction.assign(total_size, 0);
            auto pow_level = pow(2, level);
            for (int x = 0; x < _resolution; x++) {
                for (int y = 0; y < _resolution; y++) {
                    for (int z = 0; z < _resolution; z++) {
                        auto p = IntPoint(x, y, z);
                        auto pidx = voxel_idx(p);
                        if (not _voxels[pidx])
                            continue;

                        auto positive_point = p + pow_level * directions[positive];
                        auto positive_idx = voxel_idx(positive_point);
                        if (positive_idx == -1 or not _voxels[positive_idx])
                            continue;
                        auto value = 25;
                        for (uint other = 0; other < directions.size(); other++) {
                            if (other == positive)
                                continue;
                            auto other_point = p + pow_level * directions[other];
                            auto other_idx = voxel_idx(other_point);
                            if (other_idx == -1 or not _voxels[other_idx])
                                value++;
                            else
                                value--;
                        }
                        values[pidx] = value;
                    }
                }
            }
            max_suppress(values);
            uint vidx = 0;
            for (int x = 0; x < _resolution; x++) {
                for (int y = 0; y < _resolution; y++) {
                    for (int z = 0; z < _resolution; z++) {
                        if (values[vidx] > 40) {
                            auto tip = make_shared<TipDescriptor>();
                            tip->point = from_int_point(IntPoint(x, y, z));
                            tip->direction = normal_directions[positive];
                            tip->original = false;
                            _tips.push_back(tip);
                        }
                        vidx++;
                    }
                }
            }
        }
    }
}

void Polyhedron::voxel_cube(IntPoint center, int max_radius) {
    //auto tip = tips_in_cube(center, max_radius);
    //cout << "Tip? " << (tip != nullptr) << endl;
    //return;
    vector<bool> object;
    object.insert(object.begin(), (uint) pow(2 * max_radius + 1, 3), false);
    object[CUBE_IDX_POINT(IntPoint(0, 0, 0), max_radius)] = true;
    queue<IntPoint> last_level;
    last_level.push(IntPoint(0, 0, 0));
    queue<IntPoint> next_level;

    Vector3 cc_center(0, 0, 0);
    std::vector<int> component;
    SphereGraph graph;

    int radius;
    for (radius = 1; radius < max_radius; radius++) {
        while (not last_level.empty()) {
            auto current = last_level.front();
            last_level.pop();
            IntPoint current_point = center + IntVector(IntPoint(CGAL::ORIGIN), current);
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dz = -1; dz <= 1; dz++) {
                        auto next_point = current_point + IntVector(dx, dy, dz);
                        if (voxel_idx(next_point) == -1)
                            continue;
                        auto relative = current + IntVector(dx, dy, dz);
                        //if (not(abs(relative.x()) <= radius and abs(relative.y()) <= radius
                        //        and abs(relative.z()) <= radius))
                        //    continue;
                        if (_voxels[voxel_idx(next_point)] and not object[CUBE_IDX_POINT(relative, max_radius)]) {
                            next_level.push(relative);
                            object[CUBE_IDX_POINT(relative, max_radius)] = true;
                        }
                    }
                }
            }
        }
        last_level = next_level;
        next_level = queue<IntPoint>();
    }
    if (last_level.empty())
        return;
    _junctions.clear();
    auto junction = make_shared<JunctionDescriptor>();
    junction->center = center;
    while (not last_level.empty()) {
        IntPoint current = last_level.front();
        last_level.pop();
        junction->small_cc.push_back(from_int_point(center + IntVector(CGAL::Origin(), current)));
    }
    _junctions.push_back(junction);
}
