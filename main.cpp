#include <boost/program_options.hpp>
#include "SliceApp.h"
#include "3D/Polyhedron.h"
#include "3D/Visualizer3D.h"

namespace po = boost::program_options;

double crit_angle;
double cos_crit_angle;

int main(int argc, char ** argv)
{
    srand((uint)time(nullptr));
    setbuf(stdout, nullptr);

    std::string shape = "";
    std::string distribution_string;
    double backtrack_lin, backtrack_exp;
    bool non_interactive;
    bool silent;
    int time_limit;
    int cut_limit;

    QApplication app(argc, argv);

    po::options_description desc("Allowed options");
    po::positional_options_description pos;
    desc.add_options()
            ("angle", po::value<double>(&crit_angle)->default_value(135), "critical overhang angle")
            ("methods", po::value<std::string>(&distribution_string)->default_value("0.5,0.3,0.1,0.1"),
             "distribution of methods for search (4 values)")
            ("bt-linear", po::value<double>(&backtrack_lin)->default_value(0.985), "backtrack linear probability")
            ("bt-exp", po::value<double>(&backtrack_exp)->default_value(0.76), "backtrack exponential probability")
            ("file", po::value<std::string>(&shape), "file to process")
            ("automatic,a", po::bool_switch(&non_interactive)->default_value(false), "don't open windows until result")
            ("silent", po::bool_switch(&silent)->default_value(false), "no debug prints")
            ("time-limit", po::value<int>(&time_limit)->default_value(50), "runtime limit in minutes")
            ("cut-limit", po::value<int>(&cut_limit)->default_value(5000), "amount of cuts to try before stopping")
            ("help", "print a help message");
    pos.add("file", -1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
    po::notify(vm);

    if (shape.empty() or vm.count("help")) {
        cout << desc << "\n";
        exit(0);
    }

    vector<double> distribution_vector;
    size_t start = 0, end = 0;
    while ((end = distribution_string.find(',', start)) != std::string::npos) {
        if (end != start)
            distribution_vector.push_back(atof(distribution_string.substr(start, end).c_str()));
        start = end + 1;
    }
    if (start != end)
        distribution_vector.push_back(atof(distribution_string.substr(start, end).c_str()));


    crit_angle = crit_angle/180*M_PI;
    cos_crit_angle = cos(crit_angle) + 0.001;

    Polyhedron * p = new Polyhedron(shape);
    cout << p->size() << endl;
    SliceApp slice_app(&app, p, shape, distribution_vector, backtrack_lin, backtrack_exp,
                                   not silent, time_limit, cut_limit);

    if (non_interactive) {
        slice_app.random_backtrack_wrapped();
        exit(0);
    }
    auto vis = slice_app.visualize();
    return app.exec();
}