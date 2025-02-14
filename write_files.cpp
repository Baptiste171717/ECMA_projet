#include "write_files.h"

using namespace std;



void exportTo_txt_with_total_information(const vector<Point>& edges, const string& filename, const float elapsed, const double best_score, const float gap, const float best_bound) {

    ofstream file(filename, ofstream::trunc);

    file<<"The score of this solution is "<< best_score<< " found after "<<elapsed<< " seconds with total gap = " <<gap<<"%.\n";
    file<<"\n";
    file<< "The best bound obtained with the dualised problem is "<< best_bound<<".\n";
    file<<"\n";
    file<<"Here is the list of activated arcs : \n";
    file<<"\n";
    file << "Source,Target\n";

    for (const auto& edge : edges) {

        file<< edge.get_x() << "," << edge.get_y() << "\n";

    }

    file.close();
}


void exportToCSV(const vector<Point>& edges, const string& filename) {
    ofstream file(filename, ofstream::trunc);
    file << "Source,Target\n";
    for (const auto& edge : edges) {
        file<< edge.get_x() << "," << edge.get_y() << "\n";
    }
    file.close();
}




void appendToCSV(const std::string& filename, const std::vector<std::string>& row) {
    std::ofstream file(filename, std::ios::app);
    for (size_t i = 0; i < row.size(); ++i) {
        file << row[i];
        if (i < row.size() - 1) file << ",";
    }
    file << "\n";
    file.close();
}


