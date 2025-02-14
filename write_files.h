#pragma once
#include "general_classes.h"


void exportToCSV(const std::vector<Point>& edges, const std::string& filename);
void exportTo_txt_with_total_information(const std::vector<Point>& edges, const std::string& filename, const float elapsed, const double best_score, const float gap, const float best_bound);
void appendToCSV(const std::string& filename, const std::vector<std::string>& row);
