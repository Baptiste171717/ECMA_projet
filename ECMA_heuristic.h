#pragma once
#include "general_classes.h"
#include <ilcplex/ilocplex.h>




int slave_problem_resolution(std::vector<bool> activated_arcs, std::vector<int> th, const int T, std::vector<Point> Arcs, std::vector<int> Arcs_time);


int completing_travelling_backward(const Point first_selected_arcs, std::vector<std::vector<Point>>& pre_activated_out_arcs, std::vector<std::vector<Point>>& pre_activated_in_arcs,
                                   const std::vector<int> d, int C, std::vector<bool>& activated_arcs, std::vector<bool>& visiting_cities, int& total_demand,
                                   const std::vector<int> out_depot_arcs);

int completing_travelling_forward(const Point first_selected_arcs, std::vector<std::vector<Point>>& out_arcs, std::vector<std::vector<Point>>& in_arcs, const std::vector<int> d, const int C, std::vector<bool>& activated_arcs,
                                  std::vector<bool>& visiting_cities, int& total_demand, const std::vector<int> in_depot_arcs);

void completing_total_traveling(std::vector<std::vector<Point>>& pre_activated_out_arcs, std::vector<std::vector<Point>>& pre_activated_in_arcs, const std::vector<Point> Arcs, std::vector<bool> pre_activated_arcs,
                                std::vector<int> d, int C, std::vector<bool>& visiting_cities, std::vector<bool>& activated_arcs);

void heuristic_creation_solution(const std::string path_to_complete_solution, const std::string path_to_solution, float& elapsed, bool* best_current_solution, int& best_score, const int solutions_table_size, const int generations_number,
                                 const int size_population_to_mutate, const float keeping_proportion, const float banning_proportion, const float keeping_proportion_outside_mix, const float keeping_proportion_inside_mix,
                                 const float banning_proportion_mix, const int tailleX, const std::vector<int> Arcs_time, const std::vector<Point> Arcs, const std::vector<int> th, const int T, const int C, const std::vector<int> d,
                                 const int n, float& gap, float& best_found,  int tabou_size, int maximum_number_banned_arcs);
