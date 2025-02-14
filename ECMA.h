#pragma once

#include "general_classes.h"
#include <ilcplex/ilocplex.h>
#include <chrono>
#include "write_files.h"


class MyLazyCallback : public IloCplex::LazyConstraintCallbackI {
public:
    MyLazyCallback(IloEnv env, const IloNumVar& z, const IloBoolVarArray& vars, const std::vector<int>& th, int tailleX, const std::vector<Point>& Arcs, const int T,  std::vector<int> Arcs_time,
                   std::string path_to_complete_solution, std::string path_to_solution, bool* best_lazy_callback_current_solution, double& lazy_callback_best_score, float& lazy_callback_elapsed, float& gap,
                   float best_bound,  std::chrono::steady_clock::time_point startTime)
        : IloCplex::LazyConstraintCallbackI(env),z(z), vars(vars), th(th), tailleX(tailleX), Arcs(Arcs), T(T), Arcs_time(Arcs_time), path_to_complete_solution(path_to_complete_solution),
        path_to_solution(path_to_solution), best_lazy_callback_current_solution(best_lazy_callback_current_solution), lazy_callback_best_score(lazy_callback_best_score),
        lazy_callback_elapsed(lazy_callback_elapsed), gap(gap), best_bound(best_bound), startTime(startTime){}

    void main() override;

    IloCplex::CallbackI* duplicateCallback() const override {
        return new (getEnv()) MyLazyCallback(getEnv(), z, vars, th, tailleX, Arcs, T, Arcs_time, path_to_complete_solution, path_to_solution, best_lazy_callback_current_solution, lazy_callback_best_score,
                                                    lazy_callback_elapsed, gap, best_bound, startTime);
    }

private:
    IloBoolVarArray vars;
    IloIntVar z;
    std::vector<int> Arcs_time;
    std::vector<int> th;
    int tailleX;
    int T;
    std::vector<Point> Arcs;
    std::string path_to_complete_solution;
    std::string path_to_solution;
    bool* best_lazy_callback_current_solution;
    double& lazy_callback_best_score;
    float& lazy_callback_elapsed;
    float& gap;
    float best_bound;
    std::chrono::steady_clock::time_point startTime;

};




void resolution_by_dualisation( std::string path_to_complete_solution, std::string path_to_solution, IloEnv& env, int n, int C, int T, std::vector<int> th, std::vector<std::vector<int>> t, std::vector<int> d, std::vector<Point> Arcs,
                                std::vector<int> Arcs_time, std::vector<std::vector<int>> in_arcs, std::vector<std::vector<int>> out_arcs, const int tailleX, bool* best_dualisation_current_solution,
                                double& dualisation_best_score, float& dualisation_elapsed, float& gap, float& best_bound, const int time_solver);

void resolution_by_lazy_callback(std::string path_to_complete_solution, std::string path_to_solution, IloEnv& env, int n, int C, int T, std::vector<int> th, std::vector<std::vector<int>> t, std::vector<int> d, std::vector<Point> Arcs,
                                  std::vector<int> Arcs_time, std::vector<std::vector<int>> in_arcs, std::vector<std::vector<int>> out_arcs, const int tailleX,
                                  bool* best_lazy_callback_current_solution, double& lazy_callback_best_score, float& lazy_callback_elapsed, float& gap, float best_bound, const int time_solver);



int cutting_planes_slave_problem_resolution(const IloNumArray& xValues, std::vector<int> th, const int T, std::vector<Point> Arcs, std::vector<int> Arcs_time, float* delta_1_solution,
                                            float* delta_2_solution, int* arcs_coordinates_table, const int n);

bool checkSlaveProblem(const  IloNumArray& xValues, double zValue, std::vector<int> th, std::vector<Point> Arcs, std::vector<int> Arcs_time, int T, float* delta_1_Solution, float* delta_2_solution, int& test_score,
                       int* arcs_coordinates_table, const int n);

IloRange generateCut(IloEnv env, const IloBoolVarArray& x, const IloIntVar& z, std::vector<Point> Arcs,  std::vector<int> Arcs_time, std::vector<int> th, float* delta_1_Solution, float* delta_2_Solution);

void resolution_by_cutting_planes(std::string path_to_complete_solution, std::string path_to_solution, IloEnv& env, int n, int C, int T, std::vector<int> th, std::vector<std::vector<int>> t,
                                  std::vector<int> d, std::vector<Point> Arcs, std::vector<int> Arcs_time, std::vector<std::vector<int>> in_arcs, std::vector<std::vector<int>> out_arcs, const int tailleX,
                                  bool* best_cutting_planes_current_solution, double& cutting_planes_best_score, float& cutting_planes_elapsed, float& gap, float& best_bound, const int time_solver, std::chrono::milliseconds maxDuration,
                                  int* arcs_coordinates_table);
void static_resolution(std::string path_to_complete_solution, std::string path_to_solution, IloEnv& env, int n, int C, int T, std::vector<int> th, std::vector<std::vector<int>> t,
                                  std::vector<int> d, std::vector<Point> Arcs, std::vector<int> Arcs_time, std::vector<std::vector<int>> in_arcs, std::vector<std::vector<int>> out_arcs, const int tailleX,
                                  bool* best_static_current_solution, double& static_best_score, float& static_elapsed, float& PR_gap, double& dualisation_best_score, const int time_solver);

