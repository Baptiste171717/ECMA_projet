#include <iostream>
#include <fstream>
#include <sstream>
#include "ECMA.h"
#include "ECMA_heuristic.h"


// Inclusion de la librairie Cplex
using namespace std;



// Macro pouvant etre necessaire dans visual studio
ILOSTLBEGIN

    int main(){

    srand(12);

    string number_instance = "95";
    string euclidean_bool = "false";

    string path = "C:/Users/33782/OneDrive/Documents/Projet_ECMA_Baptiste_Vert/data/n_"+number_instance+"-euclidean_"+euclidean_bool;

    ifstream inputFile(path);
    if (!inputFile) {
        cerr << "Error opening file" << endl;
        return 1;
    }

    int n, T, C;
    vector<vector<int>> t;
    vector<int> th, d;

    string line;
    while (getline(inputFile, line)) {
        istringstream iss(line);
        if (line.find("n =") != string::npos) {
            iss.ignore(4); // Skip "n = "
            iss >> n;
        } else if (line.find("t =") != string::npos) {
            iss.ignore(5); // Skip "t = ["
            t.resize(n, vector<int>(n));
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    iss >> t[i][j];
                    if (iss.peek() == ' '|| iss.peek() == ';' || iss.peek() == ']'){
                        iss.ignore();
                    }
                }
            }
        } else if (line.find("th =") != string::npos) {
            iss.ignore(6); // Skip "th = ["
            th.resize(n);
            for (int i = 0; i < n; ++i) {
                iss >> th[i];
                if (iss.peek() == ' '||iss.peek() == ',' || iss.peek() == ']') iss.ignore();
            }
        } else if (line.find("T =") != string::npos) {
            iss.ignore(4); // Skip "T = "
            iss >> T;
        } else if (line.find("d =") != string::npos) {
            iss.ignore(5); // Skip "d = ["
            d.resize(n);
            for (int i = 0; i < n; ++i) {
                iss >> d[i];
                if (iss.peek() == ',' || iss.peek() == ']'){
                    iss.ignore();
                }
            }
        } else if (line.find("C =") != string::npos) {
            iss.ignore(4); // Skip "C = "
            iss >> C;
        }
    }
    inputFile.close();


    vector<Point> Arcs;
    int* arcs_coordinates_table = new int[n*n];
    vector<int> Arcs_time;
    vector<vector<int>> in_arcs(n);
    vector<vector<int>> out_arcs(n);
    int arc_count = 0;

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if(t[i][j]!=0){
                arcs_coordinates_table[n*i +j] = arc_count;
                Arcs.push_back(Point(i,j));
                Arcs_time.push_back(t[i][j]);
                in_arcs[j].push_back(arc_count);
                out_arcs[i].push_back(arc_count);
                arc_count++;
            }
            else{
                arcs_coordinates_table[n*i +j] = -1;
            }
        }
    }


    const int tailleX = Arcs.size();

    float heuristic_elapsed = 0;
    int heuristic_best_score = 0;
    float heuristic_gap =100;
    bool* best_heuristic_current_solution = new bool[tailleX];

    const int solutions_table_size =140;
    const int generations_number = 40;
    const int size_population_to_mutate = 50;
    const float keeping_proportion = 0.3;
    const float banning_proportion = 0.4;
    const float keeping_proportion_outside_mix = 0.3;
    const float keeping_proportion_inside_mix = 0.2;
    const float banning_proportion_mix = 0.05; //actually here it is the opposite way
    int tabou_size = 10;
    int maximum_number_banned_arcs = tailleX*0.5;


    const int time_solver = 1000;


    float dualisation_elapsed = 0;
    double dualisation_best_score = 99999999;
    float dualisation_gap = 100;
    bool* best_dualisation_current_solution = new bool[tailleX];
    float best_bound = 10;


    float static_elapsed = 0;
    double static_best_score = 99999999;
    float PR_gap = 100;
    bool* best_static_current_solution = new bool[tailleX];


    float lazy_callback_elapsed = 0;
    double lazy_callback_best_score = 999999;
    float lazy_callback_gap = 0;
    bool* best_lazy_callback_current_solution = new bool[tailleX];

    const int cutting_planes_time_solver = time_solver/20;

    float cutting_planes_elapsed = 0;
    double cutting_planes_best_score = 999999;
    float cutting_planes_gap = 0;
    bool* best_cutting_planes_current_solution = new bool[tailleX];
    chrono::milliseconds max_duration(time_solver*1000);



    const string path_to_complete_heuristic_solution = "C:/Users/33782/OneDrive/Documents/Projet_ECMA_Baptiste_Vert/solutions/complete_solution_heuristic_n_"+number_instance+"-euclidean_"+euclidean_bool+".txt";
    const string path_to_heuristic_solution = "C:/Users/33782/OneDrive/Documents/Projet_ECMA_Baptiste_Vert/solutions/solution_heuristic_n_"+number_instance+"-euclidean_"+ euclidean_bool+".csv";

    string path_to_complete_dualisation_solution = "C:/Users/33782/OneDrive/Documents/Projet_ECMA_Baptiste_Vert/solutions/complete_solution_dualisation_n_"+number_instance+"-euclidean_"+euclidean_bool+".txt";
    string path_to_dualisation_solution = "C:/Users/33782/OneDrive/Documents/Projet_ECMA_Baptiste_Vert/solutions/solution_dualisation_n_"+number_instance+"-euclidean_"+euclidean_bool+".csv";

    string path_to_complete_static_solution = "C:/Users/33782/OneDrive/Documents/Projet_ECMA_Baptiste_Vert/solutions/complete_solution_static_n_"+number_instance+"-euclidean_"+euclidean_bool+".txt";
    string path_to_static_solution = "C:/Users/33782/OneDrive/Documents/Projet_ECMA_Baptiste_Vert/solutions/solution_static_n_"+number_instance+"-euclidean_"+euclidean_bool+".csv";

    string path_to_complete_lazy_call_back_solution = "C:/Users/33782/OneDrive/Documents/Projet_ECMA_Baptiste_Vert/solutions/complete_solution_lazy_call_back_n_"+number_instance+"-euclidean_"+euclidean_bool+".txt";
    string path_to_lazy_call_back_solution = "C:/Users/33782/OneDrive/Documents/Projet_ECMA_Baptiste_Vert/solutions/solution_lazy_call_back_n_"+number_instance+"-euclidean_"+euclidean_bool+".csv";

    string path_to_complete_cutting_planes_solution = "C:/Users/33782/OneDrive/Documents/Projet_ECMA_Baptiste_Vert/solutions/complete_solution_cutting_planes_n_"+number_instance+"-euclidean_"+euclidean_bool+".txt";
    string path_to_cutting_planes_solution = "C:/Users/33782/OneDrive/Documents/Projet_ECMA_Baptiste_Vert/solutions/solution_cutting_planes_n_"+number_instance+"-euclidean_"+euclidean_bool+".csv";




    IloEnv env_dualisation;

    try {

        cout << "Solving the robust problem by dualisation : " << endl;
        resolution_by_dualisation(path_to_complete_dualisation_solution,path_to_dualisation_solution,env_dualisation, n, C, T, th, t, d, Arcs, Arcs_time, in_arcs, out_arcs, tailleX, best_dualisation_current_solution, dualisation_best_score,
                                  dualisation_elapsed, dualisation_gap, best_bound, time_solver);

    } catch (IloException& e) {
        cerr << "Concert exception caught: " << e << endl;
    } catch (...) {
        cerr << "Unknown exception caught" <<endl;
    }

    env_dualisation.end();


    IloEnv env_static;


    try {

        cout << "Solving the static problem : " << endl;
        static_resolution(path_to_complete_static_solution, path_to_static_solution, env_static, n, C, T, th, t,
                          d, Arcs, Arcs_time, in_arcs, out_arcs, tailleX, best_static_current_solution, static_best_score,
                          static_elapsed, PR_gap, dualisation_best_score, time_solver);

    } catch (IloException& e) {
        cerr << "Concert exception caught: " << e << endl;
    } catch (...) {
        cerr << "Unknown exception caught" <<endl;
    }

    env_static.end();

    cout << "Solving the robust problem with the designed heuristic: " << endl;
    heuristic_creation_solution(path_to_complete_heuristic_solution, path_to_heuristic_solution, heuristic_elapsed, best_heuristic_current_solution, heuristic_best_score,
                                solutions_table_size, generations_number, size_population_to_mutate, keeping_proportion, banning_proportion, keeping_proportion_outside_mix,
                                keeping_proportion_inside_mix, banning_proportion_mix, tailleX, Arcs_time, Arcs, th, T, C, d, n, heuristic_gap, best_bound, tabou_size,
                                maximum_number_banned_arcs);




    IloEnv env_lazy_call_back;


    try {

        cout << "Solving the robust problem with lazy_call_back : " << endl;
        resolution_by_lazy_callback(path_to_complete_lazy_call_back_solution, path_to_lazy_call_back_solution ,env_lazy_call_back, n,  C, T, th, t, d, Arcs, Arcs_time, in_arcs, out_arcs, tailleX,
                                     best_lazy_callback_current_solution, lazy_callback_best_score, lazy_callback_elapsed, lazy_callback_gap, best_bound, time_solver);


    } catch (IloException& e) {
        cerr << "Concert exception caught: " << e << endl;
    } catch (...) {
        cerr << "Unknown exception caught" <<endl;
    }

    env_lazy_call_back.end();


    IloEnv env_cutting_planes;


    try {

        cout << "Solving the robust problem with cutting planes : " << endl;
        resolution_by_cutting_planes(path_to_complete_cutting_planes_solution, path_to_cutting_planes_solution, env_cutting_planes, n,  C, T, th, t, d, Arcs, Arcs_time, in_arcs, out_arcs, tailleX,
                                     best_cutting_planes_current_solution, cutting_planes_best_score, cutting_planes_elapsed, cutting_planes_gap, best_bound, cutting_planes_time_solver, max_duration, arcs_coordinates_table);


    } catch (IloException& e) {
        cerr << "Concert exception caught: " << e << endl;
    } catch (...) {
        cerr << "Unknown exception caught" <<endl;
    }

    env_cutting_planes.end();



//    vector<string> columns_name = {"Instance", "Valeur statique", "PR", "Meilleure borne robuste", "Plans coupants", "Plans coupants", "Branch-and-cut", "Branch-and-cut", "Dualisation", "Dualisation", "Heuristique",
//                                   "Heuristique"};
//    vector<string> second_columns_name = {"", "", "", "", "Time", "Gap", "Time", "Gap", "Time", "Gap", "Time", "Gap"};

    double value = PR_gap*100;
    ostringstream oss;
    oss << fixed << setprecision(2) << value;
    string strValue_PR = oss.str();
    oss.str("");

    value = dualisation_elapsed;
    oss << fixed << setprecision(2) << value;
    string strValue_dualisation_elapsed = oss.str();
    oss.str("");

    value = heuristic_elapsed;
    oss << fixed << setprecision(2) << value;
    string strValue_heuristic_elapsed = oss.str();
    oss.str("");

    value = lazy_callback_elapsed;
    oss << fixed << setprecision(2) << value;
    string strValue_lazy_callback_elapsed = oss.str();
    oss.str("");

    value = cutting_planes_elapsed;
    oss << fixed << setprecision(2) << value;
    string strValue_cutting_planes_elapsed = oss.str();
    oss.str("");

    value = dualisation_gap*100;
    oss << fixed << setprecision(2) << value;
    string strValue_dualisation_gap = oss.str();
    oss.str("");

    value = heuristic_gap*100;
    oss << fixed << setprecision(2) << value;
    string strValue_heuristic_gap = oss.str();
    oss.str("");

    value = lazy_callback_gap*100;
    oss << fixed << setprecision(2) << value;
    string strValue_lazy_callback_gap = oss.str();
    oss.str("");

    value = cutting_planes_gap*100;
    oss << fixed << setprecision(2) << value;
    string strValue_cutting_planes_gap = oss.str();
    oss.str("");

    value = dualisation_best_score;
    oss << fixed << setprecision(2) << value;
    string strValue_dualisation_best_score = oss.str();
    oss.str("");

    value = static_best_score;
    oss << fixed << setprecision(2) << value;
    string strValue_static_best_score = oss.str();
    oss.str("");

    value = best_bound;
    oss << fixed << setprecision(2) << value;
    string strValue_best_bound = oss.str();
    oss.str("");


    vector<string> newRow = {number_instance+"-euclidean_"+euclidean_bool, strValue_static_best_score, strValue_PR, strValue_best_bound, strValue_cutting_planes_elapsed,
                             strValue_cutting_planes_gap, strValue_lazy_callback_elapsed, strValue_lazy_callback_gap,
                             strValue_dualisation_elapsed, strValue_dualisation_gap, strValue_heuristic_elapsed, strValue_heuristic_gap};


//    appendToCSV("C:/Users/33782/OneDrive/Documents/Projet_ECMA_Baptiste_Vert/results.csv", columns_name);
//    appendToCSV("C:/Users/33782/OneDrive/Documents/Projet_ECMA_Baptiste_Vert/results.csv", second_columns_name);
    appendToCSV("C:/Users/33782/OneDrive/Documents/Projet_ECMA_Baptiste_Vert/results.csv", newRow);

    delete [] best_heuristic_current_solution;
    delete [] best_dualisation_current_solution;
    delete [] best_static_current_solution;
    delete [] best_lazy_callback_current_solution;
    delete [] best_cutting_planes_current_solution;
    delete [] arcs_coordinates_table;

    return 0;

}

