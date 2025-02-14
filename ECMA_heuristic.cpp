#include "ECMA_heuristic.h"
#include "write_files.h"
#include <set>
#include <unordered_set>
#include <chrono>

using namespace std;


int completing_travelling_backward(const Point first_selected_arcs, vector<vector<Point>>& pre_activated_out_arcs, vector<vector<Point>>& pre_activated_in_arcs, const vector<int> d, const int C,
                                   vector<bool>& activated_arcs, vector<bool>& visiting_cities, int& total_demand, const vector<int> out_depot_arcs ){



    if(pre_activated_in_arcs[first_selected_arcs.get_x()].empty()){

        activated_arcs[out_depot_arcs[first_selected_arcs.get_x()]] = true;
        return total_demand;

    }

    else{

        bool found_new_arc = false;

        while(!found_new_arc && !pre_activated_in_arcs[first_selected_arcs.get_x()].empty()){

            Point selected_in_arcs(pre_activated_in_arcs[first_selected_arcs.get_x()].back().get_y(), first_selected_arcs.get_x());
            int number_selected_arc = pre_activated_in_arcs[first_selected_arcs.get_x()].back().get_x();

            pre_activated_in_arcs[first_selected_arcs.get_x()].pop_back();

            if((!visiting_cities[selected_in_arcs.get_x()] ||selected_in_arcs.get_x() == 0)  && (total_demand + d[selected_in_arcs.get_x()]) <= C){

                found_new_arc =true;
                visiting_cities[selected_in_arcs.get_x()] = true;
                pre_activated_in_arcs[first_selected_arcs.get_x()].clear();
                pre_activated_out_arcs[selected_in_arcs.get_x()].clear();
                activated_arcs[number_selected_arc] = true;
                total_demand += d[selected_in_arcs.get_x()];

                if(selected_in_arcs.get_x()==0){
                    return total_demand;
                }

                total_demand = completing_travelling_backward(selected_in_arcs, pre_activated_out_arcs, pre_activated_in_arcs, d, C, activated_arcs, visiting_cities, total_demand, out_depot_arcs);

            }

        }

        if(found_new_arc){
            return total_demand;
        }

        else{

            activated_arcs[out_depot_arcs[first_selected_arcs.get_x()]] = true;
            return total_demand;

        }

    }

}

int completing_travelling_forward(const Point first_selected_arcs, vector<vector<Point>>& pre_activated_out_arcs, vector<vector<Point>>& pre_activated_in_arcs, const vector<int> d,
                                  int C, vector<bool>& activated_arcs, vector<bool>& visiting_cities, int& total_demand, const vector<int> in_depot_arcs){

    if(pre_activated_out_arcs[first_selected_arcs.get_y()].empty()){

        activated_arcs[in_depot_arcs[first_selected_arcs.get_y()]] = true;
        return total_demand;

    }

    else{

        bool found_new_arc = false;

        while(!found_new_arc && !pre_activated_out_arcs[first_selected_arcs.get_y()].empty()){

            Point selected_out_arcs(first_selected_arcs.get_y(), pre_activated_out_arcs[first_selected_arcs.get_y()].back().get_y());
            int number_selected_arc = pre_activated_out_arcs[first_selected_arcs.get_y()].back().get_x();
            pre_activated_out_arcs[first_selected_arcs.get_y()].pop_back();

            if((!visiting_cities[selected_out_arcs.get_y()] ||selected_out_arcs.get_y()==0)  && total_demand+ d[selected_out_arcs.get_y()] <= C){

                found_new_arc =true;
                visiting_cities[selected_out_arcs.get_y()] = true;
                pre_activated_out_arcs[first_selected_arcs.get_y()].clear();
                pre_activated_in_arcs[selected_out_arcs.get_y()].clear();

                activated_arcs[number_selected_arc] = true;
                total_demand += d[selected_out_arcs.get_y()];

                if(selected_out_arcs.get_y()==0){
                    return total_demand;
                }

                total_demand = completing_travelling_forward(selected_out_arcs, pre_activated_out_arcs, pre_activated_in_arcs, d, C, activated_arcs, visiting_cities, total_demand, in_depot_arcs);

            }

        }

        if(found_new_arc){

            return total_demand;

        }
        else{

            activated_arcs[in_depot_arcs[first_selected_arcs.get_y()]] = true;
            return total_demand;

        }
    }

}


void completing_total_traveling(vector<vector<Point>>& pre_activated_out_arcs, vector<vector<Point>>& pre_activated_in_arcs, const vector<Point> Arcs,
                                vector<bool> pre_activated_arcs, vector <int> d, int C, vector<bool>& visiting_cities, vector<bool>& activated_arcs){

    vector<int> out_depot_arcs(visiting_cities.size());
    vector<int> in_depot_arcs(visiting_cities.size());



    for (int a = 0; a < Arcs.size(); ++a) {

        if(Arcs[a].get_y()==0){
            in_depot_arcs[Arcs[a].get_x()] = a ;
        }
        if(Arcs[a].get_x()==0){
            out_depot_arcs[Arcs[a].get_y()] = a;
        }

    }



    for (int a = 0; a < Arcs.size(); ++a) {

        if(pre_activated_arcs[a] && (!visiting_cities[Arcs[a].get_x()] || Arcs[a].get_x() ==0) && (!visiting_cities[Arcs[a].get_y()] || Arcs[a].get_y() ==0) && d[Arcs[a].get_x()] + d[Arcs[a].get_y()] <= C){

            visiting_cities[Arcs[a].get_x()] = true;
            visiting_cities[Arcs[a].get_y()] = true;
            activated_arcs[a] = true;

            int total_demand = d[Arcs[a].get_x()] + d[Arcs[a].get_y()];

            if(Arcs[a].get_x()!= 0){

                total_demand = completing_travelling_backward(Arcs[a], pre_activated_out_arcs, pre_activated_in_arcs, d, C, activated_arcs, visiting_cities, total_demand, out_depot_arcs);

            }

            if(Arcs[a].get_y() !=0){

                total_demand = completing_travelling_forward(Arcs[a], pre_activated_out_arcs, pre_activated_in_arcs, d, C, activated_arcs, visiting_cities, total_demand, in_depot_arcs);

            }

        }

    }

}


int slave_problem_resolution(vector<bool> activated_arcs, const vector<int> th, const int T, const vector<Point> Arcs, const vector<int> Arcs_time){

    IloEnv env;

    int slaveObjectiveValue = 0;

    try {

        const int tailleX = Arcs.size();

        IloModel slaveModel(env);

        IloNumVarArray delta_1(env, tailleX, 0.0, 1);
        IloNumVarArray delta_2(env, tailleX, 0.0, 2);

        IloRangeArray constraints_1(env);
        IloRangeArray constraints_2(env);

        IloExpr expression(env);



        for (int a = 0; a < tailleX; ++a) {

            expression += (Arcs_time[a] + delta_1[a]*( th[Arcs[a].get_x()] + th[Arcs[a].get_y()]) + delta_2[a]*th[Arcs[a].get_x()]*th[Arcs[a].get_y()])*activated_arcs[a] ;

        }


        IloObjective obj(env, expression, IloObjective::Maximize);
        slaveModel.add(obj);
        expression.end();

        IloExpr expr(env);

        for (int a = 0; a < tailleX; ++a) {
            expr += delta_1[a];
        }

        constraints_1.add(expr <= T);
        slaveModel.add(constraints_1);
        expr.clear();

        for (int a = 0; a < tailleX; ++a) {
            expr += delta_2[a];
        }

        constraints_2.add(expr <= T*T);
        slaveModel.add(constraints_2);
        expr.clear();

        IloCplex slaveCplex(slaveModel);
        slaveCplex.setOut(env.getNullStream());

        if (!slaveCplex.solve()) {
            throw std::runtime_error("Slave problem could not be solved.");
        }


        slaveObjectiveValue = slaveCplex.getObjValue();
        constraints_1.end();
        constraints_2.end();
        delta_1.end();
        delta_2.end();
        slaveModel.end();

    } catch (IloException& e) {
        env.error() << "Exception caught: " << e << std::endl;
    }

    env.end();


    return slaveObjectiveValue;

}

void heuristic_creation_solution(const string path_to_complete_solution, const string path_to_solution, float& elapsed, bool* best_current_solution, int& best_score, const int solutions_table_size,
                                 const int generations_number, const int size_population_to_mutate, const float keeping_proportion, const float banning_proportion, const float keeping_proportion_outside_mix,
                                 const float keeping_proportion_inside_mix, const float banning_proportion_mix, const int tailleX, const vector<int> Arcs_time, const vector<Point> Arcs,
                                 const vector<int> th, const int T, const int C, const vector<int> d, const int n, float& gap, float& best_bound, int tabou_size, int maximum_number_banned_arcs){



    auto start_heuristic_time = chrono::high_resolution_clock::now();

    bool** solutions_table = new bool*[solutions_table_size];
    int* solutions_table_score = new int[solutions_table_size];
    int** tabou_arcs_solutions_table = new int*[solutions_table_size];
    int* tabou_number_banned_possibilities = new int[solutions_table_size];

    for (int i = 0; i < solutions_table_size; ++i) {

        solutions_table[i] = new bool[tailleX];
        solutions_table_score[i] = 0;

        for (int j = 0; j < tailleX; ++j) {
            solutions_table[i][j] = false;
        }

    }

    multiset<Point> ordered_arcs_by_time_set;

    for (int a = 0; a < tailleX; ++a) {
        ordered_arcs_by_time_set.insert(Point(a, Arcs_time[a]));
    }


    vector<bool> pre_activated_arcs(tailleX);
    vector<bool> activated_arcs(tailleX);
    vector<bool> visiting_cities(n);
    vector<vector<Point>> pre_activated_in_arcs(n);
    vector<vector<Point>> pre_activated_out_arcs(n);
    vector<int> out_depot_arcs(n);
    vector<int> in_depot_arcs(n);

    for (int a = 0; a < Arcs.size(); ++a) {

        if(Arcs[a].get_y()==0){
            in_depot_arcs[Arcs[a].get_x()] = a ;
        }
        if(Arcs[a].get_x()==0){
            out_depot_arcs[Arcs[a].get_y()] = a;
        }

    }


    for (const Point& point : ordered_arcs_by_time_set) {


        if( Arcs[point.get_x()].get_x()!=0 && Arcs[point.get_x()].get_y()!=0){

            pre_activated_arcs[point.get_x()] = true;

            pre_activated_in_arcs[Arcs[point.get_x()].get_y()].push_back(Point(point.get_x(), Arcs[point.get_x()].get_x()));
            pre_activated_out_arcs[Arcs[point.get_x()].get_x()].push_back(Point(point.get_x(),Arcs[point.get_x()].get_y()));
        }
        else{
            pre_activated_arcs[point.get_x()] = false;
        }

        activated_arcs[point.get_x()] = false;
    }


    for (int i = 0; i < n; ++i) {
        visiting_cities[i] =false;
    }



    completing_total_traveling(pre_activated_out_arcs, pre_activated_in_arcs, Arcs, pre_activated_arcs, d, C, visiting_cities, activated_arcs);

    for (int i = 1; i < n; ++i) {
        if(!visiting_cities[i]){

            activated_arcs[in_depot_arcs[i]] = true;
            activated_arcs[out_depot_arcs[i]] = true;

        }

    }



    best_score = slave_problem_resolution(activated_arcs,  th, T, Arcs, Arcs_time);

    solutions_table_score[0] = best_score;

    cout<<"the best_current_score is " <<best_score<<endl;


    bool* current_solution = new bool[tailleX];


    for (int a = 0; a < Arcs.size(); ++a) {
        best_current_solution[a] = activated_arcs[a];
        current_solution[a] = activated_arcs[a];
        solutions_table[0][a] = activated_arcs[a];
    }


    for (int member = 1; member < solutions_table_size; ++member) {



        int unchanged_variables_number = rand()%((int)round(tailleX*keeping_proportion));

        int completed_variables_number = rand()%((int)round(tailleX*banning_proportion));

        int banned_variables_number = tailleX - unchanged_variables_number -completed_variables_number;

        int total_number = unchanged_variables_number + banned_variables_number;

        unordered_set<int> total_concerned_index_set;

        vector<int> random_vector;

        while(total_concerned_index_set.size()!=total_number){

            int num = rand()%tailleX;

            if (total_concerned_index_set.insert(num).second) {

                random_vector.push_back(num);

            }

        }



        vector<int> modified_arcs_time(tailleX);

        for (int a = 0; a < tailleX; ++a) {

            modified_arcs_time[a] = -Arcs_time[a];

        }

        int min_int_arc = -10000;

        int max_int_arc = 10;

        for (int i = 0; i < unchanged_variables_number; ++i) {

            modified_arcs_time[random_vector[i]] = max_int_arc;

        }


        for (int i = unchanged_variables_number; i < unchanged_variables_number + banned_variables_number; ++i) {

            modified_arcs_time[random_vector[i]] = min_int_arc;

        }

        ordered_arcs_by_time_set.clear();

        for (int a = 0; a < tailleX; ++a) {

            ordered_arcs_by_time_set.insert(Point(a, modified_arcs_time[a]));

        }


        vector<bool> pre_activated_arcs_2(tailleX);
        vector<vector<Point>> pre_activated_in_arcs_2(n);
        vector<vector<Point>> pre_activated_out_arcs_2(n);

        for (const Point& point : ordered_arcs_by_time_set) {


            if( Arcs[point.get_x()].get_x()!=0 && Arcs[point.get_x()].get_y()!=0 && (modified_arcs_time[point.get_x()] !=  min_int_arc)){

                pre_activated_arcs_2[point.get_x()] = true;
                pre_activated_in_arcs_2[Arcs[point.get_x()].get_y()].push_back(Point(point.get_x(), Arcs[point.get_x()].get_x()));
                pre_activated_out_arcs_2[Arcs[point.get_x()].get_x()].push_back(Point(point.get_x(),Arcs[point.get_x()].get_y()));

            }
            else{

                pre_activated_arcs_2[point.get_x()] = false;

            }

            activated_arcs[point.get_x()] = false;
        }



        for (int i = 0; i < n; ++i) {
            visiting_cities[i] =false;
        }

        completing_total_traveling(pre_activated_out_arcs_2, pre_activated_in_arcs_2, Arcs, pre_activated_arcs_2, d, C, visiting_cities, activated_arcs);

        for (int i = 1; i < n; ++i) {
            if(!visiting_cities[i]){

                activated_arcs[in_depot_arcs[i]] = true;
                activated_arcs[out_depot_arcs[i]] = true;

            }

        }



        int current_score = slave_problem_resolution(activated_arcs,  th, T, Arcs, Arcs_time);

        solutions_table_score[member] = current_score;

        for (int a = 0; a < Arcs.size(); ++a) {
            current_solution[a] = activated_arcs[a];
            solutions_table[member][a] = activated_arcs[a];
        }

        cout<<"the current_score is " <<current_score<<endl;

        if(current_score <best_score){

            auto end_heuristic = chrono::high_resolution_clock::now();
            chrono::duration<double> duration_heuristic = end_heuristic - start_heuristic_time;
            elapsed = duration_heuristic.count();


            best_score = current_score;

            cout<< "the new best_score is " << best_score<< " found at time "<< elapsed<<endl;

            for (int a = 0; a < tailleX; ++a) {

                best_current_solution[a] = current_solution[a];

            }

            vector<Point> selected_arcs;

            for (int a = 0; a < tailleX; ++a) {

                if(best_current_solution[a]){

                    cout<<"use arc "<< Arcs[a].get_x()<< " "<< Arcs[a].get_y()<<endl;

                    selected_arcs.push_back(Arcs[a]);

                }

            }

            cout << "The total score of this heuristic solution is " <<best_score<< " found at time "<<elapsed <<endl;

            gap = (best_score-best_bound)/(best_bound+ 1e-10);

            exportToCSV(selected_arcs, path_to_solution);
            exportTo_txt_with_total_information(selected_arcs, path_to_complete_solution, elapsed, best_score, 100*gap, best_bound);

        }


    }



    for (int s = 0; s < solutions_table_size; ++s) {
        tabou_arcs_solutions_table[s] = new int[tailleX];
        tabou_number_banned_possibilities[s] = 0;
        for (int a = 0; a < tailleX; ++a) {
            tabou_arcs_solutions_table[s][a] = 0;
        }
    }


    bool multiply_tabou = false;
    int number_reset = 0 ;
    for (int population = 0; population < generations_number; ++population) {


        if(population > generations_number/2 && !multiply_tabou){
            number_reset +=1;
            multiply_tabou = (number_reset==10);
            for (int i = 0; i < (int)solutions_table_size*0.8; ++i) {

                solutions_table_score[i] = best_score;

                for (int j = 0; j < tailleX; ++j) {

                    solutions_table[i][j] = best_current_solution[j];

                }

            }
        }



        cout<<"this is the population number "<<population<<" over "<<generations_number<<endl;

        multiset<Point> ordered_solution_index_value_set;

        for (int i = 0; i < solutions_table_size; ++i) {

            ordered_solution_index_value_set.insert(Point(i, solutions_table_score[i]));

        }


        int* best_solution_index_array= new int[size_population_to_mutate]; //could be a pointer...

        if (number_reset < 10 && 0 < number_reset ){

            auto it = ordered_solution_index_value_set.rbegin(); // Reverse iterator to start from the largest element  //use r.begin for the k greatest value

            for (int i = 0; i < size_population_to_mutate && it != ordered_solution_index_value_set.rend(); ++i, ++it) { //use r.end for th k greastest value

                best_solution_index_array[i] = it->get_x();

            }

        }

        else{

            auto it = ordered_solution_index_value_set.begin(); // Reverse iterator to start from the largest element  //use r.begin for the k greatest value

            for (int i = 0; i < size_population_to_mutate && it != ordered_solution_index_value_set.end(); ++i, ++it) { //use r.end for th k greastest value

                best_solution_index_array[i] = it->get_x();

            }

        }




        for (int var = 0;  var< size_population_to_mutate; var+=2) {


            int unchanged_variables_inside_mix_number = rand()%((int)round(tailleX*keeping_proportion_inside_mix));

            int unchanged_variables_outside_mix_number = rand()%((int)round(tailleX*keeping_proportion_outside_mix));

            int completing_variables_outside_mix_number = rand()%((int)round(tailleX*banning_proportion_mix));

            int banned_variables_inside_mix_number = tailleX - unchanged_variables_inside_mix_number -unchanged_variables_outside_mix_number - completing_variables_outside_mix_number;

            int total_number = unchanged_variables_outside_mix_number + unchanged_variables_inside_mix_number + banned_variables_inside_mix_number;

            unordered_set<int> total_concerned_index_set;

            vector<int> random_vector;


            while(total_concerned_index_set.size()!=total_number){

                int num = rand()%tailleX;

                if (total_concerned_index_set.insert(num).second) {

                    random_vector.push_back(num);

                }

            }

            vector<int> modified_arcs_time(tailleX);

            for (int a = 0; a < tailleX; ++a) {

                modified_arcs_time[a] = -Arcs_time[a];

            }

            int max_int_arc = 10000;
            int min_int_arc = -10000; // for banned arc, need to be careful with the side elements are extracted from the vector


            for (int a = 0; a < unchanged_variables_outside_mix_number; ++a) {

                modified_arcs_time[random_vector[a]] = 0;

            }

            for(int a = unchanged_variables_outside_mix_number; a< unchanged_variables_inside_mix_number + unchanged_variables_outside_mix_number; a++){

                if(solutions_table[best_solution_index_array[var]][random_vector[a]]){

                    modified_arcs_time[random_vector[a]] = max_int_arc -Arcs_time[random_vector[a]];

                }
                else{

                    modified_arcs_time[random_vector[a]] = min_int_arc;

                }
            }



            for (int a = unchanged_variables_inside_mix_number+ unchanged_variables_outside_mix_number; a < total_number; ++a) {

                if(tabou_arcs_solutions_table[best_solution_index_array[var]][random_vector[a]]==0 && tabou_number_banned_possibilities[best_solution_index_array[var]] < maximum_number_banned_arcs){

                    tabou_arcs_solutions_table[best_solution_index_array[var]][random_vector[a]] = tabou_size;
                    tabou_number_banned_possibilities[best_solution_index_array[var]] +=1 ;

                }

            }

            int number_banned_arcs = 0;
            int number_debanned_arcs = 0;

            for (int a = 0; a < tailleX; ++a) {

                if(tabou_arcs_solutions_table[best_solution_index_array[var]][a] != 0 ){


                    modified_arcs_time[a] = min_int_arc;
                    tabou_arcs_solutions_table[best_solution_index_array[var]][a] -=1;
                    number_banned_arcs+=1;

                    if(tabou_arcs_solutions_table[best_solution_index_array[var]][a] == 0){
                        number_debanned_arcs +=1;
                        tabou_number_banned_possibilities[best_solution_index_array[var]] -= 1;
                    }

                }

            }


            ordered_arcs_by_time_set.clear();

            for (int a = 0; a < tailleX; ++a) {

                ordered_arcs_by_time_set.insert(Point(a, modified_arcs_time[a]));

            }


            vector<bool> pre_activated_arcs_2(tailleX);
            vector<vector<Point>> pre_activated_in_arcs_2(n);
            vector<vector<Point>> pre_activated_out_arcs_2(n);

            for (const Point& point : ordered_arcs_by_time_set) {


                if( Arcs[point.get_x()].get_x()!=0 && Arcs[point.get_x()].get_y()!=0 && (modified_arcs_time[point.get_x()] !=  min_int_arc)){

                    pre_activated_arcs_2[point.get_x()] = true;
                    pre_activated_in_arcs_2[Arcs[point.get_x()].get_y()].push_back(Point(point.get_x(), Arcs[point.get_x()].get_x()));
                    pre_activated_out_arcs_2[Arcs[point.get_x()].get_x()].push_back(Point(point.get_x(), Arcs[point.get_x()].get_y()));

                }
                else{

                    pre_activated_arcs_2[point.get_x()] = false;

                }

                activated_arcs[point.get_x()] = false;
            }



            for (int i = 0; i < n; ++i) {

                visiting_cities[i] =false;

            }

            completing_total_traveling(pre_activated_out_arcs_2, pre_activated_in_arcs_2, Arcs, pre_activated_arcs_2, d, C, visiting_cities, activated_arcs);

            for (int i = 1; i < n; ++i) {

                if(!visiting_cities[i]){

                    activated_arcs[in_depot_arcs[i]] = true;
                    activated_arcs[out_depot_arcs[i]] = true;

                }

            }


            int current_score = slave_problem_resolution(activated_arcs,  th, T, Arcs, Arcs_time);

            for (int a = 0; a < Arcs.size(); ++a) {
                //                current_solution[a] = activated_arcs[a];
                solutions_table[best_solution_index_array[var]][a] = activated_arcs[a];
            }


            solutions_table_score[best_solution_index_array[var]] = current_score;

            cout<<"the current_score within this meta_heuristic is " <<current_score<<endl;

            if(current_score < best_score){

                auto end_2 = chrono::high_resolution_clock::now();
                chrono::duration<double> duration_heuristic = end_2 - start_heuristic_time;
                elapsed = duration_heuristic.count();
                best_score = current_score;

                for (int a = 0; a < tailleX; ++a) {

                    best_current_solution[a] = activated_arcs[a];

                }

                vector<Point> edges;

                for (int a = 0; a < tailleX; ++a) {

                    if(best_current_solution[a]){

                        cout<<"use arc "<< Arcs[a].get_x()<< " "<< Arcs[a].get_y()<<endl;

                        edges.push_back(Arcs[a]);

                    }

                }

                cout << "The total score of this heuristic solution is " <<best_score<< " found at time "<<elapsed<<" in generation "<<population <<endl;

                gap = (best_score-best_bound)/(best_bound+ 1e-10);

                exportToCSV(edges, path_to_solution);
                exportTo_txt_with_total_information(edges, path_to_complete_solution, elapsed, best_score, 100*gap, best_bound);

            }
        }

        delete [] best_solution_index_array;
    }

    delete [] current_solution;
    delete [] solutions_table;
    delete [] solutions_table_score;
    delete [] tabou_arcs_solutions_table;
    delete [] tabou_number_banned_possibilities;
}
