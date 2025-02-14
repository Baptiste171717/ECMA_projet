#include "ECMA.h"



using namespace std;


void resolution_by_dualisation(string path_to_complete_solution, string path_to_solution, IloEnv& env, int n, int C, int T, vector<int> th, vector<vector<int>> t, vector<int> d,vector<Point> Arcs,
                                vector<int> Arcs_time, vector<vector<int>> in_arcs, vector<vector<int>> out_arcs, const int tailleX, bool* best_dualisation_current_solution, double& dualisation_best_score, float& dualisation_elapsed,
                               float& gap, float& best_bound, const int time_solver){

    try {

        auto start_dualisation_time = chrono::high_resolution_clock::now();

        IloBoolVarArray x(env, tailleX);
        IloNumVarArray y_T(env, 2, 0.0, IloInfinity);
        IloNumVarArray y_1(env, tailleX, 0.0, IloInfinity);
        IloNumVarArray y_2(env, tailleX, 0.0, IloInfinity);
        IloIntVarArray u(env, n-1, 0, IloIntMax);


        IloRangeArray constraints_1(env);
        IloRangeArray constraints_2(env);
        IloRangeArray constraints_3(env);
        IloRangeArray constraints_4(env);
        IloRangeArray constraints_5(env);
        IloRangeArray constraints_6(env);
        IloRangeArray constraints_7(env);
        IloRangeArray constraints_8(env);


        IloModel model(env);

        IloExpr expression(env);


        for (int a = 0; a < tailleX; ++a) {
            expression += y_1[a] + 2*y_2[a] + Arcs_time[a]*x[a];
        }

        expression += T*y_T[0] + T*T*y_T[1];

        IloObjective obj(env, expression, IloObjective::Minimize);
        model.add(obj);
        expression.end();

        IloExpr expr(env);


        for (int a = 0; a < tailleX; ++a) {
            expr += (th[Arcs[a].get_x()] + th[Arcs[a].get_y()])*x[a] - y_T[0] - y_1[a];
            constraints_1.add(expr <= 0);
            expr.clear();
        }
        model.add(constraints_1);


        for (int a = 0; a < tailleX; ++a) {
            expr += (th[Arcs[a].get_x()]*th[Arcs[a].get_y()])*x[a] - y_T[1] - y_2[a];
            constraints_2.add(expr <= 0);
            expr.clear();
        }
        model.add(constraints_2);


        for (int i = 1; i < n; ++i) {
            for (int a = 0; a < in_arcs[i].size(); ++a) {

                expr+= x[in_arcs[i][a]];

            }
            constraints_3.add(expr == 1);
            expr.clear();
        }

        model.add(constraints_3);


        for (int i = 1; i < n; ++i) {
            for (int a = 0; a < out_arcs[i].size(); ++a) {
                expr+= x[out_arcs[i][a]];
            }
            constraints_4.add(expr == 1);
            expr.clear();
        }

        model.add(constraints_4);


        for (int a = 0; a < out_arcs[0].size(); ++a) {
            expr+= x[out_arcs[0][a]];
        }
        for (int a = 0; a < in_arcs[0].size(); ++a) {
            expr-= x[in_arcs[0][a]];
        }
        constraints_5.add(expr ==0);
        expr.clear();
        model.add(constraints_5);


        for (int i = 0; i < n-1; ++i) {
            expr += u[i]- C + d[i+1];
            constraints_6.add(expr <= 0);
            expr.clear();
        }
        model.add(constraints_6);


        for (int a = 0; a < tailleX; ++a) {
            if(Arcs[a].get_x() !=0 && Arcs[a].get_y()!= 0){
                expr+= d[Arcs[a].get_x()] - C*(1-x[a]) - u[Arcs[a].get_y()-1] + u[Arcs[a].get_x()-1];
                constraints_7.add(expr<=0);
                expr.clear();
            }
        }
        model.add(constraints_7);


        for (int a = 0; a < out_arcs[0].size(); ++a) {
            expr += u[Arcs[a].get_y()-1] - C*(1 - x[a]);
            constraints_8.add(expr <= 0);
            expr.clear();
        }
        model.add(constraints_8);

        IloCplex cplex(model);

        // Masque les informations fournies par Cplex durant la resolution
//        cplex.setOut(env.getNullStream());


        cplex.setParam(IloCplex::TiLim, time_solver);

        cplex.solve();

//        gap = cplex.getMIPRelativeGap();

        best_bound = cplex.getBestObjValue();




        if (cplex.getStatus() == IloAlgorithm::Infeasible)
            cout << "No feasible solution found within the time limit." << endl;

        // Si le resultat est faisable
        else{

            auto end = chrono::high_resolution_clock::now();
            chrono::duration<double> duration_heuristic = end - start_dualisation_time;
            dualisation_elapsed = duration_heuristic.count();

            // Afficher la valeur de l'objectif
            cout << "objective: " << cplex.getObjValue() << endl;

            dualisation_best_score = cplex.getObjValue();

            gap = (dualisation_best_score-best_bound)/(best_bound+ 1e-10);

            // Afficher la valeur des variables
            IloNumArray xSolution(env);
            cplex.getValues(x, xSolution);

            for (int a = 0; a < Arcs.size(); ++a) {
                best_dualisation_current_solution[a] = (xSolution[a] >0.99);
            }

            vector<Point> edges;

            for (int a = 0; a < tailleX; ++a) {

                if(best_dualisation_current_solution[a]){

                    cout<<"use arc "<< Arcs[a].get_x()<< " "<< Arcs[a].get_y()<<endl;

                    edges.push_back(Arcs[a]);

                }

            }

            cout << "The total score of this dualisation solution is " <<dualisation_best_score<< " found at time "<<dualisation_elapsed <<endl;

            exportToCSV(edges, path_to_solution);
            exportTo_txt_with_total_information(edges, path_to_complete_solution, dualisation_elapsed, dualisation_best_score, gap*100, best_bound);

        }

        // Si une erreur survient
        y_1.end();
        y_2.end();
        y_T.end();
        u.end();
        constraints_1.end();
        constraints_2.end();
        constraints_3.end();
        constraints_4.end();
        constraints_5.end();
        constraints_6.end();
        constraints_7.end();
        constraints_8.end();
        x.end();
        model.end();
    } catch (const IloException& e){

        cerr << e;
        throw;
    }

}


void static_resolution(string path_to_complete_solution, string path_to_solution, IloEnv& env, int n, int C, int T, vector<int> th, vector<vector<int>> t,
                       vector<int> d, vector<Point> Arcs, vector<int> Arcs_time, vector<vector<int>> in_arcs, vector<vector<int>> out_arcs, const int tailleX,
                       bool* best_static_current_solution, double& static_best_score, float& static_elapsed, float& PR_gap, double& dualisation_best_score, const int time_solver){

    try {

        auto start_static_time = chrono::high_resolution_clock::now();
        // Define the master problem
        IloModel static_model(env);
        IloCplex static_cplex(static_model);

        // Define variables

        IloBoolVarArray x(env, tailleX);
        IloIntVar z(env, 0, IloIntMax);
        IloIntVarArray u(env, n-1, 0, IloIntMax);

        IloRangeArray constraints_initial_cut(env);
        IloRangeArray constraints_1(env);
        IloRangeArray constraints_2(env);
        IloRangeArray constraints_3(env);
        IloRangeArray constraints_4(env);
        IloRangeArray constraints_5(env);
        IloRangeArray constraints_6(env);


        IloObjective obj(env, z, IloObjective::Minimize);
        static_model.add(obj);

        IloExpr expr(env);

        for (int a = 0; a < tailleX; ++a) {
            expr += Arcs_time[a]*x[a];
        }

        expr -= z;
        constraints_initial_cut.add(expr <= 0);
        static_model.add(constraints_initial_cut);

        expr.clear();

        for (int i = 1; i < n; ++i) {

            for (int a = 0; a < in_arcs[i].size(); ++a) {

                expr+= x[in_arcs[i][a]];

            }

            constraints_1.add(expr == 1);
            expr.clear();

        }

        static_model.add(constraints_1);


        for (int i = 1; i < n; ++i) {

            for (int a = 0; a < out_arcs[i].size(); ++a) {

                expr+= x[out_arcs[i][a]];

            }

            constraints_2.add(expr == 1);
            expr.clear();

        }

        static_model.add(constraints_2);


        for (int a = 0; a < out_arcs[0].size(); ++a) {
            expr+= x[out_arcs[0][a]];
        }
        for (int a = 0; a < in_arcs[0].size(); ++a) {
            expr-= x[in_arcs[0][a]];
        }
        constraints_3.add(expr ==0);
        expr.clear();
        static_model.add(constraints_3);


        for (int i = 0; i < n-1; ++i) {
            expr += u[i]- C + d[i+1];
            constraints_4.add(expr <= 0);
            expr.clear();
        }
        static_model.add(constraints_4);


        for (int a = 0; a < tailleX; ++a) {
            if(Arcs[a].get_x() !=0 && Arcs[a].get_y()!= 0){
                expr+= d[Arcs[a].get_x()] - C*(1-x[a]) - u[Arcs[a].get_y()-1] + u[Arcs[a].get_x()-1];
                constraints_5.add(expr<=0);
                expr.clear();
            }
        }
        static_model.add(constraints_5);


        for (int a = 0; a < out_arcs[0].size(); ++a) {
            expr += u[Arcs[a].get_y()-1] - C*(1 - x[a]);
            constraints_6.add(expr <= 0);
            expr.clear();
        }

        static_model.add(constraints_6);
        // Solve the master problem
        bool optimal = false;

        float* delta_1_solution = new float[tailleX];
        float* delta_2_solution = new float[tailleX];

        static_cplex.setParam(IloCplex::TiLim, time_solver);

//        int best_test_score = 9999999;

        int current_test_score =0;

        IloNumArray xSolution(env);
        static_cplex.solve();
        static_cplex.getValues(x, xSolution);
        static_best_score = static_cplex.getObjValue();


        auto end = chrono::high_resolution_clock::now();
        chrono::duration<double> duration_static = end - start_static_time;
        static_elapsed = duration_static.count();

        IloAlgorithm::Status status = static_cplex.getStatus();

        bool have_a_solution = false;

        if (status == IloAlgorithm::Optimal) {
            have_a_solution =true;
            cout << "Optimal solution found." << endl;
        } else if (status == IloAlgorithm::Feasible) {
            have_a_solution =true;
            cout << "Feasible solution found within the time limit." << endl;
        }
        else if (status == IloAlgorithm::Infeasible) {
            std::cout << "No feasible solution exists." << std::endl;
        }


        if(have_a_solution){

            for (int a = 0; a < Arcs.size(); ++a) {

                best_static_current_solution[a] = (xSolution[a] >0.99999);

            }

            vector<Point> selected_arcs;

            for (int a = 0; a < tailleX; ++a) {

                if(best_static_current_solution[a]){

                    cout<<"use arc "<< Arcs[a].get_x()<< " "<< Arcs[a].get_y()<<endl;

                    selected_arcs.push_back(Arcs[a]);

                }

            }

            PR_gap = (dualisation_best_score-static_best_score)/(static_best_score+ 1e-10);

            cout << "The total score of this static solution is " <<static_best_score<< " found at time "<<static_elapsed <<endl;

            exportToCSV(selected_arcs, path_to_solution);
            exportTo_txt_with_total_information(selected_arcs, path_to_complete_solution, static_elapsed, static_best_score, PR_gap*100, best_bound);

        }


        delete [] delta_1_solution;
        delete [] delta_2_solution;

        u.end();
        constraints_1.end();
        constraints_2.end();
        constraints_3.end();
        constraints_4.end();
        constraints_5.end();
        constraints_6.end();
        x.end();
        z.end();
        static_model.end();

    } catch (IloException& e) {
        cerr << "Error: " << e.getMessage() << endl;
    } catch (...) {
        cerr << "Unknown error" << endl;
    }

}



void MyLazyCallback::main() {



    IloEnv env = getEnv();
    IloEnv env_1;

    try {

        IloNumArray x(env_1);
        double z_value = getValue(z);
        getValues(x, vars);


        IloModel slaveModel(env_1);

        IloNumVarArray delta_1(env_1, tailleX, 0.0, 1);
        IloNumVarArray delta_2(env_1, tailleX, 0.0, 2);

        IloRangeArray constraints_1(env_1);
        IloRangeArray constraints_2(env_1);

        IloExpr expression(env_1);

        for (int a = 0; a < tailleX; ++a) {
            expression += (Arcs_time[a] + delta_1[a]*( th[Arcs[a].get_x()] + th[Arcs[a].get_y()]) + delta_2[a]*th[Arcs[a].get_x()]*th[Arcs[a].get_y()])*x[a] ;
        }

        IloObjective obj(env_1, expression, IloObjective::Maximize);
        slaveModel.add(obj);
        expression.end();

        IloExpr expr(env_1);

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
        if (!slaveCplex.solve()) {
            throw std::runtime_error("Slave problem could not be solved.");
        }


        double slaveObjectiveValue = slaveCplex.getObjValue();

        if(slaveObjectiveValue < lazy_callback_best_score){

            auto end = chrono::high_resolution_clock::now();
            chrono::duration<double> duration_heuristic = end - startTime;
            lazy_callback_elapsed = duration_heuristic.count();
            lazy_callback_best_score = slaveObjectiveValue;
            gap = (lazy_callback_best_score-best_bound)/(best_bound+ 1e-10);

            for (int a = 0; a < Arcs.size(); ++a) {
                best_lazy_callback_current_solution[a] = (x[a] >=0.999);
            }

            vector<Point> selected_arcs;

            for (int a = 0; a < tailleX; ++a) {

                if(best_lazy_callback_current_solution[a]){

                    cout<<"use arc "<< Arcs[a].get_x()<< " "<< Arcs[a].get_y()<<endl;

                    selected_arcs.push_back(Arcs[a]);

                }

            }

            cout << "The total score of this lazy_call_back solution  is " <<lazy_callback_best_score<< " found at time "<<lazy_callback_elapsed <<endl;

            exportToCSV(selected_arcs, path_to_solution);
            exportTo_txt_with_total_information(selected_arcs, path_to_complete_solution, lazy_callback_elapsed, lazy_callback_best_score, gap*100, best_bound);

        }

        cout<<"the z_value is "<<z_value<<", the slave_value is "<<slaveObjectiveValue<<endl;

        if (slaveObjectiveValue > z_value) {

            cout<<slaveObjectiveValue<<" "<<z_value<<endl;

            IloNumArray delta_1_Solution(env_1);
            slaveCplex.getValues(delta_1, delta_1_Solution);
            IloNumArray delta_2_Solution(env_1);
            slaveCplex.getValues(delta_2, delta_2_Solution);


            IloExpr cutExpr(env);
            // Define the cut expression
            for (int a = 0; a < tailleX; ++a) {
                cutExpr += (Arcs_time[a] + delta_1_Solution[a]*( th[Arcs[a].get_x()] + th[Arcs[a].get_y()]) + delta_2_Solution[a]*th[Arcs[a].get_x()]*th[Arcs[a].get_y()])*vars[a] ;
            }
            cutExpr -= z ;

            add(IloRange(env, -IloInfinity, cutExpr, 0));
            cutExpr.clear();

            if(slaveObjectiveValue == lazy_callback_best_score){
                cutExpr = z - slaveObjectiveValue;

                add(IloRange(env, -IloInfinity, cutExpr, 0));

                cout<< "add new valid cut for slave objectif"<<endl;
            }


            delta_1_Solution.end();
            delta_2_Solution.end();
            cutExpr.end();
        }

        x.end();
        constraints_1.end();
        constraints_2.end();
        delta_1.end();
        delta_2.end();
        slaveModel.end();
        env_1.end();

    } catch (IloException& e) {
        env.error() << "Exception caught: " << e << std::endl;
    }

}



void resolution_by_lazy_callback(string path_to_complete_solution, string path_to_solution, IloEnv& env, int n, int C, int T, vector<int> th, vector<vector<int>> t, vector<int> d, vector<Point> Arcs,
                                  vector<int> Arcs_time, vector<vector<int>> in_arcs, vector<vector<int>> out_arcs, const int tailleX, bool* best_lazy_callback_current_solution,
                                 double& lazy_callback_best_score, float& lazy_callback_elapsed, float& gap, float best_bound, const int time_solver){

    try {

        auto start_lazy_callback_time = chrono::high_resolution_clock::now();

        IloBoolVarArray x(env, tailleX);
        IloIntVar z(env, 0, IloIntMax);
        IloIntVarArray u(env, n-1, 0, IloIntMax);

        IloRangeArray constraints_initial_cut(env);
        IloRangeArray constraints_1(env);
        IloRangeArray constraints_2(env);
        IloRangeArray constraints_3(env);
        IloRangeArray constraints_4(env);
        IloRangeArray constraints_5(env);
        IloRangeArray constraints_6(env);

        IloModel model(env);

        IloObjective obj(env, z, IloObjective::Minimize);
        model.add(obj);

        IloExpr expr(env);

        for (int a = 0; a < tailleX; ++a) {
            expr += Arcs_time[a]*x[a];
        }

        expr -= z;
        constraints_initial_cut.add(expr <= 0);
        model.add(constraints_initial_cut);

        expr.clear();

        for (int i = 1; i < n; ++i) {

            for (int a = 0; a < in_arcs[i].size(); ++a) {

                expr+= x[in_arcs[i][a]];

            }

            constraints_1.add(expr == 1);
            expr.clear();

        }

        model.add(constraints_1);


        for (int i = 1; i < n; ++i) {

            for (int a = 0; a < out_arcs[i].size(); ++a) {

                expr+= x[out_arcs[i][a]];

            }

            constraints_2.add(expr == 1);
            expr.clear();

        }

        model.add(constraints_2);


        for (int a = 0; a < out_arcs[0].size(); ++a) {
            expr+= x[out_arcs[0][a]];
        }
        for (int a = 0; a < in_arcs[0].size(); ++a) {
            expr-= x[in_arcs[0][a]];
        }
        constraints_3.add(expr ==0);
        expr.clear();
        model.add(constraints_3);


        for (int i = 0; i < n-1; ++i) {
            expr += u[i]- C + d[i+1];
            constraints_4.add(expr <= 0);
            expr.clear();
        }
        model.add(constraints_4);


        for (int a = 0; a < tailleX; ++a) {
            if(Arcs[a].get_x() !=0 && Arcs[a].get_y()!= 0){
                expr+= d[Arcs[a].get_x()] - C*(1-x[a]) - u[Arcs[a].get_y()-1] + u[Arcs[a].get_x()-1];
                constraints_5.add(expr<=0);
                expr.clear();
            }
        }
        model.add(constraints_5);


        for (int a = 0; a < out_arcs[0].size(); ++a) {
            expr += u[Arcs[a].get_y()-1] - C*(1 - x[a]);
            constraints_6.add(expr <= 0);
            expr.clear();
        }
        model.add(constraints_6);


        IloCplex cplex(model);


//        cplex.setOut(env.getNullStream());


        cplex.setParam(IloCplex::TiLim, time_solver);
        cplex.use(new (env) MyLazyCallback(env, z, x, th, tailleX, Arcs, T, Arcs_time, path_to_complete_solution, path_to_solution, best_lazy_callback_current_solution, lazy_callback_best_score,
                    lazy_callback_elapsed, gap, best_bound, start_lazy_callback_time));



        if (cplex.solve()) {

            std::cout << "Solution status: " << cplex.getStatus() << std::endl;
            std::cout << "Objective value: " << cplex.getObjValue() << std::endl;

//            auto end = chrono::high_resolution_clock::now();
//            chrono::duration<double> duration_heuristic = end - start_lazy_callback_time;
//            lazy_callback_elapsed = duration_heuristic.count();

//            lazy_callback_best_score = cplex.getObjValue();

//            gap = (lazy_callback_best_score-best_bound)/(best_bound+ 1e-10);

//            IloNumArray xSolution(env);
//            cplex.getValues(x, xSolution);

//            for (int a = 0; a < Arcs.size(); ++a) {
//                best_lazy_callback_current_solution[a] = (xSolution[a] >=0.999);
//            }

//            vector<Point> selected_arcs;

//            for (int a = 0; a < tailleX; ++a) {

//                if(best_lazy_callback_current_solution[a]){

//                cout<<"use arc "<< Arcs[a].get_x()<< " "<< Arcs[a].get_y()<<endl;

//                selected_arcs.push_back(Arcs[a]);

//                }

//            }

            cout << "The total score of this lazy_call_back solution  is " <<lazy_callback_best_score<< " found at time "<<lazy_callback_elapsed <<endl;

//            exportToCSV(selected_arcs, path_to_solution);
//            exportTo_txt_with_total_information(selected_arcs, path_to_complete_solution, lazy_callback_elapsed, lazy_callback_best_score, gap*100, best_bound);

        }
        else {
            std::cerr << "Failed to solve the problem." << std::endl;
        }

        // Si une erreur survient

        u.end();
        constraints_1.end();
        constraints_2.end();
        constraints_3.end();
        constraints_4.end();
        constraints_5.end();
        constraints_6.end();
        x.end();
        z.end();
        model.end();

    } catch (const IloException& e){

        // Afficher l'erreur
        cerr << e;
        throw;
    }
}


int cutting_planes_slave_problem_resolution(const IloNumArray& xValues, vector<int> th, const int T, vector<Point> Arcs, vector<int> Arcs_time, float* delta_1_solution, float* delta_2_solution,
                                            int* arcs_coordinates_table, const int n){

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

            expression += (Arcs_time[a] + delta_1[a]*( th[Arcs[a].get_x()] + th[Arcs[a].get_y()]) + delta_2[a]*th[Arcs[a].get_x()]*th[Arcs[a].get_y()])*xValues[a] ;

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
        else{
            IloNumArray delta_slave_solution_1(env);
            IloNumArray delta_slave_solution_2(env);
            slaveCplex.getValues(delta_1, delta_slave_solution_1);
            slaveCplex.getValues(delta_2, delta_slave_solution_2);

            for (int a = 0; a < tailleX; ++a) {

                if(xValues[a] ){

                    delta_1_solution[a] = delta_slave_solution_1[a];
                    delta_2_solution[a] = delta_slave_solution_2[a];

                }
                else{ //here we actually have the problem of the simple loops, so we can actually add time constraints that are not contain by the uncertainty set and can lead to a higher value of the optimum

                    if(arcs_coordinates_table[n*Arcs[a].get_y() + Arcs[a].get_x()]!=-1){
                        delta_1_solution[a] = delta_slave_solution_1[arcs_coordinates_table[n*Arcs[a].get_y() + Arcs[a].get_x()]];
                        delta_2_solution[a] = delta_slave_solution_2[arcs_coordinates_table[n*Arcs[a].get_y() + Arcs[a].get_x()]];
                    }
                }

            }

            delta_slave_solution_1.end();
            delta_slave_solution_2.end();
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

bool checkSlaveProblem(const IloNumArray& xValues, double zValue, vector<int> th, vector<Point> Arcs, vector<int> Arcs_time, int T, float* delta_1_Solution, float* delta_2_solution, int& test_score,
                        int* arcs_coordinates_table, const int n) {

    test_score = cutting_planes_slave_problem_resolution(xValues, th, T, Arcs, Arcs_time, delta_1_Solution, delta_2_solution, arcs_coordinates_table, n);

    cout<< "the value of the slave problem is "<< test_score<< " and the z_value is "<< zValue<<endl;

    return (zValue >= test_score);

}

IloRange generateCut(IloEnv env, const IloBoolVarArray& x, const IloIntVar& z, vector<Point> Arcs,  vector<int> Arcs_time, vector<int> th, float* delta_1_Solution, float* delta_2_Solution) {

    int tailleX = Arcs.size();
    IloExpr cutExpr(env);
    // Define the cut expression
    for (int a = 0; a < tailleX; ++a) {
        cutExpr += (Arcs_time[a] + delta_1_Solution[a]*( th[Arcs[a].get_x()] + th[Arcs[a].get_y()]) + delta_2_Solution[a]*th[Arcs[a].get_x()]*th[Arcs[a].get_y()])*x[a] ;
    }
    cutExpr -= z ;

    return IloRange(env, -IloInfinity, cutExpr, 0); // Placeholder
}

void resolution_by_cutting_planes( string path_to_complete_solution, string path_to_solution, IloEnv& env, int n, int C, int T, vector<int> th, vector<vector<int>> t, vector<int> d, vector<Point> Arcs,
                               vector<int> Arcs_time, vector<vector<int>> in_arcs, vector<vector<int>> out_arcs, const int tailleX, bool* best_cutting_planes_current_solution,
                                       double& cutting_planes_best_score, float& cutting_planes_elapsed, float& gap, float& best_bound, const int time_solver, chrono::milliseconds maxDuration, int* arcs_coordinates_table){

    try {

        auto start_cutting_planes_time = chrono::high_resolution_clock::now();
        // Define the master problem
        IloModel master_model(env);
        IloCplex master_cplex(master_model);

        // Define variables

        IloBoolVarArray x(env, tailleX);
        IloIntVar z(env, 0, IloIntMax);
        IloIntVarArray u(env, n-1, 0, IloIntMax);

        IloRangeArray constraints_initial_cut(env);
        IloRangeArray constraints_1(env);
        IloRangeArray constraints_2(env);
        IloRangeArray constraints_3(env);
        IloRangeArray constraints_4(env);
        IloRangeArray constraints_5(env);
        IloRangeArray constraints_6(env);


        IloObjective obj(env, z, IloObjective::Minimize);
        master_model.add(obj);

        IloExpr expr(env);

        for (int a = 0; a < tailleX; ++a) {
            expr += Arcs_time[a]*x[a];
        }

        expr -= z;
        constraints_initial_cut.add(expr <= 0);
        master_model.add(constraints_initial_cut);

        expr.clear();

        for (int i = 1; i < n; ++i) {

            for (int a = 0; a < in_arcs[i].size(); ++a) {

                expr+= x[in_arcs[i][a]];

            }

            constraints_1.add(expr == 1);
            expr.clear();

        }

        master_model.add(constraints_1);


        for (int i = 1; i < n; ++i) {

            for (int a = 0; a < out_arcs[i].size(); ++a) {

                expr+= x[out_arcs[i][a]];

            }

            constraints_2.add(expr == 1);
            expr.clear();

        }

        master_model.add(constraints_2);


        for (int a = 0; a < out_arcs[0].size(); ++a) {
            expr+= x[out_arcs[0][a]];
        }
        for (int a = 0; a < in_arcs[0].size(); ++a) {
            expr-= x[in_arcs[0][a]];
        }
        constraints_3.add(expr ==0);
        expr.clear();
        master_model.add(constraints_3);


        for (int i = 0; i < n-1; ++i) {
            expr += u[i]- C + d[i+1];
            constraints_4.add(expr <= 0);
            expr.clear();
        }
        master_model.add(constraints_4);


        for (int a = 0; a < tailleX; ++a) {
            if(Arcs[a].get_x() !=0 && Arcs[a].get_y()!= 0){
                expr+= d[Arcs[a].get_x()] - C*(1-x[a]) - u[Arcs[a].get_y()-1] + u[Arcs[a].get_x()-1];
                constraints_5.add(expr<=0);
                expr.clear();
            }
        }
        master_model.add(constraints_5);


        for (int a = 0; a < out_arcs[0].size(); ++a) {
            expr += u[Arcs[a].get_y()-1] - C*(1 - x[a]);
            constraints_6.add(expr <= 0);
            expr.clear();
        }

        master_model.add(constraints_6);
        // Solve the master problem
        bool optimal = false;

        float* delta_1_solution = new float[tailleX];
        float* delta_2_solution = new float[tailleX];

        master_cplex.setParam(IloCplex::TiLim, time_solver);
        master_cplex.setOut(env.getNullStream());

        int best_test_score = 9999999;

        int current_test_score =0;

        IloNumArray xSolution(env);

        while (!optimal) {

            master_cplex.solve();

            IloNumArray xValues(env);
            master_cplex.getValues(xValues, x);
            double zValue = master_cplex.getValue(z);

            // Check the solution against the slave problem

            bool satisfiesConstraints = checkSlaveProblem(xValues, zValue, th, Arcs, Arcs_time, T, delta_1_solution, delta_2_solution, current_test_score, arcs_coordinates_table, n);

            if(current_test_score < best_test_score && (master_cplex.getStatus() == IloAlgorithm::Feasible ||master_cplex.getStatus() == IloAlgorithm::Optimal)){
                best_test_score = current_test_score;
                master_cplex.getValues(x, xSolution);
                auto end = chrono::high_resolution_clock::now();
                chrono::duration<double> duration_cutting_planes = end - start_cutting_planes_time;
                cutting_planes_elapsed = duration_cutting_planes.count();
            }

            if (satisfiesConstraints || (chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start_cutting_planes_time) > maxDuration)) {

                optimal = true;

            }

            else {

                // Add cuts to the master problem
                IloRange newCut = generateCut(env, x, z, Arcs, Arcs_time, th, delta_1_solution, delta_2_solution);
                master_model.add(newCut);

            }
            xValues.end();
        }

//        auto end = chrono::high_resolution_clock::now();
//        chrono::duration<double> duration_cutting_planes = end - start_cutting_planes_time;
//        cutting_planes_elapsed = duration_cutting_planes.count();

        IloAlgorithm::Status status = master_cplex.getStatus();

        bool have_a_solution = false;

        if (status == IloAlgorithm::Optimal) {
            have_a_solution =true;
            cout << "Optimal solution found." << endl;
            cutting_planes_best_score = cutting_planes_slave_problem_resolution(xSolution, th, T, Arcs, Arcs_time, delta_1_solution, delta_2_solution, arcs_coordinates_table, n);
        } else if (status == IloAlgorithm::Feasible) {
            have_a_solution =true;
            cout << "Feasible solution found within the time limit." << endl;
            cutting_planes_best_score = cutting_planes_slave_problem_resolution(xSolution, th, T, Arcs, Arcs_time, delta_1_solution, delta_2_solution, arcs_coordinates_table, n);
        }
         else if (status == IloAlgorithm::Infeasible) {
            std::cout << "No feasible solution exists." << std::endl;
        }


        if(have_a_solution){

            for (int a = 0; a < Arcs.size(); ++a) {

                best_cutting_planes_current_solution[a] = (xSolution[a] >0.99999);

            }

            vector<Point> selected_arcs;

            for (int a = 0; a < tailleX; ++a) {

                if(best_cutting_planes_current_solution[a]){

                    cout<<"use arc "<< Arcs[a].get_x()<< " "<< Arcs[a].get_y()<<endl;

                    selected_arcs.push_back(Arcs[a]);

                }

            }

            gap = (cutting_planes_best_score-best_bound)/(best_bound+ 1e-10);

            cout << "The total score of this cutting_planes solution is " <<cutting_planes_best_score<< " found at time "<<cutting_planes_elapsed <<endl;

            exportToCSV(selected_arcs, path_to_solution);
            exportTo_txt_with_total_information(selected_arcs, path_to_complete_solution, cutting_planes_elapsed, cutting_planes_best_score, gap*100, best_bound);

        }


        delete [] delta_1_solution;
        delete [] delta_2_solution;
        u.end();
        constraints_1.end();
        constraints_2.end();
        constraints_3.end();
        constraints_4.end();
        constraints_5.end();
        constraints_6.end();
        x.end();
        z.end();
        master_model.end();

    } catch (IloException& e) {
        cerr << "Error: " << e.getMessage() << endl;
    } catch (...) {
        cerr << "Unknown error" << endl;
    }

}


