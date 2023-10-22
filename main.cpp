#include <iostream>

#include <idol/modeling.h>
#include <idol/problems/knapsack-problem/KP_Instance.h>
#include <idol/optimizers/solvers/GLPK.h>
#include <idol/optimizers/branch-and-bound/BranchAndBound.h>

#include <idol/optimizers/branch-and-bound/node-selection-rules/factories/BestBound.h>
#include <idol/optimizers/branch-and-bound/node-selection-rules/factories/WorstBound.h>
#include <idol/optimizers/branch-and-bound/node-selection-rules/factories/DepthFirst.h>
#include <idol/optimizers/branch-and-bound/node-selection-rules/factories/BreadthFirst.h>

#include <idol/optimizers/branch-and-bound/branching-rules/factories/StrongBranching.h>
#include <idol/optimizers/branch-and-bound/branching-rules/factories/FirstInfeasibleFound.h>
#include <idol/optimizers/branch-and-bound/branching-rules/factories/LeastInfeasible.h>
#include <idol/optimizers/branch-and-bound/branching-rules/factories/MostInfeasible.h>
#include <idol/optimizers/branch-and-bound/branching-rules/factories/PseudoCost.h>
#include <idol/optimizers/branch-and-bound/branching-rules/factories/UniformlyRandom.h>

#include <idol/optimizers/callbacks/SimpleRounding.h>
#include <fstream>

using namespace idol;

Model create_kp_model(Env& t_env, const std::string& t_filename) {

    const auto instance = Problems::KP::read_instance_kplib(t_filename);
    const auto n_items = instance.n_items();

    Model result(t_env);

    auto x = result.add_vars(Dim<1>(n_items), 0, 1, Binary, "x");

    result.add_ctr(idol_Sum(j, Range(n_items), instance.weight(j) * x[j]) <= instance.capacity());

    result.set_obj_expr(idol_Sum(j, Range(n_items), -instance.profit(j) * x[j]));

    return result;

}

unsigned int get_n_solved_nodes(const Model& t_model) {

    const auto& optimizer = t_model.optimizer();

    if (!optimizer.is<Optimizers::BranchAndBound<NodeVarInfo>>()) {
        return 0;
    }

    const auto& branch_and_bound = optimizer.as<Optimizers::BranchAndBound<NodeVarInfo>>();

    return branch_and_bound.n_solved_nodes();

}

NodeSelectionRuleFactory<NodeVarInfo>* get_node_selection_rule(const std::string& t_arg) {

    if (t_arg == "best-bound") {
        return new BestBound::Strategy<NodeVarInfo>(BestBound());
    }

    if (t_arg == "worst-bound") {
        return new WorstBound::Strategy<NodeVarInfo>(WorstBound());
    }

    if (t_arg == "depth-first") {
        return new DepthFirst::Strategy<NodeVarInfo>(DepthFirst());
    }

    if (t_arg == "breadth-first") {
        return new BreadthFirst::Strategy<NodeVarInfo>(BreadthFirst());
    }

    throw Exception("Unknown node selection rule: " + t_arg);
}

BranchingRuleFactory<NodeVarInfo>* get_branching_rule(const std::string& t_arg) {

    if (t_arg == "most-infeasible") {
        return new MostInfeasible::Strategy<NodeVarInfo>();
    }

    if (t_arg == "least-infeasible") {
        return new LeastInfeasible::Strategy<NodeVarInfo>();
    }

    if (t_arg == "first-infeasible") {
        return new FirstInfeasibleFound::Strategy<NodeVarInfo>();
    }

    if (t_arg == "uniformly-random") {
        return new UniformlyRandom::Strategy<NodeVarInfo>();
    }

    if (t_arg == "strong-branching") {
        return new StrongBranching::Strategy<NodeVarInfo>();
    }

    if (t_arg == "pseudo-cost") {
        return new PseudoCost::Strategy<NodeVarInfo>();
    }

    throw Exception("Unknown branching rule rule: " + t_arg);

}

CallbackFactory* get_heuristic(const std::string& t_arg) {

    if (t_arg == "-") {
        return nullptr;
    }

    if (t_arg == "simple-rounding") {
        return new Heuristics::SimpleRounding();
    }

    throw Exception("Unknown heuristic: " + t_arg);
}

int main(int t_argc, const char** t_argv) {

    if (t_argc < 2) {
        throw Exception("Arguments: path_to_instance solver [node-selection-rule] [branching-rule] [heuristic]");
    }

    const std::string path_to_instance = t_argv[1];
    const std::string solver = t_argv[2];
    std::string str_node_selection_rule = "-";
    std::string str_branching_rule = "-";
    std::string str_heuristic = "-";

    Env env;
    auto model = create_kp_model(env, path_to_instance);

    if (solver == "external") {

        model.use(GLPK());

    } else if (solver == "bab") {

        if (t_argc != 6) {
            throw Exception("Arguments node-selection-rule, branching-rule and heuristic are mandatory when solver is idol");
        }

        str_node_selection_rule = t_argv[3];
        str_branching_rule = t_argv[4];
        str_heuristic = t_argv[5];

        std::unique_ptr<NodeSelectionRuleFactory<NodeVarInfo>> node_selection_rule(get_node_selection_rule(str_node_selection_rule));
        std::unique_ptr<BranchingRuleFactory<NodeVarInfo>> branching_rule(get_branching_rule(str_branching_rule));
        std::unique_ptr<CallbackFactory> heuristic(get_heuristic(str_heuristic));

        model.use(
            BranchAndBound()
                .with_node_optimizer(GLPK::ContinuousRelaxation())
                .with_node_selection_rule(*node_selection_rule)
                .with_branching_rule(*branching_rule)
                .conditional((bool) heuristic, [&](auto& x) {
                    x.with_callback(*heuristic);
                })
                .with_log_level(Info, Blue)
                .with_log_frequency(1)
        );

    } else {
        throw Exception("Unknown solver: " + solver);
    }

    model.optimizer().set_param_time_limit(5 * 60);

    model.optimize();

    std::ofstream file("results_KP_idol.csv", std::ios::out | std::ios::app);

    if (!file.is_open()) {
        throw std::runtime_error("Could not open error destination file.");
    }

    file << "result,"
         << path_to_instance << ","
         << solver << ","
         << str_node_selection_rule << ","
         << str_branching_rule << ","
         << str_heuristic << ","
         << model.optimizer().time().count() << ","
         << get_n_solved_nodes(model) << ","
         << model.get_best_bound() << ","
         << model.get_best_obj()
         << std::endl;

    file.close();

    return 0;
}
