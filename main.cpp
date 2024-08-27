#include <iostream>

#include <idol/modeling.h>
#include <idol/problems/knapsack-problem/KP_Instance.h>
#include <idol/optimizers/mixed-integer-optimization/wrappers/GLPK/GLPK.h>
#include <idol/optimizers/mixed-integer-optimization/branch-and-bound/BranchAndBound.h>

#include <idol/optimizers/mixed-integer-optimization/branch-and-bound/node-selection-rules/factories/BestBound.h>
#include <idol/optimizers/mixed-integer-optimization/branch-and-bound/node-selection-rules/factories/WorstBound.h>
#include <idol/optimizers/mixed-integer-optimization/branch-and-bound/node-selection-rules/factories/DepthFirst.h>
#include <idol/optimizers/mixed-integer-optimization/branch-and-bound/node-selection-rules/factories/BreadthFirst.h>
#include <idol/optimizers/mixed-integer-optimization/branch-and-bound/node-selection-rules/factories/BestEstimate.h>

#include <idol/optimizers/mixed-integer-optimization/branch-and-bound/branching-rules/factories/StrongBranching.h>
#include <idol/optimizers/mixed-integer-optimization/branch-and-bound/branching-rules/factories/FirstInfeasibleFound.h>
#include <idol/optimizers/mixed-integer-optimization/branch-and-bound/branching-rules/factories/LeastInfeasible.h>
#include <idol/optimizers/mixed-integer-optimization/branch-and-bound/branching-rules/factories/MostInfeasible.h>
#include <idol/optimizers/mixed-integer-optimization/branch-and-bound/branching-rules/factories/PseudoCost.h>
#include <idol/optimizers/mixed-integer-optimization/branch-and-bound/branching-rules/factories/UniformlyRandom.h>

#include <idol/optimizers/mixed-integer-optimization/callbacks/heuristics/SimpleRounding.h>
#include <idol/optimizers/mixed-integer-optimization/callbacks/cutting-planes/KnapsackCover.h>
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

    if (!optimizer.is<Optimizers::BranchAndBound<DefaultNodeInfo>>()) {
        return 0;
    }

    const auto& branch_and_bound = optimizer.as<Optimizers::BranchAndBound<DefaultNodeInfo>>();

    return branch_and_bound.n_solved_nodes();

}

NodeSelectionRuleFactory<DefaultNodeInfo>* get_node_selection_rule(const std::string& t_arg) {

    if (t_arg == "best-bound") {
        return new BestBound::Strategy<DefaultNodeInfo>(BestBound());
    }

    if (t_arg == "worst-bound") {
        return new WorstBound::Strategy<DefaultNodeInfo>(WorstBound());
    }

    if (t_arg == "depth-first") {
        return new DepthFirst::Strategy<DefaultNodeInfo>(DepthFirst());
    }

    if (t_arg == "breadth-first") {
        return new BreadthFirst::Strategy<DefaultNodeInfo>(BreadthFirst());
    }

    if (t_arg == "best-estimate") {
        return new BestEstimate::Strategy<DefaultNodeInfo>(BestEstimate());
    }

    throw Exception("Unknown node selection rule: " + t_arg);
}

BranchingRuleFactory<DefaultNodeInfo>* get_branching_rule(const std::string& t_arg) {

    if (t_arg == "most-infeasible") {
        return new MostInfeasible::Strategy<DefaultNodeInfo>();
    }

    if (t_arg == "least-infeasible") {
        return new LeastInfeasible::Strategy<DefaultNodeInfo>();
    }

    if (t_arg == "first-infeasible") {
        return new FirstInfeasibleFound::Strategy<DefaultNodeInfo>();
    }

    if (t_arg == "uniformly-random") {
        return new UniformlyRandom::Strategy<DefaultNodeInfo>();
    }

    if (t_arg == "strong-branching") {
        return new StrongBranching::Strategy<DefaultNodeInfo>();
    }

    if (t_arg == "pseudo-cost") {
        return new PseudoCost::Strategy<DefaultNodeInfo>();
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

BranchAndBoundCallbackFactory<DefaultNodeInfo>* get_cutting_planes(const std::string& t_arg) {

    if (t_arg == "-") {
        return nullptr;
    }

    if (t_arg == "knapsack-cover") {
        return Cuts::KnapsackCover().with_max_cuts_factor(50).clone();
    }

    throw Exception("Unknown heuristic: " + t_arg);
}

int main(int t_argc, const char** t_argv) {

    if (t_argc < 2) {
        throw Exception("Arguments: path_to_instance solver [node-selection-rule] [branching-rule] [heuristic] [cutting-planes]");
    }

    const std::string path_to_instance = t_argv[1];
    const std::string solver = t_argv[2];
    std::string str_node_selection_rule = "-";
    std::string str_branching_rule = "-";
    std::string str_heuristic = "-";
    std::string str_cutting_planes = "-";

    Env env;
    auto model = create_kp_model(env, path_to_instance);

    if (solver == "external") {

        model.use(GLPK());

    } else if (solver == "bab") {

        if (t_argc != 7) {
            throw Exception("Arguments node-selection-rule, branching-rule, heuristic and cutting-planes are mandatory when solver is idol");
        }

        str_node_selection_rule = t_argv[3];
        str_branching_rule = t_argv[4];
        str_heuristic = t_argv[5];
        str_cutting_planes = t_argv[6];

        std::unique_ptr<NodeSelectionRuleFactory<DefaultNodeInfo>> node_selection_rule(get_node_selection_rule(str_node_selection_rule));
        std::unique_ptr<BranchingRuleFactory<DefaultNodeInfo>> branching_rule(get_branching_rule(str_branching_rule));
        std::unique_ptr<CallbackFactory> heuristic(get_heuristic(str_heuristic));
        std::unique_ptr<BranchAndBoundCallbackFactory<DefaultNodeInfo>> cutting_planes(get_cutting_planes(str_cutting_planes));

        model.use(
            BranchAndBound()
                .with_node_optimizer(GLPK::ContinuousRelaxation())
                .with_node_selection_rule(*node_selection_rule)
                .with_branching_rule(*branching_rule)
                .conditional((bool) heuristic, [&](auto& x) {
                    x.add_callback(*heuristic);
                })
                .conditional((bool) cutting_planes, [&](auto& x) {
                    x.add_callback(*cutting_planes);
                })
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
         << str_cutting_planes << ","
         << model.optimizer().time().count() << ","
         << get_n_solved_nodes(model) << ","
         << model.get_best_bound() << ","
         << model.get_best_obj()
         << std::endl;

    file.close();

    return 0;
}
