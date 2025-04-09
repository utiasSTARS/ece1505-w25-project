import numpy as np
import os

def get_configurations():
    return {
    "q_goal_term_1_1_170":
        np.array([0.20, 1.70, -1.45, -0.87, 0.00, 1.40, 0.00, 0.02, 0.02,
                  -0.20, 1.70, 1.45, -0.87, 0.00, 1.40, 1.30, 0.02, 0.02]),
    "q_goal_term_1_1_125":
        np.array([0.20, 1.25, -1.45, -0.87, 0.00, 1.40, 0.00, 0.02, 0.02,
                  -0.20, 1.25, 1.45, -0.87, 0.00, 1.40, 1.30, 0.02, 0.02]),
    "q_goal_term_1_1_100":
        np.array([0.20, 1.00, -1.45, -0.87, 0.00, 1.40, 0.00, 0.02, 0.02,
                  -0.20, 1.00, 1.45, -0.87, 0.00, 1.40, 1.30, 0.02, 0.02]),
    "q_start_term_1_1_176":
        np.array([-1.09, 1.76,  0.86, -0.07, 0.00, 3.75, 2.89,  0.02, 0.02,
                   1.09, 1.76, -0.86, -0.07, 0.00, 3.75, 2.05, 0.02, 0.02]),
    "q_start_term_1_1_125":
        np.array([-1.09, 1.25, 0.86, -0.07, 0.00, 3.75, 2.89, 0.02, 0.02,
                  1.09, 1.25, -0.86, -0.07, 0.00, 3.75, 2.05, 0.02, 0.02]),
    "q_start_term_1_1_100":
        np.array([-1.09, 1.00, 0.86, -0.07, 0.00, 3.75, 2.89, 0.02, 0.02,
                  1.09, 1.00, -0.86, -0.07, 0.00, 3.75, 2.05, 0.02, 0.02]),
    "q_home_term_1_1":
        np.array([0, -0.785, 0, -2.35, 0, 1.57, np.pi / 4, 0.02, 0.02,
                   0, -0.785, 0, -2.35, 0, 1.57, np.pi / 4, 0.02, 0.02]),
    "q_home_term_1_1_min_200":
        np.array([0, -0.785, 0, -2.00, 0, 1.57, np.pi / 4, 0.02, 0.02,
                  0, -0.785, 0, -2.00, 0, 1.57, np.pi / 4, 0.02, 0.02]),
    "q_home_goal_term_1_1_j4":
        np.array([0, -0.785, 0, -0.87, 0, 1.57, np.pi / 4, 0.02, 0.02,
                  0, -0.785, 0, -0.87, 0, 1.57, np.pi / 4, 0.02, 0.02]),
    "q_home_goal_term_1_1_j6":
        np.array([0, -0.785, 0, -0.87, 0, 1.40, np.pi / 4, 0.02, 0.02,
                  0, -0.785, 0, -0.87, 0, 1.40, np.pi / 4, 0.02, 0.02]),
    "q_home_goal_term_1_1_j7":
        np.array([0, -0.785, 0, -0.87, 0, 1.40, 0.00, 0.02, 0.02,
                  0, -0.785, 0, -0.87, 0, 1.40, 1.30, 0.02, 0.02]),
    "q_home_goal_term_1_1_j1":
        np.array([0.20, -0.785, 0, -0.87, 0, 1.40, 0.00, 0.02, 0.02,
                  -0.20, -0.785, 0, -0.87, 0, 1.40, 1.30, 0.02, 0.02]),
    # "q_home_goal_term_1_1_j3":
    #     np.array([0.20, -0.785, -1.45, -0.87, 0, 1.40, 0.00, 0.02, 0.02,
    #               -0.20, -0.785, 1.45, -0.87, 0, 1.40, 1.30, 0.02, 0.02]),
    "q_home_goal_term_1_1_j2": #is not needed, because duplicate of goal
        np.array([0.20, 1.70, -1.45, -0.87, 0, 1.40, 0.00, 0.02, 0.02,
                  -0.20, 1.70, 1.45, -0.87, 0, 1.40, 1.30, 0.02, 0.02]),
    "q_home_start_term_1_1_j4":
        np.array([0, -0.785, 0, -0.07, 0, 1.57, np.pi / 4, 0.02, 0.02,
                  0, -0.785, 0, -0.07, 0, 1.57, np.pi / 4, 0.02, 0.02]),
    "q_home_start_term_1_1_j6":
        np.array([0, -0.785, 0, -0.07, 0, 3.75, np.pi / 4, 0.02, 0.02,
                  0, -0.785, 0, -0.07, 0, 3.75, np.pi / 4, 0.02, 0.02]),
    "q_home_start_term_1_1_j7":
        np.array([0, -0.785, 0, -0.07, 0, 3.75, 2.89, 0.02, 0.02,
                  0, -0.785, 0, -0.07, 0, 3.75, 2.05, 0.02, 0.02]),
    "q_home_start_term_1_1_j3":
        np.array([0, -0.785, 0.86, -0.07, 0, 3.75, 2.89, 0.02, 0.02,
                  0, -0.785, -0.86, -0.07, 0, 3.75, 2.05, 0.02, 0.02]),
    "q_home_start_term_1_1_j2_half":
        np.array([0, 0.5, 0.86, -0.07, 0, 3.75, 2.89, 0.02, 0.02,
                  0, 0.5, -0.86, -0.07, 0, 3.75, 2.05, 0.02, 0.02]),
    "q_home_start_term_1_1_j1":
        np.array([-1.09, 0.5, 0.86, -0.07, 0, 3.75, 2.89, 0.02, 0.02,
                   1.09, 0.5, -0.86, -0.07, 0, 3.75, 2.05, 0.02, 0.02]),
}


def get_region_files():
    parent_folder = "./examples/drake_gcs_planning/iris_regions"
    file_names = {"goal_term_170":
                      "iris_region_q_goal_term_1_1_170.pkl",
                  "goal_term_125":
                      "iris_region_q_goal_term_1_1_125.pkl",
                  "goal_term_100":
                      "iris_region_q_goal_term_1_1_100.pkl",
                  "start_term_176":
                      "iris_region_q_start_term_1_1_176.pkl",
                  "start_term_125":
                      "iris_region_q_start_term_1_1_125.pkl",
                  "start_term_100":
                      "iris_region_q_start_term_1_1_100.pkl",
                  "home_term":
                      "iris_region_q_home_term_1_1.pkl",
                  "home_term_1_1_min_200":
                      "iris_region_q_home_term_1_1_min_200.pkl",
                  "q_home_goal_term_1_1_j4":
                      "iris_region_q_home_goal_term_1_1_j4.pkl",
                  "q_home_goal_term_1_1_j6":
                      "iris_region_q_home_goal_term_1_1_j6.pkl",
                  "q_home_goal_term_1_1_j7":
                      "iris_region_q_home_goal_term_1_1_j7.pkl",
                  "q_home_goal_term_1_1_j1":
                      "iris_region_q_home_goal_term_1_1_j1.pkl",
                  "q_home_start_term_1_1_j4":
                      "iris_region_q_home_start_term_1_1_j4.pkl",
                  "q_home_start_term_1_1_j6":
                      "iris_region_q_home_start_term_1_1_j6.pkl",
                  "q_home_start_term_1_1_j7":
                      "iris_region_q_home_start_term_1_1_j7.pkl",
                  "q_home_start_term_1_1_j3":
                      "iris_region_q_home_start_term_1_1_j3.pkl",
                  "q_home_start_term_1_1_j2_half":
                      "iris_region_q_home_start_term_1_1_j2_half.pkl",
                  "q_home_start_term_1_1_j1":
                      "iris_region_q_home_start_term_1_1_j1.pkl"}

    for region_name, file_name in file_names.items():
        file_names[region_name] = os.path.join(parent_folder, file_name)
    return file_names


def get_minimum_graph_nodes():
    return ["goal_term_170",
            "goal_term_125",
            "goal_term_100",
            "start_term_176",
            "q_home_start_term_1_1_j6",
            "q_home_start_term_1_1_j1"]
