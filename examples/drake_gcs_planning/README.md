# Motion planning with GCS

## Generate convex sets (iris regions)

First, specify the configurations for constructing convex sets in the get_configurations() function located 
in convex_sets_config.py. Then, execute the following command in the top-level directory of the repo:

```bash
python examples/drake_gcs_planning/construct_convex_sets.py --action gen_convex_sets
```

The resulting .pkl file(-s) containing the iris region(-s) (convex set(-s)) will be 
saved in the ./examples/drake_gcs_planning/iris_regions directory.


## Build a connectivity graph 

To check the connectivity of the graph (how convex sets overlap), 
run the following command in the top-level directory of the repo:

```bash
python examples/drake_gcs_planning/construct_convex_sets.py --action gen_graph --full_graph
```

It will visualize the graph using the iris regions and their corresponding names defined in 
the get_region_files() function from convex_sets_config.py. The resulting visualization, graph.png, will be saved 
in the ./examples/drake_gcs_planning/iris_regions directory.

If you wish to visualize the overlap between specific convex sets, select them in the get_minimum_graph_nodes() 
function located in ./examples/drake_gcs_planning/iris_regions, and then run:
```bash
python examples/drake_gcs_planning/construct_convex_sets.py --action gen_graph
```

## Visualize iris regions 

To visualize iris regions, run the following command in the top-level directory of the repo:
```bash
python examples/drake_gcs_planning/construct_convex_sets.py --action vis_convex_sets --full_graph
```
or (the idea is similar to "Build a connectivity graph")
```bash
python examples/drake_gcs_planning/construct_convex_sets.py --action vis_convex_sets
```

If you want to visualize one specific iris region, run:
```bash
python examples/drake_gcs_planning/construct_convex_sets.py --action vis_convex_sets --region_name <region_name>
```

## GCS motion planning
Set optimization parameters in config.ini and run:
```bash
python examples/drake_gcs_planning/motion_planner.py
```
