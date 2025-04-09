import numpy as np


class CollisionChecker:
    def __init__(self, plant, plant_context, scene_graph, scene_graph_context):
        self.plant = plant
        self.plant_context = plant_context
        self.scene_graph = scene_graph
        self.scene_graph_context = scene_graph_context

    def is_valid(self, q: np.array, min_penetration=0.02):
        self.plant.SetPositions(self.plant_context, q)

        # perform collision checking.
        query_object = self.scene_graph.get_query_output_port().Eval(
            self.scene_graph_context
        )
        inspector = query_object.inspector()
        colliding_bodies = set()
        for penetration in query_object.ComputePointPairPenetration():
            if min_penetration <= penetration.depth:
                body1, body2 = [
                    self.plant.GetBodyFromFrameId(inspector.GetFrameId(geometry_id))
                    for geometry_id in [penetration.id_A, penetration.id_B]
                ]
                colliding_bodies.update([(body1, body2), (body2, body1)])

        # return True if no collisions, False otherwise.
        return len(colliding_bodies) == 0
