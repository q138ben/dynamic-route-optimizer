import math
import logging
from typing import List, Tuple, Dict
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

class ProductionRouteOptimizer:
    def __init__(self, locations: List[Tuple[float, float]], num_vehicles: int, depot_index: int = 0):
        """
        Initialize the optimizer.
        :param locations: List of (x, y) coordinates. The first location is assumed to be the depot.
        :param num_vehicles: Number of vehicles available.
        :param depot_index: Index of the depot in the locations list.
        """
        self.locations = locations
        self.num_vehicles = num_vehicles
        self.depot_index = depot_index
        self.num_locations = len(locations)
        # Dictionary for dynamic traffic delays. Key: (i, j), Value: extra delay (float)
        self.traffic_delays: Dict[Tuple[int, int], float] = {}
        # Setup logger for production-level logging
        self.logger = logging.getLogger("ProductionRouteOptimizer")
        self.logger.setLevel(logging.DEBUG)
        ch = logging.StreamHandler()
        ch.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        ch.setFormatter(formatter)
        if not self.logger.handlers:
            self.logger.addHandler(ch)
        # Compute initial distance matrix (adjusted for any delays)
        self.distance_matrix = self.create_distance_matrix()

    def create_distance_matrix(self) -> List[List[int]]:
        """
        Compute the Euclidean distance matrix between all locations and adjust with traffic delays.
        Returns a 2D list of integer costs (multiplied by 100 for precision).
        """
        matrix = []
        for i in range(self.num_locations):
            row = []
            for j in range(self.num_locations):
                if i == j:
                    row.append(0)
                else:
                    # Base Euclidean distance.
                    base_distance = math.hypot(self.locations[i][0] - self.locations[j][0],
                                               self.locations[i][1] - self.locations[j][1])
                    # Get any extra delay cost for this edge.
                    delay = self.traffic_delays.get((i, j), 0)
                    # Total cost = base distance + delay.
                    cost = base_distance + delay
                    # Multiply by 100 and cast to int (OR‑Tools requires integer costs).
                    row.append(int(cost * 100))
            matrix.append(row)
        self.logger.debug("Distance matrix updated.")
        return matrix

    def update_traffic(self, edge: Tuple[int, int], delay: float):
        """
        Update the traffic delay for a specific edge (i, j) and recompute the distance matrix.
        :param edge: Tuple of indices (i, j).
        :param delay: Additional cost (delay) to add to the edge.
        """
        self.traffic_delays[edge] = delay
        self.logger.info(f"Traffic update: edge {edge} now has an extra delay of {delay}.")
        # Recreate the distance matrix after the traffic update.
        self.distance_matrix = self.create_distance_matrix()

    def solve(self, time_limit: int = 30) -> List[List[int]]:
        """
        Solve the multi-vehicle routing problem using OR-Tools.
        :param time_limit: Time limit for the search in seconds.
        :return: A list of routes (one per vehicle), where each route is a list of location indices starting and ending at the depot.
        """
        self.logger.info("Starting VRP optimization using OR-Tools.")
        try:
            # Create the routing index manager.
            manager = pywrapcp.RoutingIndexManager(self.num_locations, self.num_vehicles, self.depot_index)
            # Create Routing Model.
            routing = pywrapcp.RoutingModel(manager)

            # Define the cost (distance) callback.
            def distance_callback(from_index, to_index):
                from_node = manager.IndexToNode(from_index)
                to_node = manager.IndexToNode(to_index)
                return self.distance_matrix[from_node][to_node]

            transit_callback_index = routing.RegisterTransitCallback(distance_callback)
            routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

            # Add a dimension for tracking cumulative distance.
            routing.AddDimension(
                transit_callback_index,
                0,              # no slack
                10000000,       # maximum travel distance (an arbitrarily high value)
                True,           # start cumul to zero
                "Distance"
            )

            # Set search parameters.
            search_parameters = pywrapcp.DefaultRoutingSearchParameters()
            search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
            search_parameters.time_limit.seconds = time_limit

            # Solve the problem.
            solution = routing.SolveWithParameters(search_parameters)

            if solution:
                self.logger.info("Solution found.")
                routes = []
                for vehicle_id in range(self.num_vehicles):
                    index = routing.Start(vehicle_id)
                    route = []
                    while not routing.IsEnd(index):
                        node = manager.IndexToNode(index)
                        route.append(node)
                        index = solution.Value(routing.NextVar(index))
                    route.append(manager.IndexToNode(index))  # add the end node (should be depot)
                    routes.append(route)
                return routes
            else:
                self.logger.error("No solution found!")
                return []
        except Exception as e:
            self.logger.exception("Error during optimization:")
            return []

# Example usage (this would be in your main.py or similar production entry-point):

if __name__ == "__main__":
    # Sample locations: depot plus several stops.
    locations = [
        (0, 0),      # Depot
        (2, 3),
        (5, 4),
        (1, 7),
        (6, 1),
        (3, 5),
        (4, 7),
        (8, 3),
        (7, 8),
        (2, 9)
    ]
    num_vehicles = 3
    optimizer = ProductionRouteOptimizer(locations, num_vehicles)

    # Solve the VRP with initial conditions.
    initial_routes = optimizer.solve(time_limit=30)
    print("Initial routes:")
    for idx, route in enumerate(initial_routes):
        print(f"Vehicle {idx + 1}: {route}")

    # Simulate a traffic update between locations 1 and 3 (for example, heavy traffic adds extra cost).
    optimizer.update_traffic((1, 3), delay=5.0)
    optimizer.update_traffic((3, 1), delay=5.0)  # Assuming symmetric delay for simplicity.

    # Re-solve after the traffic update.
    updated_routes = optimizer.solve(time_limit=30)
    print("\nUpdated routes after traffic changes:")
    for idx, route in enumerate(updated_routes):
        print(f"Vehicle {idx + 1}: {route}")
