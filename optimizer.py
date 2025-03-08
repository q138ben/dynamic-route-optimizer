import math
import logging
import sqlite3
from typing import List, Tuple, Dict
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

class ProductionRouteOptimizer:
    def __init__(self, 
                 locations: List[Tuple[float, float]], 
                 num_vehicles: int, 
                 depot_index: int = 0,
                 demands: List[int] = None,
                 time_windows: List[Tuple[int, int]] = None,
                 vehicle_capacities: List[int] = None,
                 speed: float = 1.0):
        """
        Initialize the optimizer.
        :param locations: List of (x, y) coordinates. The first location is the depot.
        :param num_vehicles: Number of vehicles available.
        :param depot_index: Index of the depot.
        :param demands: Demand at each location (depot demand should be 0).
        :param time_windows: List of (start, end) time windows (in minutes) for each location.
        :param vehicle_capacities: Capacity for each vehicle.
        :param speed: Speed factor to convert distances into travel time.
        """
        self.locations = locations
        self.num_vehicles = num_vehicles
        self.depot_index = depot_index
        self.num_locations = len(locations)
        self.speed = speed

        # Defaults: zero demand and wide open time windows if not provided.
        self.demands = demands if demands is not None else [0] * self.num_locations
        self.time_windows = time_windows if time_windows is not None else [(0, 1000)] * self.num_locations
        self.vehicle_capacities = vehicle_capacities if vehicle_capacities is not None else [100] * self.num_vehicles
        
        # Traffic delays dictionary: keys are (i, j) pairs, values are extra delay costs.
        self.traffic_delays: Dict[Tuple[int, int], float] = {}
        
        # Setup production-level logging.
        self.logger = logging.getLogger("ProductionRouteOptimizer")
        self.logger.setLevel(logging.DEBUG)
        ch = logging.StreamHandler()
        ch.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        ch.setFormatter(formatter)
        if not self.logger.handlers:
            self.logger.addHandler(ch)
        
        # Compute initial matrices.
        self.distance_matrix = self.create_distance_matrix()
        self.travel_time_matrix = self.create_travel_time_matrix()

    def create_distance_matrix(self) -> List[List[int]]:
        """
        Compute the Euclidean distance matrix between all locations and adjust with traffic delays.
        Returns a 2D list of integer costs (scaled by 100 for precision).
        """
        matrix = []
        for i in range(self.num_locations):
            row = []
            for j in range(self.num_locations):
                if i == j:
                    row.append(0)
                else:
                    base_distance = math.hypot(self.locations[i][0] - self.locations[j][0],
                                               self.locations[i][1] - self.locations[j][1])
                    delay = self.traffic_delays.get((i, j), 0)
                    cost = base_distance + delay
                    row.append(int(cost * 100))
            matrix.append(row)
        self.logger.debug("Distance matrix updated.")
        return matrix

    def create_travel_time_matrix(self) -> List[List[int]]:
        """
        Compute a travel time matrix assuming travel time = (distance + delay) / speed.
        Time is scaled to integer values (multiplied by 100).
        """
        matrix = []
        for i in range(self.num_locations):
            row = []
            for j in range(self.num_locations):
                if i == j:
                    row.append(0)
                else:
                    base_distance = math.hypot(self.locations[i][0] - self.locations[j][0],
                                               self.locations[i][1] - self.locations[j][1])
                    delay = self.traffic_delays.get((i, j), 0)
                    travel_time = (base_distance + delay) / self.speed
                    row.append(int(travel_time * 100))
            matrix.append(row)
        self.logger.debug("Travel time matrix updated.")
        return matrix

    def update_traffic(self, edge: Tuple[int, int], delay: float):
        """
        Update the traffic delay for a specific edge (i, j) and recompute the matrices.
        :param edge: Tuple (i, j) indicating an edge between locations.
        :param delay: Additional delay cost to add.
        """
        self.traffic_delays[edge] = delay
        self.logger.info(f"Traffic update: edge {edge} now has an extra delay of {delay}.")
        self.distance_matrix = self.create_distance_matrix()
        self.travel_time_matrix = self.create_travel_time_matrix()

    def fetch_traffic_delay_from_db(self, edge: Tuple[int, int]) -> float:
        """
        Simulate fetching a traffic delay value from a SQLite database.
        In production, this would query a live database.
        """
        try:
            conn = sqlite3.connect('traffic_data.db')
            cursor = conn.cursor()
            query = "SELECT delay FROM traffic_delays WHERE from_node=? AND to_node=?"
            cursor.execute(query, (edge[0], edge[1]))
            result = cursor.fetchone()
            conn.close()
            if result:
                delay = result[0]
                self.logger.info(f"Fetched delay {delay} for edge {edge} from database.")
                return delay
            else:
                self.logger.info(f"No delay data for edge {edge} found in database. Using default 0.")
                return 0.0
        except Exception as e:
            self.logger.exception("Error fetching traffic delay from DB:")
            return 0.0

    def update_traffic_from_api(self):
        """
        Simulate a real-time API call that updates traffic delays.
        In production, this function would call an external API.
        """
        import random
        sample_edges = [(1, 2), (3, 4), (2, 5)]
        for edge in sample_edges:
            delay = random.uniform(0, 10)  # simulate a delay value.
            self.update_traffic(edge, delay)
        self.logger.info("Traffic delays updated from API simulation.")

    def solve(self, time_limit: int = 30) -> List[List[int]]:
        """
        Solve the multi-vehicle routing problem with capacity and time window constraints.
        :param time_limit: Maximum time (in seconds) allowed for the solver.
        :return: List of routes (one per vehicle), where each route is a list of location indices starting and ending at the depot.
        """
        self.logger.info("Starting VRP optimization with capacities and time windows using OR-Tools.")
        try:
            manager = pywrapcp.RoutingIndexManager(self.num_locations, self.num_vehicles, self.depot_index)
            routing = pywrapcp.RoutingModel(manager)

            # Define and register the distance callback.
            def distance_callback(from_index, to_index):
                from_node = manager.IndexToNode(from_index)
                to_node = manager.IndexToNode(to_index)
                return self.distance_matrix[from_node][to_node]
            transit_callback_index = routing.RegisterTransitCallback(distance_callback)
            routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

            # Add a Capacity dimension.
            def demand_callback(from_index):
                from_node = manager.IndexToNode(from_index)
                return self.demands[from_node]
            demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
            routing.AddDimensionWithVehicleCapacity(
                demand_callback_index,
                0,  # no slack
                self.vehicle_capacities,  # vehicle capacities
                True,  # start cumul to zero
                "Capacity"
            )

            # Add a Time Windows dimension.
            def time_callback(from_index, to_index):
                from_node = manager.IndexToNode(from_index)
                to_node = manager.IndexToNode(to_index)
                return self.travel_time_matrix[from_node][to_node]
            time_callback_index = routing.RegisterTransitCallback(time_callback)
            routing.AddDimension(
                time_callback_index,
                30,         # allow waiting time (slack)
                100000,     # maximum time per vehicle (set arbitrarily high)
                False,      # do not force start cumul to zero; we set time windows separately
                "Time"
            )
            time_dimension = routing.GetDimensionOrDie("Time")
            # Set time window constraints for each location.
            for node in range(self.num_locations):
                index = manager.NodeToIndex(node)
                start, end = self.time_windows[node]
                time_dimension.CumulVar(index).SetRange(start, end)
            # For each vehicle's start node (depot), set the time window.
            for vehicle_id in range(self.num_vehicles):
                index = routing.Start(vehicle_id)
                depot_window = self.time_windows[self.depot_index]
                time_dimension.CumulVar(index).SetRange(depot_window[0], depot_window[1])

            # Set search parameters.
            search_parameters = pywrapcp.DefaultRoutingSearchParameters()
            search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
            search_parameters.time_limit.seconds = time_limit

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
                    route.append(manager.IndexToNode(index))  # add depot at end
                    routes.append(route)
                return routes
            else:
                self.logger.error("No solution found!")
                return []
        except Exception as e:
            self.logger.exception("Error during optimization:")
            return []

# Example usage:
if __name__ == "__main__":
    # Define sample locations (x, y) coordinates.
    locations = [
        (0, 0),   # Depot
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
    # Sample demands for each location (depot demand is 0).
    demands = [0, 10, 15, 5, 10, 7, 8, 12, 9, 6]
    # Sample time windows (in minutes) for each location.
    time_windows = [
        (0, 300),   # Depot
        (30, 150),
        (60, 200),
        (50, 180),
        (20, 120),
        (80, 250),
        (100, 300),
        (40, 160),
        (70, 210),
        (90, 240)
    ]
    # Define vehicle capacities (all vehicles have capacity 30).
    vehicle_capacities = [30, 30, 30]

    optimizer = ProductionRouteOptimizer(
        locations, num_vehicles, depot_index=0,
        demands=demands, time_windows=time_windows,
        vehicle_capacities=vehicle_capacities, speed=1.0
    )
    
    # Optionally, update traffic data from a simulated API.
    optimizer.update_traffic_from_api()
    
    # Solve the VRP with all constraints.
    routes = optimizer.solve(time_limit=30)
    print("Optimized routes with capacity and time window constraints:")
    for idx, route in enumerate(routes):
        print(f"Vehicle {idx + 1}: {route}")
