from ortools.constraint_solver import pywrapcp, routing_enums_pb2


def Get_Joint_Sequence_And_Heuristic_Vector(data, manager, routing, solution, list_loc_and_task):
    agents_sequences = []
    agents_distances = []

    for vehicle_id in range(data['num_vehicles']):
        sequence = []
        route_distance = 0
        index = routing.Start(vehicle_id)

        while not routing.IsEnd(index):
            try:
                sequence.append(list_loc_and_task[manager.IndexToNode(index)][0])
            except TypeError as e:
                sequence.append(list_loc_and_task[manager.IndexToNode(index)])

            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)

        try:
            sequence.append(list_loc_and_task[manager.IndexToNode(index)][0])
        except TypeError as e:
            sequence.append(list_loc_and_task[manager.IndexToNode(index)])

        agents_sequences.append(sequence)
        agents_distances.append(route_distance)

    return agents_sequences, agents_distances


def Estimate_the_time_between_two_points(p1, p2, num_cols):
    try:
        p1 = p1[0]
    except TypeError as e:
        pass

    try:
        p2 = p2[0]
    except TypeError as e:
        pass

    return (abs(p1 // num_cols - p2 // num_cols) +
            abs(p1 % num_cols - p2 % num_cols))


def create_data_model(label, taskLocs, nom_cols):
    list_loc_and_task = list(label.v) + taskLocs
    data = {'distance_matrices': []}
    for agent in range(len(label.v)):
        distance_matrix = []
        for row in range(len(list_loc_and_task)):
            new_row = []
            for col in range(len(list_loc_and_task)):
                if row == col or (col < len(label.v) <= row):
                    new_row.append(0)
                else:
                    new_row.append(Estimate_the_time_between_two_points(list_loc_and_task[row], list_loc_and_task[col], nom_cols))
            distance_matrix.append(new_row)
        data['distance_matrices'].append(distance_matrix)

    data['num_vehicles'] = len(label.v)
    data['depots'] = list(range(len(label.v)))

    return data


def SolveMHPP(label, taskLocs, nom_cols):
    data = create_data_model(label, taskLocs, nom_cols)

    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrices'][0]), data['num_vehicles'], data['depots'], data['depots'])
    routing = pywrapcp.RoutingModel(manager)

    transit_callback_indices = []
    for vehicle_id in range(data['num_vehicles']):
        def distance_callback(from_index, to_index, vehicle_id=vehicle_id):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return data['distance_matrices'][vehicle_id][from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        transit_callback_indices.append(transit_callback_index)

    for vehicle_id in range(data['num_vehicles']):
        routing.SetArcCostEvaluatorOfVehicle(transit_callback_indices[vehicle_id], vehicle_id)

    # Define a Min-Max constraint on the route length to minimize the maximum distance
    dimension_name = 'Distance'
    routing.AddDimensionWithVehicleTransits(
        transit_callback_indices,
        0,
        3000,
        True,
        dimension_name
    )
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.seconds = 30

    solution = routing.SolveWithParameters(search_parameters)

    agents_sequences, agents_distances = Get_Joint_Sequence_And_Heuristic_Vector(data, manager, routing, solution, list(label.v) + taskLocs)

    print((agents_sequences, agents_distances))
