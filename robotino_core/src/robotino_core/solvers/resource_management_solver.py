from copy import copy
import numpy as np

def solve_brute_force(tasks, x_0):

    # Assertions
    assert isinstance(tasks, list)
    assert isinstance(x_0, int)

    # Compute resources
    res, seq, charging_cost = compute_resource_levels(tasks, x_0)

    # Init
    ch_index = None
    ch_station = None
    ch_cost = float('inf')
    ch_time = float('inf')

    # Init hist
    x_hist = []
    y_hist = []
    ch_time_opts_x = []
    ch_time_opts_y = []

    # Add charging station if necessary
    if sum(n < self.x_min for n in res) > 0:

        # Brute force over all positions
        for i in range(1, len(tasks)):

            # Objective function
            output = obj_function(i, tasks, x_0)

            # Save hist
            x_hist.append(i)
            y_hist.append(output["ch_cost"] + output["ch_time"])
            ch_time_opts_x.append(output['x_hist'])
            ch_time_opts_y.append(output['y_hist'])

            # Optimize
            if output["ch_cost"] + output["ch_time"] < ch_cost + ch_time:
                ch_index = i
                ch_station = output["ch"]
                ch_cost = output["ch_cost"]
                ch_time = output["ch_time"]

        # Solution found?
        if ch_cost < float('inf'):
            tasks = copy(tasks)
            tasks.insert(ch_index, ch_station)
            ch_level = ch_time / self.beta
            res, seq, charging_cost = compute_resource_levels(tasks, x_0, ch_level)
            return {"ch_index": ch_index, "ch_station": ch_station, "ch_cost": ch_cost, "ch_time": ch_time,
                        "x_hist": x_hist, "y_hist": y_hist, "ch_time_opts_x": ch_time_opts_x,
                        "ch_time_opts_y": ch_time_opts_y, "res": res, "seq": seq}
        else:
            return {"ch_index": None, "ch_station": None, "ch_cost": float('inf'), "ch_time": float('inf'),
                        "x_hist": x_hist, "y_hist": y_hist, "ch_time_opts_x": ch_time_opts_x,
                        "ch_time_opts_y": ch_time_opts_y,
                        "res": res, "seq": seq}
    else:
        return {"ch_index": None, "ch_station": None, "ch_cost": 0, "ch_time": 0,
                    "x_hist": x_hist, "y_hist": y_hist, "ch_time_opts_x": ch_time_opts_x,
                    "ch_time_opts_y": ch_time_opts_y,
                    "res": res, "seq": seq}

def obj_function(x, tasks, x_0):

    # Copy tasks
    tasks = copy(tasks)

    # Insert charging station
    ch = search_closest_charging_station(tasks[x], self.charging_stations)
    tasks.insert(x, ch)

    # Init hist
    x_hist = []
    y_hist = []

    # Find optimal charging time
    ch_cost = float('inf')
    ch_time = float('inf')
    for level in range(0, 100):

        # Compute results
        res, seq, cost = compute_resource_levels(tasks, x_0, level)

        # Plot
        #plt.figure(1)
        #ax = plt.subplot()
        #plot_resources(seq, res, ax)
        #plt.draw()
        #plt.pause(0.1)

        # Get optimal charging time
        x_hist.append(level)
        if sum(n > self.x_max for n in res) > 0:
            y_hist.append(10000)
            break
        else:
            if sum(n < self.x_min for n in res) <= 0:
                ch_cost = cost
                ch_time = level * self.beta
                y_hist.append(ch_cost + ch_time)
                break
            else:
                y_hist.append(10000)

    # Make output
    output = {"ch": ch, "ch_cost": ch_cost, "ch_time": ch_time, "x_hist": x_hist, "y_hist": y_hist}

    return output

def compute_resource_levels(tasks, x_0, refill_level=0):

    # Assertions
    assert isinstance(tasks, list)
    assert isinstance(x_0, int)

    # Compute cost matrix
    c = [np.linalg.norm(np.subtract(tasks[i], tasks[i + 1])) for i in range(len(tasks) - 1)]

    # Compute resources at nodes without charging
    x = [x_0] + list(x_0 - np.cumsum(np.multiply(self.alpha, c)))

    # Compute charging index
    index = [i for i, task in enumerate(tasks) if task in self.charging_stations]

    # Compute resources at nodes with charging
    seq = list(range(len(x)))
    if not len(index) == 0:

        # Add charging load to resources
        ch_plus = np.repeat(refill_level, len(x))
        ch_plus[:index[0]+1] = 0
        x += ch_plus

        # Add charged status
        x = list(x)
        x.insert(index[0]+1, x[index[0]] + refill_level)
        seq.insert(index[0]+1, index[0])

        # Calculate properties
        ch_cost = c[index[0] - 1] + c[index[0]]

    else:

        # Calculate properties
        ch_cost = 0

    return x, seq, ch_cost

def search_closest_charging_station(location, charging_stations):
    dists = [dist(location, station) for station in charging_stations]
    index = dists.index(min(dists))
    return charging_stations[index]

def dist(a, b):
    return [], np.linalg.norm(np.subtract(a, b))