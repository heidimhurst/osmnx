################################################################################
# Module: route.py
# Description: Create routes of fixed length within a tolerance.
# License: MIT, see full license in LICENSE.txt
# Web: https://github.com/gboeing/osmnx
# Author: Heidi Hurst
################################################################################

import networkx as nx
import logging as lg
import time

from . import core
from .utils import log, get_nearest_node, add_edge_bearings


def evaluate_edges(graph, route, freq={}, eval_function=lambda x: x['length'], *args, **kwargs):
    # get all neighbors of current node
    current_node = route[-1]
    # edges = [(current_node, neighbor, 0) for neighbor in nx.all_neighbors(graph, current_node)]

    # get all neighbors
    neighbors = [neighbor for neighbor in nx.all_neighbors(graph, current_node)]
    # ensure that you cannot revisit the previous node
    if len(route) > 1:
        previous_node = route[-2]
        try:
            neighbors.remove(previous_node)
        except ValueError:
            pass

    # initialize suitability list
    suitability = []

    # for each neighbor, query edge attributes
    for neighbor in neighbors:
        # initialize attribute dictionary
        attributes = dict()
        # road length
        attributes['length'] = graph[current_node][neighbor][0]['length']

        # frequency
        attributes['frequency'] = 0
        if (current_node, neighbor, 0) in freq.keys():
            attributes['frequency'] += freq[(current_node, neighbor, 0)]
        elif (neighbor, current_node, 0) in freq.keys():
            attributes['frequency'] += freq[(neighbor, current_node, 0)]

        # bearings
        if len(route) > 1:
            attributes['previous_bearing'] = graph[previous_node][current_node][0]['bearing']
        else:
            attributes['previous_bearing'] = None

        attributes['next_bearing'] = graph[current_node][neighbor][0]['bearing']
        # attributes['home_bearing'] =

        # append suitability to list
        print(eval_function(attributes))
        suitability.append(eval_function(attributes))

        # select node with optimized suitability
        next_node = neighbors[suitability.index(min(suitability))]

    return next_node


def inbound_optimization(attributes={}):
    print(attributes)
    suitability = attributes['length']

    return suitability


def next_inbound_node(graph, route, freq={}, *args, **kwargs):
    # call evaluate edges with inbound node parameters?
    next_node = evaluate_edges(graph=graph, route=route, freq=freq, eval_function=inbound_optimization)

    return next_node


def generate_route(lat, lon, goal_length, tolerance=0.5, length_unit='km', freq={}, *args, **kwargs):
    #
    start_time = time.time()

    # convert km to meters
    if length_unit == 'km':
        goal_length = 1000 * goal_length
        tolerance = 1000 * tolerance

    # get graph of area
    # TODO: modify to load from point id?
    streets = core.graph_from_point((lat, lon), distance=goal_length / 1.75)
    # add length attribute to all nodes
    core.add_edge_lengths(streets)
    add_edge_bearings(streets)
    # get start node
    start_node = ox.get_nearest_node(streets, (lat, lon))

    # setup
    route_length = 0
    route = [start_node]

    # outbound portion
    #    while route_length < goal_length/2:
    #        # select next node based on optimization function
    #        next_node = next_outbound_node(streets, route, freq, *args, **kwargs)
    #        # augment route length
    #        route_length += streets[route[-1]][next_node]['length']
    #        # add new node to route
    #        route.append(next_node)

    # inbound portion (return)
    #    while route[-1] != start_node and route_length < goal_length - tolerance:
    # i = 2
    while route_length < goal_length - tolerance:
        # ox.log('Searching for node number {} from current node id {}'.format(i, route[-1]))
        # print(route)
        # i += 1
        # select node based on inbound optimization function
        next_node = next_inbound_node(streets, route, freq, *args, **kwargs)
        # augment route length
        route_length += streets[route[-1]][next_node][0]['length']
        # add new node to route
        route.append(next_node)

        # prevent function for running forever
        if route_length > goal_length + tolerance:
            log('Unable to find route of length {} {} for start node {} with tolerance {}.  Process took {:.2f} seconds'.format(
                    goal_length,
                    length_unit,
                    start_node,
                    tolerance,
                    time.time() - start_time),
                level=lg.WARNING)
            route = [start_node]
            break

    log('Found route of length {:.0f}m in {:.2f} seconds'.format(route_length, time.time() - start_time))
    return route
