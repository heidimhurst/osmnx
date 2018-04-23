################################################################################
# Module: route.py
# Description: Create routes of fixed length within a tolerance.
# License: MIT, see full license in LICENSE.txt
# Web: https://github.com/gboeing/osmnx
# Author: Heidi Hurst
################################################################################

import networkx as nx
import numpy as np
import logging as lg
import time

from . import core
from .utils import log, get_nearest_node, add_edge_bearings
from .simplify import simplify_graph


def novelty_score(graph, route, freq={}):
    """
    Returns information about how novel a route is based on a frequency dictionary.

    Parameters
    ----------
    graph : networkx graph object
        graph containing OSM data for area of interest
    route : list
        list of nodes traversed by a route
    freq : dict
        dictionary of frequency of traversal, where keys are in format (startnode, endnode, 0)

    Returns
    -------
    novel_segments : int
        number of total segments on the route that have not been previously traveled
    novel_length : float
        length (in graph length terms, usually meters) of novel road segments

    """
    # iterate through all edges traversed
    novel_segments = 0
    novel_length = 0
    for i in range(len(route)-1):
        if not((route[i], route[i+1], 0) in freq.keys() or (route[i+1], route[i], 0) in freq.keys()):
            novel_segments += 1
            novel_length += seg_attribute(graph, route[i], route[i+1])
    return novel_segments, novel_length


def seg_attribute(graph, start, finish, attribute='length'):
    """
    Used to work around oneway streets, returns value of attribute for edge.

    Parameters
    ----------
    graph : networkx graph object
        graph containing OSM data for area of interest
    start : int
        id of start node
    finish : int
        id of finish node
    attribute : string
        name of attribute to be obtained (default: length)

    Returns
    -------
    length : float
    """
    try:
        attr = graph[start][finish][0][attribute]
    except KeyError:
        try:
            attr = graph[finish][start][0][attribute]
        except KeyError:
            attr = 0
            log("No attribute {} found between nodes {} and {}".format(attribute,
                                                                       start,
                                                                       finish),
                level=lg.ERROR)
    return attr


def total_route_length(graph, route):
    """
    Returns the length of a route.

    Parameters
    ----------
    graph : networkx graph object
        graph containing OSM data for area of interest
    route : list
        list of nodes traversed by route, in order

    Returns
    -------
    length : float
        length of route (in meters?)
    """
    length = 0
    for i in range(len(route)-1):
        length += seg_attribute(graph, route[i], route[i+1])

    return length


def evaluate_edges(graph, route, freq={}, eval_function=lambda x: x['length'], *args, **kwargs):
    """
    This function evaluates edges based on some evaluation function, returning the id of the best next node.

    Parameters
    ----------
    graph : networkx graph object
        graph containing OSM data for area of interest
    route : list
        list of nodes traversed by a route
    freq : dict
        dictionary of frequency of traversal, where keys are in format (startnode, endnode, 0)

    Returns
    -------
    next_node : int
        id of next best node
    """
    # get all neighbors of current node
    current_node = route[-1]

    # get all adjacent edges
    adj_edges = graph.out_edges(iter([current_node]), data=True)

    log(adj_edges)

    # ensure that you cannot revisit the previous nodes
    if len(route) > 1:
        previous_node = route[-2]
        adj_edges = [x for x in adj_edges if x[1] != previous_node]

    # remove neighbor nodes that are dead ends (only point to one node)(?)
    n = []
    badnode = []
    for edge in adj_edges:
        edge_end = edge[1]
        nneighbor = [neighbor for neighbor in nx.all_neighbors(graph, edge_end)]
        if len(set(nneighbor)) > 1:
            n.append(edge)
        else:
            badnode.append(edge)
            adj_edges.remove(edge)
    # neighbors = n

    # initialize suitability list
    suitability = []
    n_edge = []

    log(adj_edges)

    # for each edge, query edge attributes
    for edge in adj_edges:
        log("Testing edge {}".format(edge))
        # set dict of attributes
        attributes = edge[-1]

        # add attributes that aren't included by default
        # has it been traveled before on this trip?
        attributes['traveled'] = edge[0:-1] in zip(route, route[1:], [0]*(len(route) - 2))

        if (edge[0], edge[1], 0) in freq.keys():
            attributes['frequency'] = freq[(edge[0], edge[1], 0)]
        else:
            attributes['frequency'] = 0

        if len(route) > 1:
            attributes['previous_bearing'] = graph[previous_node][current_node][0]['bearing']

        suitability.append(eval_function(attributes))
        n_edge.append(edge[1])

    # select index randomly with probability proportional evaulation function
    suitability = [i/sum(suitability) for i in suitability]
    index = np.random.choice(list(range(len(suitability))), 1, suitability)[0].astype(np.int64)
    # pick edge with the lowest index
    next_node = n_edge[index]
    log("Choosing node {} (suitability {}) from among nodes with suitability {}".format(next_node,
                                                                                        suitability[index],
                                                                                        suitability))

    return next_node


def outbound_optimization(attributes={}):
    suitability = (not attributes['traveled']) * attributes['length'] - attributes['frequency']

    return suitability


def next_outbound_node(graph, route, freq={}, *args, **kwargs):
    """
    Wrapper for outbound optimization algorithm.

    Parameters
    ----------
    graph : networkx graph object
        graph containing OSM data for area of interest
    route : list
        list of nodes traversed by a route
    freq : dict
        dictionary of frequency of traversal, where keys are in format (startnode, endnode, 0)

    Returns
    -------
    next_node : int
        integer ID of next best node
    """

    # call evaluate edges with inbound node parameters?
    next_node = evaluate_edges(graph=graph, route=route, freq=freq, eval_function=outbound_optimization)

    return next_node


def inbound_optimization(attributes={}):
    suitability = (not attributes['traveled']) * attributes['length'] - attributes['frequency']

    return suitability


def next_inbound_node(graph, route, freq={}, *args, **kwargs):
    """
    Wrapper for inbound optimization algorithm.

    Parameters
    ----------
    graph : networkx graph object
        graph containing OSM data for area of interest
    route : list
        list of nodes traversed by a route
    freq : dict
        dictionary of frequency of traversal, where keys are in format (startnode, endnode, 0)

    Returns
    -------
    next_node : int
        integer ID of next best node
    """
    # call evaluate edges with inbound node parameters?
    next_node = evaluate_edges(graph=graph, route=route, freq=freq, eval_function=inbound_optimization)

    return next_node


def generate_route(lat, lon, goal_length, tolerance=0.5, length_unit='km', freq={}, *args, **kwargs):
    """
    This function generates a loop route of specified length within a tolerance for a given start point.

    This is accomplished by breaking the route creation process into two phases: outbound and inbound.
    For the outbound phase, route segment are chosen based on the maximum outbound_optimization score
    until the total route length has reached one half of the goal length.
    The route then heads for home and begins the inbound phase.
    For the inbound phase, the inbound_optimization

    Parameters
    ----------
    lat : float
        latitude of start point
    lon : float
        longitude of start point
    goal_length : float
        goal length of route
    tolerance : float
        amount of allowable error in route
    length_unit : string
        unit that goal_length and tolerance are specified in - default km
    freq: dictionary
        dictionary of frequency of traversal, where keys are in format (startnode, endnode, 0)

    Returns
    -------
    route : list
        list of nodes traversed by route
    """
    # initialize timing for route finding
    start_time = time.time()

    # convert km to meters
    if length_unit == 'km':
        goal_length = 1000 * goal_length
        tolerance = 1000 * tolerance

    # get graph of area
    # TODO: modify to load from point id?
    streets = core.graph_from_point((lat, lon), distance=goal_length / 1.75, network_type='walk')
    # simplify topology, if possible
    try:
        simplify_graph(streets)
    except:
        log('Unable to simplify graph further')
        pass
    # add length attribute to all nodes
    core.add_edge_lengths(streets)
    add_edge_bearings(streets)

    # get start node
    start_node = core.get_nearest_node(streets, (lat, lon))

    # setup
    route_length = 0
    route = [start_node]

    # outbound portion
    while route_length < goal_length/2:
        log('Searching for node number {} from current node id {}'.format(len(route) + 1, route[-1]))

        # select node based on inbound optimization function
        next_node = next_outbound_node(streets, route, freq, *args, **kwargs)

        # augment route length
        if type(next_node) == int:
            route_length += seg_attribute(streets, route[-1], next_node)
            route.append(next_node)
        else:
            # add new node to route
            route.extend(next_node)
            # todo: remove retraced edge(s)

    # inbound portion (return)
    while route[-1] != start_node and route_length < goal_length + tolerance:
        # while route_length < goal_length - tolerance:
        log('Searching for node number {} from current node id {}'.format(len(route) + 1, route[-1]))

        log('Shortest path from node {} takes {}m'.format(route[-1],
                                                          nx.shortest_path_length(streets, route[-1], route[0], 'length')))
        log('Shortest loop is {}m'.format(route_length + nx.shortest_path_length(streets, route[-1], route[0], 'length')))
        quick_return_length = route_length + nx.shortest_path_length(streets, route[-1], route[0], 'length')

        if quick_return_length < goal_length - tolerance:
            # select node based on inbound optimization function
            next_node = next_inbound_node(streets, route, freq, *args, **kwargs)

            # augment route length
            if type(next_node) == int:
                route_length += seg_attribute(streets, route[-1], next_node)
                route.append(next_node)
            else:
                # add new node to route
                route.extend(next_node)
                # todo: remove retraced edge(s)

        elif quick_return_length < goal_length:
            next_node = nx.shortest_path(streets, route[-1], route[0], 'length')
            route.extend(next_node)
            break

        # prevent function for running forever
        elif route_length > goal_length + tolerance:
            log('Unable to find route of length {} {} for start node {} with tolerance {}.  Process took {:.2f} seconds'.format(
                    goal_length,
                    length_unit,
                    start_node,
                    tolerance,
                    time.time() - start_time),
                level=lg.WARNING)
            route = [start_node]
            break
    novel_segments, novel_length = novelty_score(streets, route, freq)
    log('Found route of length {:.0f}m in {:.2f} seconds.  \
        Novel length: {:.0f}m Novel segments: {}/{}'.format(total_route_length(streets, route),
                                                            time.time() - start_time, novel_length,
                                                            novel_segments, len(route) - 1))
    return route
