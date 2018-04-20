################################################################################
# Module: gpx.py
# Description: Read in, parse, and analyze gpx tracks.
# License: MIT, see full license in LICENSE.txt
# Web: https://github.com/gboeing/osmnx
# Author: Heidi Hurst
################################################################################

import gpxpy
import datetime
import requests
import time
import re
import networkx as nx
import logging as lg
import os

from .utils import log

from . import core


def parse_gpx(filepath):
    """
    Extract lat, long, timestamp from gpx file.

    Uses package gpxpy to read a file and extract lat, long, and timestamp.

    Parameters
    ----------
    filepath : string
        relative or absolute path to gpx file

    Returns
    -------
    lat : list
        ordered list of all latitudes
    lon : list
        ordered list of all longitudes
    timestamp : list
        ordered list of extracted timestamps
    """

    # read in file
    gpx_file = open(filepath, 'r')

    # parse GPS track
    gps = gpxpy.parse(gpx_file)

    # create coordinates lists
    lat = []
    lon = []
    time = []
    for track in gps.tracks:
        for segment in track.segments:
            for point in segment.points:
                lat.append(point.latitude)
                lon.append(point.longitude)
                # TODO: convert to timestamp
                time.append(point.time)
    return lat, lon, time


# def request_url(lat, lon, maxpoints=0):
#     """
#     Convert lat and lon to a url query string.
#
#     Converts lat and lon to a query string using osrm router.
#
#     Parameters
#     ----------
#     lat : list
#         ordered list of all latitudes
#     lon : list
#         ordered list of all longitudes
#     maxpoints: int
#         maximum number of points to be included, defaults to all
#
#     Returns
#     -------
#     url : string
#         query url to be used for osrm routing
#     """
#     # how many points should we include? can we include all of them?
#     baseurl = 'http://router.project-osrm.org/match/v1/walking/'
#     options = '?overview=full&annotations=true'
#
#     coords = ''
#     # pick every nth coordinate up to maxpoints
#     npoints = len(lat) % maxpoints
#     point_interval = int(len(lat)/max(npoints,1))
#     for i in range(npoints):
#         coords += str(lon[i*point_interval]) + ',' + str(lat[i*point_interval]) + ';'
#
#    return baseurl + coords[:-1] + options


def match_request(url, pause_duration=None, timeout=180, error_pause_duration=None):
    """
    Send a request to the Overpass API via HTTP POST and return the JSON
    response.

    Parameters
    ----------
    data : dict or OrderedDict
        key-value pairs of parameters to post to the API
    pause_duration : int
        how long to pause in seconds before requests, if None, will query API
        status endpoint to find when next slot is available
    timeout : int
        the timeout interval for the requests library
    error_pause_duration : int
        how long to pause in seconds before re-trying requests if error

    Returns
    -------
    dict
    """

    # define the Overpass API URL, then construct a GET-style URL as a string to
    # hash to look up/save to cache
    prepared_url = requests.Request('GET', url).prepare().url
    cached_response_json = core.get_from_cache(prepared_url)

    if cached_response_json is not None:
        log('Returning cached response for {}'.format(prepared_url))
        # found this request in the cache, just return it instead of making a
        # new HTTP call
        return cached_response_json

    else:
        # if this URL is not already in the cache, pause, then request it
        if pause_duration is None:
            this_pause_duration = core.get_pause_duration()
        log('Pausing {:,.2f} seconds before making API GET request'.format(this_pause_duration))
        time.sleep(this_pause_duration)
        start_time = time.time()
        log('Posting to {} with timeout={}, "{}"'.format(url, timeout, url))
        response = requests.get(url, timeout=timeout, headers=core.get_http_headers())

        # get the response size and the domain, log result
        size_kb = len(response.content) / 1000.
        domain = re.findall(r'//(?s)(.*?)/', url)[0]
        log('Downloaded {:,.1f}KB from {} in {:,.2f} seconds'.format(size_kb, domain, time.time()-start_time))

        try:
            response_json = response.json()
            if 'remark' in response_json:
                log('Server remark: "{}"'.format(response_json['remark'], level=lg.WARNING))
            # only save if the status code is 200
            if response.status_code == 200:
                core.save_to_cache(prepared_url, response_json)
            else:
                log('Server at {} returned status code {}; failed to parse.'.format(domain,
                                                                                    response.status_code),
                                                                                    level=lg.WARNING)
        except Exception:
            #429 is 'too many requests' and 504 is 'gateway timeout' from server
            # overload - handle these errors by recursively calling
            # overpass_request until we get a valid response
            if response.status_code in [429, 504]:
                # pause for error_pause_duration seconds before re-trying request
                if error_pause_duration is None:
                    error_pause_duration = core.get_pause_duration()
                log('Server at {} returned status code {} and no JSON data. Re-trying request in {:.2f} seconds.'.format(domain,
                                                                                                                         response.status_code,
                                                                                                                         error_pause_duration),
                                                                                                                         level=lg.WARNING)
                #time.sleep(error_pause_duration)
                #response_json = core.overpass_request(data=data, pause_duration=pause_duration, timeout=timeout)

            # else, this was an unhandled status_code, throw an exception
            else:
                log('Server at {} returned status code {} and no JSON data'.format(domain, response.status_code), level=lg.ERROR)
                raise Exception('Server returned no JSON data.\n{} {}\n{}'.format(response, response.reason, response.text))

            return response_json


def reset_frequency(g):
    """
    Add dummy frequency field to all edges, set to 0.

    Parameters
    ----------
    G : networkx graph object
        graph object representing area of interest

    Returns
    -------
    none
    """
    # create frequency field and set to zero
    nx.set_edge_attributes(g, name='frequency', values=0)


def request_url(lat, lon, maxpoints=0):
    """
    Convert lat and lon to a url query string.

    Converts lat and lon to a query string using osrm router.

    Parameters
    ----------
    lat : list
        ordered list of all latitudes
    lon : list
        ordered list of all longitudes
    maxpoints: int
        maximum number of points to be included, defaults to all

    Returns
    -------
    url : string
        query url to be used for osrm routing
    """
    # set maxpoints to len lat if added
    if maxpoints == 0:
        maxpoints = len(lat)

    # how many points should we include? can we include all of them?
    baseurl = 'http://router.project-osrm.org/match/v1/walking/'
    options = '?overview=full&annotations=true'

    coords = ''
    # pick every nth coordinate up to maxpoints
    npoints = len(lat) % maxpoints
    point_interval = int(len(lat) / max(npoints, 1))
    for i in range(npoints):
        coords += str(lon[i * point_interval]) + ',' + str(lat[i * point_interval]) + ';'

    return baseurl + coords[:-1] + options


def edges_from_matchings(matching):
    """
    Identify all edges within a matching.

    Parameters
    ----------
    matching : list
        of all matchings returned by matching api

    Returns
    -------
    edges : list
        of all edge tuples
    """
    edges = []
    nodes = []
    # only address highest confidence matching?
    m = 0
    for match in matching:
        print(m)
        m += 1
        # only address good matchings
        if match['confidence'] > 0.95:
            # look through all legs
            for leg in match['legs']:
                # Extract all node sets
                node = leg['annotation']['nodes']
                for i in range(len(node) - 1):
                    if (node[i],node[i+1],0) not in edges:
                        edges.append((node[i], node[i+1], 0))
    return edges


def frequency_dict_from_matchings(matching, freq={}):
    """
    Identify all edges within a matching.

    Parameters
    ----------
    matching : list
        of all matchings returned by matching api

    freq : dictionary (optional)
        dictionary of frequency, with edge tuples (start, end, 0) as keys

    Returns
    -------
    freq : dictionary (optional)
        dictionary of frequency of occurrence of different edges
    """

    for match in matching:
        # only address good matchings
        if match['confidence'] > 0.95:
            # look through all legs
            for leg in match['legs']:
                # Extract all node sets
                node = leg['annotation']['nodes']
                for i in range(len(node) - 1):
                    key = (node[i], node[i+1], 0)
                    if key in freq:
                        freq[key] += 1
                    else:
                        freq[key] = 1
    return freq


def freq_from_folder(filepath, freq={}, npoints=100):
    """
    Read in all gpx files within a folder, and return a dictionary of how frequently edges were traversed.

    Paremeters
    ----------
    filepath : string
        path to folder containing gpx tracks (local or relative)

    freq : dictionary (optional)
        dictionary of frequency, with edge tuples (start, end, 0) as keys

    Returns
    -------
    freq : dictionary (optional)
        dictionary of frequency of occurrence of different edges
    """

    # identify all GPX files within a folder
    files = os.listdir(filepath)
    for file in files:
        # look only at gpx files
        if file.endswith('.gpx'):
            # for every file, extract matching information
            gpx_path = os.path.join(filepath, file)
            log('Extracting points from {}'.format(gpx_path), level=lg.INFO)
            # parse lat, lon out of gpx file
            lat, lon, timestamp = parse_gpx(gpx_path)
            # create match request url
            url = request_url(lat, lon, npoints)
            # send mapmatch request
            resp = match_request(url)
            log(resp, level=lg.INFO)
            # create dictionary from resultant matchings
            # use try/catch to avoid key errors where 'matchings' isn't in json
            try:
                freq = frequency_dict_from_matchings(resp['matchings'], freq)
            except KeyError:
                freq = freq
    return freq


def edge_frequency_dictionary(edges, freq = {}):
    """
    Augments counters within graph G edges based on if they're contained within the edges list.

    Parameters
    ----------
    edges : list
        list of edge tuples in the format (start node, end node, 0)
    frequency : dictionary
        dictionary of frequencies, with edge tuples as keys in the
        format (start node, end node, 0) and frequency as integer values
        defaults to empty dictionary

    Returns
    -------
    frequency : dictionary
        dictionary of frequencies, with edge tuples as keys in the
        format (start node, end node, 0) and frequency as integer values
    """
    # iterate over edges, with each edge as a key
    for key in edges:
        if key in freq:
            freq[key] += 1
        else:
            freq[key] = 1
    return freq


def edges_from_route(route):
    '''
    Convertes route comprised of list of nodes into list of touples for use in other algorithms.

    Paremeters
    ----------
    route : list
        ordered list of nodes

    Returns
    -------
    edges : list
        of edge touples (start node, end node, 0)
    '''

    edges = []
    for i in range(len(route) - 1):
        edges.append((route[i], route[i + 1], 0))
    return edges


def augment_edge_frequency(G, edges):
    """
    Augment edge frequency by 1 based on list of edge tuples.

    Parameters
    ----------
    G : networkx graph object
        graph object representing area of interest
    edges : list
        list of all edge tuples

    Returns
    -------
    none
    """
    err = 0
    good = 0
    for edge in edges:
        try:
            # try to augment edge
            G[edge[0]][edge[1]][edge[2]]['frequency'] += 1
            good += 1
        except:
            # TODO: add error logging
            print("Error accessing information for edge")
            err += 1
    print(good)
    print(err)
