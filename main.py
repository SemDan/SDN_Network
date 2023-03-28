import xml.etree.ElementTree as ET
import geopy.distance
import sys
import math

input_data_filename = "test/data_1.graphml"
output_csv_file = "test/GraphML_topo.csv"

namespaces = {
    'xml': "http://graphml.graphdrawing.org/xmlns"
}


def get_distance_by_points(latitude_1, longitude_1, latitude_2, longitude_2):
    coords_1 = (latitude_1, longitude_1)
    coords_2 = (latitude_2, longitude_2)

    return geopy.distance.geodesic(coords_1, coords_2).km


def dfs(edges_bounds, start, visited=None):
    if visited is None:
        visited = set()
    visited.add(start)

    for next_node in edges_bounds[start] - visited:
        dfs(edges_bounds, next_node, visited)
    return visited


# algorithm Dijkstra for all nodes
def alg_dijkstra(start, edges_bounds_dict, edges_distance_arr, nodes_arr):
    new_edges_bounds_dict = {start: {node for node in edges_bounds_dict[start]}}
    started_node = start
    nodes_len = len(nodes_arr)
    s = {started_node}
    v = set(x for x in nodes_arr.keys())  # all nodes set
    d = [math.inf] * nodes_len
    for target_node in edges_bounds_dict[started_node]:
        d[target_node] = edges_distance_arr[str(min(started_node, target_node)) + ' ' +
                                            str(max(started_node, target_node))]['delay']

    for _ in range(1, nodes_len):
        min_d = math.inf
        w = None
        for x in v - s:
            if d[x] <= min_d:
                min_d = d[x]
                w = x

        s |= {w}

        for v_node in v - s:
            find_key = str(min(w, v_node)) + ' ' + str(max(w, v_node))
            new_distance = edges_distance_arr.get(find_key,
                                                  {'delay': math.inf})['delay']
            if d[v_node] > d[w] + new_distance:
                if new_edges_bounds_dict.get(w) is None:
                    new_edges_bounds_dict[w] = {v_node}
                else:
                    new_edges_bounds_dict[w] |= {v_node}

                if new_edges_bounds_dict.get(v_node) is None:
                    new_edges_bounds_dict[v_node] = {w}
                else:
                    new_edges_bounds_dict[v_node] |= {w}
                    for prev_node in new_edges_bounds_dict[v_node]:
                        new_edges_bounds_dict[prev_node] -= {v_node}
            d[v_node] = min(d[v_node], d[w] + new_distance)

            tmp_dict = dict(new_edges_bounds_dict)
            for node, target_nodes in tmp_dict.items():
                for target_node in tmp_dict[node]:
                    if new_edges_bounds_dict.get(target_node) is None:
                        new_edges_bounds_dict[target_node] = {node}
                    else:
                        new_edges_bounds_dict[target_node] |= {node}
    return d, new_edges_bounds_dict


# Combine components A and B from set C - набор компонент
def merge(A, B, C):
    for node in C[B]:
        C[A] |= {node}
    del C[B]


# Return name of the component from C, which have this node
def find(v, C):
    for component, nodes in C.items():
        for node in nodes:
            if v == node:
                return component
    return None


# Create from set C new component A, which has only one node v
def initial(A, v, C=None):
    if C is None:
        C = dict()

    val = C.get(A)
    if val is None:
        C[A] = {v}
    else:
        C[A] |= {v}

    return C


# V - множество вершин E и T - множество дуг
# nsomp текущее количество компонент
# edges - сортированные edge
# components - множество V сгрупированное во множество компонент
def Kruskal(V, E):
    T = dict()
    edges = set()
    nextcomp = 0
    ncomp = len(V)
    components = dict()

    for v in V:
        nextcomp += 1
        components = initial(nextcomp, v, components)

    while ncomp > 1:
        e = E[0]
        del E[0]
        u, v = [int(x) for x in e['key'].split()]
        ucomp = find(u, components)
        vcomp = find(v, components)
        if ucomp != vcomp:
            merge(ucomp, vcomp, components)
            ncomp -= 1
            if T.get(v) is None:
                T[v] = {u}
            else:
                T[v] |= {u}

            if T.get(u) is None:
                T[u] = {v}
            else:
                T[u] |= {v}
    return T


def count_edges(edges: dict):
    c = 0
    for node, target_nodes in edges.items():
        c += len(target_nodes)
    return c


def main():
    # parsing Graph
    tree = ET.parse(input_data_filename)
    root = tree.getroot()

    graph = tree.find('xml:graph', namespaces)

    xml_data_arr = graph.findall('xml:data', namespaces)
    xml_nodes = graph.findall('xml:node', namespaces)
    xml_edges = graph.findall('xml:edge', namespaces)

    # 29 - Latitude (широта) 33 - Longitude (долгота)

    # create dict for nodes with Name, latitude and longitude
    nodes_arr = dict()
    for node in xml_nodes:
        data_node_arr = node.findall('xml:data', namespaces)
        nodes_arr[int(node.attrib['id'])] = {"name": data_node_arr[6].text}
        nodes_arr[int(node.attrib['id'])]["latitude"] = data_node_arr[1].text
        nodes_arr[int(node.attrib['id'])]["longitude"] = data_node_arr[5].text
    # print("nodes arr", nodes_arr)

    # create dict to test connection between nodes (tow directional)
    edges_bounds_dict = {int(i): set() for i in range(len(xml_nodes))}

    # create dict for edges with distance and delay
    edges_distance_arr = dict()
    for edge in xml_edges:
        node_1 = int(edge.attrib['source'])
        node_2 = int(edge.attrib['target'])

        edges_bounds_dict[node_1].add(int(node_2))
        edges_bounds_dict[node_2].add(int(node_1))

        dict_edge_key = str(node_1) + ' ' + str(node_2)
        distance = get_distance_by_points(nodes_arr[node_1]['latitude'], nodes_arr[node_1]['longitude'],
                                          nodes_arr[node_2]['latitude'], nodes_arr[node_2]['longitude'])
        edges_distance_arr[dict_edge_key] = {"distance": distance}
        edges_distance_arr[dict_edge_key]["delay"] = 4.8 * distance
    # print("edge distance", edges_distance_arr)
    # print(f"edge bounds ({count_edges(edges_bounds_dict)})", edges_bounds_dict)

    # create a connectivty graph table
    nodes_len = len(nodes_arr)
    visited = dfs(edges_bounds_dict, 0)
    print(len(visited) == len(nodes_arr), "- граф связный")

    # create CSV table
    csv_file = open(output_csv_file, 'w')
    print("Node 1 (id),Node 1 (label),Node 1 (longitude),Node 1 (latitude),"
          "Node 2 (id),Node 2 (label),Node 2 (longitude),Node 2 (latitude),"
          "Distance (km),Delay(mks)", file=csv_file)

    for node, value in nodes_arr.items():
        for node_2 in edges_bounds_dict[node]:
            node_id = min(int(node), node_2)
            node_2_id = max(int(node), node_2)
            print(f"{int(node) + 1},{value['name']},{value['longitude']},{value['latitude']},"
                  f"{int(node_2) + 1},{nodes_arr[node_2]['name']},"
                  f"{nodes_arr[node_2]['longitude']},{nodes_arr[node_2]['latitude']},"
                  f"{edges_distance_arr[str(node_id) + ' ' + str(node_2_id)]['distance']:.4f},"
                  f"{edges_distance_arr[str(node_id) + ' ' + str(node_2_id)]['delay']:.4f}", file=csv_file)
    csv_file.close()

    # K1
    # apply algorithm Dijkstra for all nodes
    # and find min from max distances from all nodes if put router in all node alternately
    min_delay = math.inf
    min_delay_index = None
    min_edges_bounds = dict()
    min_d = None
    for i in nodes_arr.keys():
        d, new_edges_bounds = alg_dijkstra(i, edges_bounds_dict, edges_distance_arr, nodes_arr)
        max_delay = 0
        max_delay_index = None
        for index, delay in enumerate(d):
            if delay > max_delay and delay != math.inf:
                max_delay = delay
                max_delay_index = index
        if max_delay < min_delay:
            min_delay = max_delay
            min_delay_index = max_delay_index
            min_edges_bounds = dict(new_edges_bounds)
    print(f"answer K1: надо ставить контроллер в узел с номером {min_delay_index}: {min_delay}")  # res for K1
    # print(f"({count_edges(min_edges_bounds)}), min_edges_bounds", min_edges_bounds)

    # csv
    csv_file = open("test/GraphML_routes_K1.csv", 'w')
    print("Node 1 (id),Node 2 (id),Path type,Path,Delay (mks)", file=csv_file)
    for node in nodes_arr.keys():
        # d_min = alg_dijkstra(node, min_edges_bounds, edges_distance_arr, nodes_arr)[0]
        d = alg_dijkstra(node, edges_bounds_dict, edges_distance_arr, nodes_arr)[0]
        for target_node in min_edges_bounds[node]:
            print(f"{node},{target_node},main,-,{d[target_node]}", file=csv_file)
            print(f"{node},{target_node},reserv,-,{d[target_node]}", file=csv_file)
    csv_file.close()

    # K2
    # apply once algorithm Kroskala and aplay Dijkstra for all nodes
    E = [{'key': key, 'delay': value['delay']} for key, value in edges_distance_arr.items()]
    E.sort(key=lambda x: x['delay'])
    # print(E)
    E = Kruskal(nodes_arr, E)
    # print(f"({count_edges(E)}) {E}")
    # print(f"({count_edges(edges_bounds_dict)}) {edges_bounds_dict}")

    visited = dfs(E, 0)
    print(f"{len(visited) == len(nodes_arr)} - граф остался связным")
    # print(f"{alg_dijkstra(0, E, edges_distance_arr, nodes_arr)[0]}")

    min_delay = math.inf
    min_delay_index = None
    min_edges_bounds = dict(E)
    min_d = None
    for i in nodes_arr.keys():
        d, new_edges_bounds = alg_dijkstra(i, E, edges_distance_arr, nodes_arr)
        max_delay = 0
        max_delay_index = None
        for index, delay in enumerate(d):
            if delay > max_delay and delay != math.inf:
                max_delay = delay
                max_delay_index = index
        if max_delay < min_delay:
            min_delay = max_delay
            min_delay_index = max_delay_index
            min_edges_bounds = dict(E)
    print(f"answer K2: надо ставить контроллер в узел с номером {min_delay_index}: {min_delay}")  # res for K2

    # create a csv for K2
    csv_file = open("test/GraphML_routes_K2.csv", 'w')
    print("Node 1 (id),Node 2 (id),Path type,Path,Delay (mks)", file=csv_file)
    for node in nodes_arr.keys():
        d_min = alg_dijkstra(node, E, edges_distance_arr, nodes_arr)[0]
        d = alg_dijkstra(node, edges_bounds_dict, edges_distance_arr, nodes_arr)[0]
        for target_node in min_edges_bounds[node]:
            print(f"{node},{target_node},main,-,{d_min[target_node]}", file=csv_file)
            print(f"{node},{target_node},reserv,-,{d[target_node]}", file=csv_file)
    csv_file.close()


if __name__ == "__main__":
    main()
