import xml.etree.ElementTree as ET
import geopy.distance
import math

input_data_filename = "./storage/data_1.graphml"
output_csv_file = "./storage/GraphML_topo.csv"

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
    started_node = start
    nodes_len = len(nodes_arr)
    s = {started_node}
    v = set(x for x in nodes_arr.keys())  # all nodes set
    d = [math.inf] * nodes_len
    for target_node in edges_bounds_dict[started_node]:
        d[target_node] = edges_distance_arr[str(min(started_node, target_node)) + ' ' +
                                            str(max(started_node, target_node))]['distance']
    # print("was", d)

    for _ in range(1, nodes_len):
        min_d = math.inf
        w = None
        for x in v - s:
            if d[x] <= min_d:
                min_d = d[x]
                w = x

        s |= {w}
        # print(w, d[w], s, v - s, d)

        for v_node in v - s:
            find_key = str(min(w, v_node)) + ' ' + str(max(w, v_node))
            # print(f"from {w} to {v_node}, key: {find_key}")
            new_distance = edges_distance_arr.get(find_key,
                                                  {'distance': math.inf})['distance']
            d[v_node] = min(d[v_node], d[w] + new_distance)
            # if w == 7 and v_node == 4:
            #     print("+++++++++++ I am here", find_key)
            #     print(f"{d[v_node]} new dist: {new_distance}")

    # print("end", d)
    # print("inf", d.count(math.inf))
    return d


def main():
    tree = ET.parse(input_data_filename)
    root = tree.getroot()

    print(root.attrib)

    # for child in root:
    #     print('tag:', child.tag, 'atr:', child.attrib)

    graph = tree.find('xml:graph', namespaces)
    print(graph)

    xml_data_arr = graph.findall('xml:data', namespaces)
    xml_nodes = graph.findall('xml:node', namespaces)
    xml_edges = graph.findall('xml:edge', namespaces)

    # 29 - Latitude (широта) 33 - Longitude (долгота)

    # print(data_arr)
    # print(nodes)
    # print(edges)

    # create dict for nodes with Name, latitude and longitude
    nodes_arr = dict()
    for node in xml_nodes:
        data_node_arr = node.findall('xml:data', namespaces)
        # print(int(node.attrib['id']), data_node_arr[1].text, data_node_arr[5].text)
        nodes_arr[int(node.attrib['id'])] = {"name": data_node_arr[6].text}
        nodes_arr[int(node.attrib['id'])]["latitude"] = data_node_arr[1].text
        nodes_arr[int(node.attrib['id'])]["longitude"] = data_node_arr[5].text
    print("nodes arr", nodes_arr)

    # create dict to storage connection between nodes (tow directional)
    edges_bounds_dict = {int(i): set() for i in range(len(xml_nodes))}

    # create dict for edges with distance and delay
    edges_distance_arr = dict()
    for edge in xml_edges:
        node_1 = int(edge.attrib['source'])
        node_2 = int(edge.attrib['target'])

        # if type(edges_bounds_dict[str(node_1)]) == 'set':
        #     edges_bounds_dict[str(node_1)] += {node_2}
        # else:
        #     edges_bounds_dict[str(node_1)] = {node_2}
        edges_bounds_dict[node_1].add(int(node_2))
        edges_bounds_dict[node_2].add(int(node_1))

        # print(edge.attrib['source'], edge.attrib['target'])
        # print(nodes_arr[node_1][0])
        dict_edge_key = str(node_1) + ' ' + str(node_2)
        distance = get_distance_by_points(nodes_arr[node_1]['latitude'], nodes_arr[node_1]['longitude'],
                                          nodes_arr[node_2]['latitude'], nodes_arr[node_2]['longitude'])
        edges_distance_arr[dict_edge_key] = {"distance": distance}
        edges_distance_arr[dict_edge_key]["delay"] = 4.8 * distance
    print("edge distance", edges_distance_arr)
    print("edge bounds", edges_bounds_dict)

    # create a connectivty graph table
    nodes_len = len(nodes_arr)
    visited = dfs(edges_bounds_dict, 0)
    print(len(visited), "=> граф связный")

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

    print()
    print()
    print()

    # algorithm Dijkstra for all nodes
    for i in nodes_arr.keys():
        d = alg_dijkstra(i, edges_bounds_dict, edges_distance_arr, nodes_arr)
        print(d)

    # for node, data in nodes_arr.items():
    #     print("#### find from", node)
    #     nodes_set_s = {node}  # start node
    #     nodes_set_begin = set(x for x in nodes_arr.keys()) - nodes_set_s  # all nodes
    #     target_nodes_len = len(nodes_set_begin)
    #     print("S:", nodes_set_s)
    #     print("all\S:", nodes_set_begin)
    #
    #     # initialization distances
    #     distances = [-1] * nodes_len
    #     distances[node] = -2
    #     for target_node in edges_bounds_dict[node]:
    #         distances[target_node] = edges_distance_arr[str(min(node, target_node)) + ' ' +
    #                                                     str(max(node, target_node))]["distance"]
    #     print("start distances to nodes", distances)
    #
    #     for i in range(1, target_nodes_len):
    #         rest_distances = {distances.index(x): x for x in distances if
    #                           (distances.index(x) in nodes_set_begin) and x > 0}
    #         print("rest distances", rest_distances)
    #         min_distance = min(x for x in rest_distances.values())
    #         min_node = distances.index(min_distance)
    #         print("min dist index", min_node, "min dist", min_distance)
    #         nodes_set_s |= {min_node}
    #         nodes_set_begin -= {min_node}
    #
    #         print("######## started", node, "iter", i)
    #         print(nodes_set_s)
    #         print(nodes_set_begin)
    #
    #         for node_set in nodes_set_begin:
    #             deep_distance = edges_distance_arr.get(str(min(min_node, node_set)) + ' ' +
    #                                                    str(max(min_node, node_set)), {'distance': -1})["distance"]
    #             print(str(min(min_node, node_set)) + ' ' +
    #                                                    str(max(min_node, node_set)) in edges_distance_arr, deep_distance)
    #             print(edges_distance_arr)
    #             print(min_node, "to", node_set)
    #             if deep_distance == -1:
    #                 print("\t++++++++++++ с текущего узла не дойти", node_set)
    #             if distances[node_set] == -1:
    #                 print("\t---------- в таблице до сих пор бесконечность", node_set)
    #             if distances[node_set] == -1 and deep_distance != -1:
    #                 distances[node_set] = deep_distance
    #                 print("дошли по новому узлу", deep_distance, edges_distance_arr.get(deep_distance))
    #             elif deep_distance != -1 and distances[node_set] != -1:
    #                 prev = distances[node_set]
    #                 distances[node_set] = min(distances[node_set], min_distance + deep_distance)
    #                 print("дошли по минимальному", node_set, distances[node_set], "=min(", prev, ", ", min_distance, "+", deep_distance)
    #             else:
    #                 distances[node_set] = -1
    #                 print("дойти не получилось")
    #         print("cur distance", distances)
    #
    #     print("end   distances to nodes", distances)


if __name__ == "__main__":
    main()
