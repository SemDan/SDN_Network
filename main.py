import xml.etree.ElementTree as ET
import geopy.distance

input_data_filename = "./storage/data_1.graphml"
output_csv_file = "./storage/GraphML_topo.csv"

namespaces = {
    'xml': "http://graphml.graphdrawing.org/xmlns"
}


def get_distance_by_points(latitude_1, longitude_1, latitude_2, longitude_2):
    coords_1 = (latitude_1, longitude_1)
    coords_2 = (latitude_2, longitude_2)

    return geopy.distance.geodesic(coords_1, coords_2).km


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
        # print(node.attrib['id'], data_node_arr[1].text, data_node_arr[5].text)
        nodes_arr[node.attrib['id']] = {"name": data_node_arr[6].text}
        nodes_arr[node.attrib['id']]["latitude"] = data_node_arr[1].text
        nodes_arr[node.attrib['id']]["longitude"] = data_node_arr[5].text
    print(nodes_arr)

    # create dict to storage connection between nodes (tow directional)
    edges_bounds_dict = {str(i): set() for i in range(len(xml_nodes))}
    print(edges_bounds_dict)

    # create dict for edges with distance and delay
    edges_distance_arr = dict()
    for edge in xml_edges:
        node_1 = edge.attrib['source']
        node_2 = edge.attrib['target']

        # if type(edges_bounds_dict[str(node_1)]) == 'set':
        #     edges_bounds_dict[str(node_1)] += {node_2}
        # else:
        #     edges_bounds_dict[str(node_1)] = {node_2}
        edges_bounds_dict[node_1].add(int(node_2))
        # edges_bounds_dict[str(node_2)] += {node_1}

        # print(edge.attrib['source'], edge.attrib['target'])
        # print(nodes_arr[node_1][0])
        dict_edge_key = str(node_1) + ' ' + str(node_2)
        distance = get_distance_by_points(nodes_arr[node_1]['latitude'], nodes_arr[node_1]['longitude'],
                                          nodes_arr[node_2]['latitude'], nodes_arr[node_2]['longitude'])
        edges_distance_arr[dict_edge_key] = {"distance": distance}
        edges_distance_arr[dict_edge_key]["delay"] = 4.8 * distance
    print(edges_distance_arr)
    print(edges_bounds_dict)

    # create CSV table
    csv_file = open(output_csv_file, 'w')
    print("Node 1 (id),Node 1 (label),Node 1 (longitude),Node 1 (latitude),"
          "Node 2 (id),Node 2 (label),Node 2 (longitude),Node 2 (latitude),"
          "Distance (km),Delay(mks)", file=csv_file)

    for node, value in nodes_arr.items():
        print(f"{int(node) + 1},{value['name']}", file=csv_file)

    csv_file.close()


if __name__ == "__main__":
    main()
