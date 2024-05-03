#include <algorithm>
#include <cassert>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

/**
  * User-defined parameters.
*/
#define RANDOM_SEED 0
#define MIN_COST (-100)
#define MAX_COST 100


int main(int argc, char** argv) {
  // Fix random seed for deterministic output
  std::srand(RANDOM_SEED);

  // Parse command-line arguments
  if (argc != 5) {
    std::cout << "Usage: " << argv[0] << " <filename> <u/d> <#nodes> <#edges>\n";
    std::cout << "Usage: " << argv[0] << " 1.in u 10 20\n";
    return 1;
  }

  std::string filename = argv[1];
  std::string graph_type = argv[2];
  int num_nodes = std::atoi(argv[3]);
  int num_edges = std::atoi(argv[4]);

  std::cout << "File name: " << filename << "\n";
  std::cout << "Graph type: " << graph_type << "\n";
  std::cout << "Number of nodes: " << num_nodes << "\n";
  std::cout << "Number of edges: " << num_edges << "\n";

  // Check #edges and #nodes
  if (num_edges < num_nodes - 1) {
    std::cout << "#edges should be equal to or greater than #nodes - 1\n";
    return 3;
  }

  // Random integer in a range [min, max]
  auto random_int = [] (int min_value, int max_value) -> int {
    assert(min_value <= max_value);
    return min_value + (std::rand() % (max_value - min_value + 1));
  };  // End random_int

  /**
   * Start creating edges here.
  */
  // Create edges for a tree
  std::vector<std::pair<int, int> > edges;
  std::set<std::pair<int, int> > edge_set;

  // All node indexes
  std::vector<int> indexes(num_nodes, 0);
  for (int i = 0; i < num_nodes; ++i) {
    indexes[i] = i;
  }

  // Random shuffle indexes
#ifdef DEBUG
  std::cout << "Node indexes before random shuffle\n";
  for (int i = 0; i < num_nodes; ++i) {
    std::cout << indexes[i] << " ";
  }
  std::cout << "\n";
#endif

  std::random_shuffle(indexes.begin(), indexes.end());

#ifdef DEBUG
  std::cout << "Node indexes after random shuffle\n";
  for (int i = 0; i < num_nodes; ++i) {
    std::cout << indexes[i] << " ";
  }
  std::cout << "\n";
#endif

  // Create a tree
  for (int i = 1; i < num_nodes; ++i) {
    const int to_node = indexes[i];
    const int from_node = indexes[random_int(0, i - 1)];
    edges.emplace_back(from_node, to_node);
    assert(edge_set.find({from_node, to_node}) == edge_set.end());
    assert((graph_type != "u") ||
        (edge_set.find({to_node, from_node}) == edge_set.end()));
    edge_set.emplace(from_node, to_node);
  }

  // Enumerate and collect all edges
  std::vector<std::pair<int, int> > possible_edges;
  for (int i = 0; i < num_nodes; ++i) {
    for (int j = 0; j <= i; ++j) {
      possible_edges.emplace_back(j, i);
      if (i != j) {
        possible_edges.emplace_back(i, j);
      }
    }
  }

  // Random shuffle all possible edges
  std::random_shuffle(possible_edges.begin(), possible_edges.end());

  // Check if an edge is already selected.
  // For undirected graphs, check both edge directions
  auto edge_exist = [&] (std::string const& graph_type,
                         int from_node, int to_node) -> bool {
      if (edge_set.find({from_node, to_node}) != edge_set.end()) {
        return true;
      } else if (graph_type == "u") {
        if (edge_set.find({to_node, from_node}) != edge_set.end()) {
          return true;
        }
      }
      return false;
  };

  // Besides the edges of a tree, we need to add more edges to meet
  // the required number of edges i.e. num_edges
  for (int k = 0; (k < (int)possible_edges.size())
      && ((int)edges.size() < num_edges); ++k) {
    auto const& edge = possible_edges[k];
    if (!edge_exist(graph_type, edge.first, edge.second)) {
      const int from_node = edge.first;
      const int to_node = edge.second;
      edges.emplace_back(from_node, to_node);
      edge_set.emplace(from_node, to_node);
    }
  }

  // Random shuffle all the selected edges
  std::random_shuffle(edges.begin(), edges.end());

  // Check if the number of selected edges 
  // meets the required number of edges i.e. num_edges
  if (num_edges != (int)edges.size()) {
    std::cerr << "Error: #created edges (" << edges.size() << ")"
        << " != #required edges (" << num_edges << ")\n";
  }

  /**
   * Start writing file here.
  */
  // Open file
  std::ofstream fout(filename, std::ios::out);
  if (!fout) {
    std::cout << "Cannot open \"" << filename << "\"\n";
    return 2;
  }

  // Graph type, number of nodes, number of edges
  fout << graph_type << "\n";
  fout << num_nodes << "\n";
  fout << num_edges << "\n";

  // Edges with cost
  for (int i = 0; i < (int)edges.size(); ++i) {
    const int from_node = edges[i].first;
    const int to_node = edges[i].second;
    const int cost = random_int(MIN_COST, MAX_COST);
    fout << from_node << " " << to_node << " " << cost << "\n";
  }

  // Ending 0
  fout << "0";

  // Close file
  fout.close();

  std::cout << "Done!\n";

  return 0;
}
