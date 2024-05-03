#include <cassert>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <vector>

#include "cycle_breaker.h"
#include "disjoint_set.h"

namespace aaron {

bool CycleBreaker::Solve(std::string input_filename,
                         std::string output_filename) {
  // Read input file
  if (!ReadInputFile(input_filename)) {
    std::cerr << "Fail to read " << input_filename << "\n";
    return false;
  }

  // The removed edges to be written to output file
  std::vector<Edge> solution;

  // Solve for different graph types
  if (graph_type_ == GraphType::kUNDIRECTED) {
    // Solution to undirected graphs
    SolveUndirectedGraph(solution);
  } else if (graph_type_ == GraphType::kDIRECTED) {
    // Solution to directed graphs
    SolveDirectedGraph(solution);
  } else {
    return false;
  }

  // Write the result to output file
  if (!WriteOutputFile(output_filename, solution)) {
    std::cerr << "Fail to write " << output_filename << "\n";
    return false;
  }

  return true;
}  // End CycleBreaker::Solve

bool CycleBreaker::ReadInputFile(std::string input_filename) {
  std::cout << "---------\n";
  std::cout << "Reading input file " << input_filename << "\n";

  // Open file
  std::ifstream fin(input_filename, std::ios::in);
  if (!fin) {
    std::cerr << "Cannot open \"" << input_filename << "\"\n";
    return false;
  }

  // Graph type
  std::string graph_type_str;
  fin >> graph_type_str;
  if (graph_type_str == "u") {
    graph_type_ = GraphType::kUNDIRECTED;
  } else if (graph_type_str == "d") {
    graph_type_ = GraphType::kDIRECTED;
  } else {
    std::cerr << "Unknown graph type \"" << graph_type_str << "\"\n";
    return false;
  }
  std::cout << "Graph type: " << graph_type_str << "\n";

  // Number of nodes
  int num_nodes = 0;
  fin >> num_nodes;
  num_nodes_ = num_nodes;
  std::cout << "Number of nodes: " << num_nodes_ << "\n";

  // Number of edges
  int num_edges = 0;
  fin >> num_edges;
  num_edges_ = num_edges;
  std::cout << "Number of edges: " << num_edges_ << "\n";

  // Read edges with cost
  // No error handling
  for (int edge_index = 0; edge_index < num_edges_; ++edge_index) {
    int from_node_index = 0;
    int to_node_index = 0;
    int edge_cost = 0;
    fin >> from_node_index >> to_node_index >> edge_cost;

#ifdef DEBUG
    std::cout << from_node_index << " " << to_node_index << " " << edge_cost
        << "\n";
#endif

    assert(from_node_index < num_nodes_);
    assert(to_node_index < num_nodes_);

    // Insert this edge to edges_
    edges_.emplace_back(from_node_index, to_node_index, edge_cost);
  }

#ifdef DEBUG
  std::cout << "\n";
  for (auto const& edge : edges_) {
    std::cout << edge.from_node << " " << edge.to_node << " "
        << edge.cost << "\n";
  }
#endif

  // Close file
  fin.close();

  std::cout << "Done!\n";

  return true;
}  // End CycleBreaker::ReadInputFile

void CycleBreaker::SolveUndirectedGraph(std::vector<Edge>& solution) const {
  std::cout << "---------\n";
  std::cout << "Solving the undirected graph...\n";

  std::vector<Edge> maxst;  // Max spanning tree
  KruskalMaxST(maxst, solution);

  std::cout << "Done!\n";
}  // End CycleBreaker::SolveUndirectedGraph

void CycleBreaker::KruskalMaxST(std::vector<Edge>& maxst,
                                std::vector<Edge>& complementary_maxst) const {
  // We use Kruskal's algorithm
  DisjointSet disjoint_set(num_nodes_);

  // All edge indexes from 0 to num_edges - 1
  std::vector<int> edge_indexes(num_edges_, 0);
  for (int edge_index = 0; edge_index < num_edges_; ++edge_index) {
    edge_indexes[edge_index] = edge_index;
  }

#ifdef DEBUG
  std::cout << "Before sorting\n";
  for (size_t i = 0; i < edge_indexes.size(); ++i) {
    const int edge_index = edge_indexes[i];
    Edge const& edge = edges_[edge_index];
    std::cout << edge.from_node << " " << edge.to_node << " " << edge.cost
        << "\n";
  }
#endif

  // Sort indexes of edges
  // Sort the indexex by the "non-increasing" order of edge cost.
  // Largest cost -> ... -> smallest cost
  std::stable_sort(edge_indexes.begin(), edge_indexes.end(),
                   [&] (int edge_index_1, int edge_index_2) {
                    return edges_[edge_index_1].cost
                        > edges_[edge_index_2].cost;
                   });

#ifdef DEBUG
  std::cout << "After sorting\n";
  for (size_t i = 0; i < edge_indexes.size(); ++i) {
    const int edge_index = edge_indexes[i];
    Edge const& edge = edges_[edge_index];
    std::cout << edge.from_node << " " << edge.to_node << " " << edge.cost
        << "\n";
  }
#endif

  // Use Kruskal's algorithm for MAXIMUM spanning tree (MaxST)
  for (size_t i = 0; i < edge_indexes.size(); ++i) {
    const int edge_index = edge_indexes[i];
    Edge const& edge = edges_[edge_index];
    if (disjoint_set.Find(edge.from_node) != disjoint_set.Find(edge.to_node)) {
      // edge in MaxST
      disjoint_set.Merge(edge.from_node, edge.to_node);
      maxst.emplace_back(edge);
    } else {
      // edge not in MaxST, need to add it to solution
      complementary_maxst.emplace_back(edge);
    }
  }
}  // End CycleBreaker::KruskalMaxST

void CycleBreaker::SolveDirectedGraph(std::vector<Edge>& solution) const {
  std::cout << "---------\n";
  std::cout << "Solving the directed graph...\n";

  // Initialization
  g_index_ = 0;
  visited_.clear();
  visited_.resize(num_nodes_, g_index_);
  onstack_.clear();
  onstack_.resize(num_nodes_, g_index_);
  adj_.clear();
  adj_.resize(num_nodes_);

  // Kruskal's algorithm
  std::vector<Edge> maxst;  // Max spanning tree
  std::vector<Edge> complementary_maxst;
  KruskalMaxST(maxst, complementary_maxst);

  // Build adjacency list
  for (auto const& edge : maxst) {
    adj_[edge.from_node][edge.to_node] = edge.cost;
  }

  // All edge indexes from 0 to num_edges - 1
  std::vector<int> edge_indexes(num_edges_, 0);
  for (int edge_index = 0; edge_index < num_edges_; ++edge_index) {
    edge_indexes[edge_index] = edge_index;
  }

  // Sort indexes of edges
  // Sort the indexex by the "non-increasing" order of edge cost.
  // Largest cost -> ... -> smallest cost
  std::stable_sort(edge_indexes.begin(), edge_indexes.end(),
                   [&] (int edge_index_1, int edge_index_2) {
                    return edges_[edge_index_1].cost
                        > edges_[edge_index_2].cost;
                   });

  // Try to add edges back
  for (int edge_index : edge_indexes) {
    Edge const& edge = edges_[edge_index];
    if (edge.from_node == edge.to_node) {
      // Self-loop edge
      // TODO: can we remove this?
      solution.emplace_back(edge);
    } else {
      if (adj_[edge.from_node].count(edge.to_node) == 0) {
        // The edge is not in maxst
        if (edge.cost > 0) {
          // Add the edge
          adj_[edge.from_node][edge.to_node] = edge.cost;
          if (IsCyclic()) {
            // Cyclic, remove the edge from adj list
            // add the edge to solution
            adj_[edge.from_node].erase(edge.to_node);
            solution.emplace_back(edge);
          }
        } else {
          // We don't consider to add negative-cost edges back
          // Directly add it to solution
          solution.emplace_back(edge);
        }
      }
    }
  }

  std::cout << "Done!\n";
}  // End CycleBreaker::SolveDirectedGraph

bool CycleBreaker::WriteOutputFile(
    std::string output_filename,
    std::vector<Edge> const& solution) const {
  std::cout << "---------\n";
  std::cout << "Writing result to " << output_filename << "\n";

  // Open file
  std::ofstream fout(output_filename, std::ios::out);
  if (!fout) {
    std::cerr << "Cannot open \"" << output_filename << "\"\n";
    return false;
  }

  // Total cost calculation
  int total_edge_cost = 0;
  for (auto const& edge : solution) {
    const int cost = edge.cost;
    total_edge_cost += cost;
  }

  // Write to file
  fout << total_edge_cost << "\n";
  for (auto const& edge : solution) {
    fout << edge.from_node << " " << edge.to_node << " " << edge.cost << "\n";
  }

  // Close file
  fout.close();

  // Print messages
  std::cout << "Total removed cost: " << total_edge_cost << "\n";
  std::cout << "Number of removed edges: " << solution.size() << "\n";
  std::cout << "Done!\n";

  return true;
}  // End CycleBreaker::WriteOutputFile

bool CycleBreaker::IsCyclic() const {
  // Reference: https://www.geeksforgeeks.org/detect-cycle-in-a-graph/
  g_index_ += 2;  // Reset
  for (int v = 0; v < num_nodes_; ++v) {
    if ((visited_[v] != g_index_) && IsCyclicUtil(v)) {
      return true;
    }
  }
  return false;
}  // End IsCyclic::IsCyclic

bool CycleBreaker::IsCyclicUtil(int v) const {
  if (visited_[v] != g_index_) {
    // Mark the current node as visited
    // and part of recursion stack
    visited_[v] = g_index_;
    onstack_[v] = g_index_;

    // Recur for all the vertices adjacent to this vertex
    for (auto const& adj : adj_[v]) {
      int w = adj.first;  // adjacent node

      if ((visited_[w] != g_index_) && IsCyclicUtil(w)) {
        return true;
      } else if (onstack_[w] == g_index_) {
        return true;
      }
    }
  }

  // Remove the vertex from recursion stack
  onstack_[v] = g_index_ - 1;
  return false;
}  // End CycleBreaker::IsCyclicUtil

}  // namespace aaron