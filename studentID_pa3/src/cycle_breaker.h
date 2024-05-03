#include <iostream>
#include <map>
#include <vector>

#ifndef CYCLE_BREAKER_H_
#define CYCLE_BREAKER_H_

namespace aaron {

#define kUNDEFINED (-1)

// CycleBreaker
class CycleBreaker {
 public:
  // Constructor and destructor
  CycleBreaker() = default;
  ~CycleBreaker() = default;

  // The main function of cycle breaker, including
  // parsing, solving, and writing results
  bool Solve(std::string input_filename, std::string output_filename);

 private:
  struct Edge {
    Edge(int f = 0, int t = 0, int c = 0)
        : from_node(f), to_node(t), cost(c) {}

    int from_node;
    int to_node;
    int cost;
  };  // End Edge

 private:
  // Read input file
  bool ReadInputFile(std::string input_filename);

  // Solving methods: undirected / directed
  void SolveUndirectedGraph(std::vector<Edge>& solution) const;
  void SolveDirectedGraph(std::vector<Edge>& solution) const;

  // Write result to output file
  bool WriteOutputFile(std::string output_filename,
                       std::vector<Edge> const& solution) const;

 private:
  // Meta data
  enum class GraphType {
    kUNDIRECTED,  // undirected graph
    kDIRECTED  // directed graph
  } graph_type_;

  int num_nodes_;
  int num_edges_;

  // Edges
  std::vector<Edge> edges_;

  // Undirected
  void KruskalMaxST(std::vector<Edge>& maxst,
                    std::vector<Edge>& complementary_maxst) const;

  // Directed graphs
  mutable std::vector<std::map<int, int> > adj_;
  mutable int g_index_;
  mutable std::vector<int> visited_;
  mutable std::vector<int> onstack_;

  bool IsCyclic() const;
  bool IsCyclicUtil(int v) const;
};  // class CycleBreaker

}  // namespace aaron

#endif  // CYCLE_BREAKER_H_