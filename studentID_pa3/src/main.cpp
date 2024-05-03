#include <iostream>
#include <string>

#include "cycle_breaker.h"

// Main function
int main(int argc, char** argv) {
  // Parse command-line arguments
  if (argc != 3) {
    std::cout << "Usage: " << argv[0] << " <input_file_name> <output_file_name>"
        << "\n";
    std::cout << "Example: " << argv[0] << " 1.in 1.out\n";
    return 1;  // Command-line argument error
  }

  // Get input and output file names
  std::string input_filename = argv[1];
  std::string output_filename = argv[2];
  std::cout << "Input filename: " << input_filename << "\n";
  std::cout << "Output filename: " << output_filename << "\n";

  // Solve cycle breaking
  aaron::CycleBreaker cycle_breaker;
  bool solve_success = cycle_breaker.Solve(input_filename, output_filename);
  if (!solve_success) {
    std::cerr << "Cycle breaker failed to solve...\n";
    return 2;  // Failed to solve
  }

  return 0;  // Exit normally
}  // End main
