#ifndef DISJOINT_SET_H
#define DISJOINT_SET_H

namespace aaron {

class DisjointSet {
  typedef int index_t;

 public:
  DisjointSet() = delete;
  explicit DisjointSet(index_t n) { Init(n); }
  ~DisjointSet() {
    delete [] parent_;
    delete [] rnk_;
  }

  // Initialize
  void Init(index_t n) {
    n_ = n;
    parent_ = new index_t[n_];
    rnk_ = new index_t[n_];
    for (index_t i = 0; i < n_; ++i) {
      rnk_[i] = 0;
      parent_[i] = i;
    }
  }

  // Find set
  index_t Find(index_t u) {
    return (u == parent_[u] ?
        u : parent_[u] = Find(parent_[u]));
  }

  // Union by rank
  void Merge(index_t x, index_t y) {
    x = Find(x);
    y = Find(y);
    if (x == y)
      return;

    if (rnk_[x] > rnk_[y])
      parent_[y] = x;
    else  // If rnk_[x] <= rnk_[y]
      parent_[x] = y;

    if (rnk_[x] == rnk_[y])
      rnk_[y]++;
  }

private:
  index_t* parent_;
  index_t* rnk_;
  index_t n_;  // Number of elements
};  // class DisjointSet

}  // namespace aaron

#endif // DISJOINT_SET_H