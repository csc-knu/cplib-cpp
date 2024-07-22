#ifndef GRAPH_HPP
#define GRAPH_HPP 1

#include <cassert>
#include <deque>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace graph {

template <typename T>
struct weighted_graph {
 public:
  static constexpr auto inf = std::numeric_limits<T>::max();

  explicit weighted_graph(int n) : _n(n), g(n) { }

  void add_directed_edge(int u, int v, T w) {
    assert(0 <= u && u < _n);
    assert(0 <= v && v < _n);
    assert(0 <= w);
    g[u].emplace_back(v, w);
  }

  void add_edge(int u, int v, T w) {
    assert(0 <= u && u < _n);
    assert(0 <= v && v < _n);
    assert(0 <= w);
    add_directed_edge(u, v, w);
    add_directed_edge(v, u, w);
  }

  std::pair<std::vector<T>, std::vector<int>> bfs(int s) {
    assert(0 <= s && s < _n);
    std::vector<T> d(_n, inf);
    std::vector<int> p(_n, -1);
    d[s] = 0;
    std::queue<int> q;
    q.push(s);
    while (!q.empty()) {
      const auto v = q.front();
      q.pop();
      for (const auto& [u, w] : g[v]) {
        assert(w == 1);
        if (d[u] > d[v] + 1) {
          p[u] = v;
          d[u] = d[v] + 1;
          q.push(u);
        }
      }
    }
    return {d, p};
  }

  std::pair<std::vector<T>, std::vector<int>> bfs01(int s) {
    assert(0 <= s && s < _n);
    std::vector<T> d(_n, inf);
    std::vector<int> p(_n, -1);
    d[s] = 0;
    std::deque<int> q;
    q.push_back(s);
    while (!q.empty()) {
      const auto v = q.front();
      q.pop_front();
      for (const auto& [u, w] : g[v]) {
        assert(w == 0 || w == 1);
        if (d[u] > d[v] + w) {
          d[u] = d[v] + w;
          p[u] = v;
          if (w) {
            q.push_back(u);
          } else {
            q.push_front(u);
          }
        }
      }
    }
    return {d, p};
  }

  // TODO: Dial's algorithm

  // TODO: Dijkstra

  // TODO: Bidirectional BFS

 private:
  int _n;
  std::vector<std::vector<std::pair<int, T>>> g;
};

}  // namespace graph

#endif  // GRAPH_HPP
