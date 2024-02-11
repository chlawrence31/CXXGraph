#ifndef __CXXGRAPH_Dinc_IMPL_H__
#define __CXXGRAPH_Dinc_IMPL_H__

#pragma once

#include "CXXGraph/Graph/Graph_decl.h"

using namespace std;

namespace CXXGraph
{
    template <typename T>
    double Graph<T>::dincs_algorithm(const Node<T> &source,const Node<T> &target) const
    {
    //Used the basics from FordFulkerson implementation to set the basics for the implementation of Dinc's algorithm

    unordered_map<shared<const Node<T>>, shared<const Node<T>>, nodeHash<T>> parent;
    unordered_map< shared<const Node<T>>, std::unordered_map<shared<const Node<T>>, double, nodeHash<T>>, nodeHash<T>> weightMap;

    auto edgeSet = this->getEdgeSet();
    for (const auto &edge : edgeSet) 
    {
    if (edge->isWeighted().value_or(false)) 
    {
      shared<const DirectedWeightedEdge<T>> dw_edge = static_pointer_cast<const DirectedWeightedEdge<T>>(edge);
      weightMap[edge->getNodePair().first][edge->getNodePair().second] = dw_edge->getWeight();
    } 
    else 
    {
      weightMap[edge->getNodePair().first][edge->getNodePair().second] = 0;  
    }
    }
    double maximumFlow = 0;
    auto nodeSet = getNodeSet();
    auto source_node_ptr = *std::find_if
    (
    nodeSet.begin(), nodeSet.end(),
      [&source](auto node) { return node->getUserId() == source.getUserId(); }
    );
  auto target_node_ptr = *std::find_if
   (
      nodeSet.begin(), nodeSet.end(),
      [&target](auto node) { return node->getUserId() == target.getUserId(); }
    );
    auto sendFlow = [this, &source, &target, &parent, &weightMap](const double flow)
    for(auto i = target, i != source, i = parent[v])
    {
        auto u = parent[i];
        weightMap[i][u] -= flow;
        weightMap[u][i] += flow;
    }
    auto bfs_helper = [this, &source_node_ptr, &target_node_ptr, &parent, &weightMap]() -> bool 
    {
    std::unordered_map<shared<const Node<T>>, bool, nodeHash<T>> visited;
    std::queue<shared<const Node<T>>> queue;
    queue.push(source_node_ptr);
    visited[source_node_ptr] = true;
    parent[source_node_ptr] = nullptr;
    while (!queue.empty()) 
    {
      auto u = queue.front();
      queue.pop();
      for (auto &v : weightMap[u]) 
      {
        if (!visited[v.first] && v.second > 0) 
        {
          queue.push(v.first);
          visited[v.first] = true;
          parent[v.first] = u;
        }
      }
    }
    return(visited[target_node_pointer])
    }
    while(bfs_helper())
    {
        double pathFlow = numeric_limits<double>::max();
        for(auto v = target; v != source; v = parent[v])
        {
            auto u = parent[v];
            pathFlow = min(pathFlow, weightMap[u][v]);
        }
        maximumFlow += pathFlow;
    }
    }

}

#endif  // __CXXGRAPH_Dinc_IMPL_H__