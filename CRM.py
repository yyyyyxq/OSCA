import numpy as np
from collections import deque, defaultdict
import matplotlib.pyplot as plt
import networkx as nx  
from CG import ConnectedGraphConstructor


 
class ConflictResolver:
    def __init__(self, graph_constructor):
        self.graph_constructor = graph_constructor  # 2.2节的图构建器
        self.pending_queue = deque()  # 待处理队列Ω
        self.conflict_free_set = []  # 无冲突集合Ψ
    
    def resolve_conflicts(self):
        """
        冲突解决主流程（2.3.2节算法）
        :return: list of dict，无冲突的连通图集合
        """
        # 步骤1：初始化待处理队列
        self.pending_queue.extend(self.graph_constructor.graphs)
        
        # 步骤2：处理队列直至为空（FIFO策略）
        while self.pending_queue:
            current_graph = self.pending_queue.popleft()
            
            # 检查是否存在冲突
            if not self._has_conflict(current_graph):
                # 无冲突：直接加入无冲突集合
                self.conflict_free_set.append(current_graph)
                continue
            
            # 有冲突：执行贪心算法提取最优无冲突子图
            optimal_subgraph = self._extract_optimal_subgraph(current_graph)
            self.conflict_free_set.append(optimal_subgraph)
            
            # 步骤3：重建剩余节点的连通图，放回待处理队列
            remaining_nodes = current_graph["nodes"] - optimal_subgraph["nodes"]
            if remaining_nodes:
                # 从原始边字典中提取剩余节点的边
                remaining_edges = []
                for (n1, n2) in current_graph["edges"]:
                    if n1 in remaining_nodes and n2 in remaining_nodes:
                        remaining_edges.append((n1, n2))
                
                # 计算剩余节点的边权重和节点权重
                remaining_edge_weights = {
                    (n1, n2): current_graph["edge_weights"][(n1, n2)] 
                    for (n1, n2) in remaining_edges
                }
                remaining_node_weights = defaultdict(float)
                for node in remaining_nodes:
                    for (n1, n2), w in remaining_edge_weights.items():
                        if node == n1 or node == n2:
                            remaining_node_weights[node] += w
                
                # 构建新连通图并加入队列
                new_graph = {
                    "nodes": remaining_nodes,
                    "edges": remaining_edges,
                    "edge_weights": remaining_edge_weights,
                    "node_weights": remaining_node_weights
                }
                self.pending_queue.append(new_graph)
        
        return self.conflict_free_set
    
    def _has_conflict(self, graph):
        """判断连通图是否存在冲突（同2.2节）"""
        drone_targets = defaultdict(set)
        for (drone_id, target_id) in graph["nodes"]:
            drone_targets[drone_id].add(target_id)
        return any(len(targets) > 1 for targets in drone_targets.values())
    
    def _extract_optimal_subgraph(self, graph):
        """
        贪心算法提取最优无冲突子图（Algorithm 1）
        :param graph: 存在冲突的连通图
        :return: 无冲突的最优子图
        """
        # Step 1：选择节点权重最大的节点作为起始点
        max_node = max(graph["node_weights"].keys(), key=lambda x: graph["node_weights"][x])
        max_drone = max_node[0]  # 起始节点所属无人机
        
        # 初始化：已选节点/无人机集合R，已选边集合E*
        selected_nodes = set([max_node])
        selected_drones = set([max_drone])
        selected_edges = []
        
        # Step 2：按边权重降序遍历所有边，筛选无冲突边
        # 边按权重降序排序
        sorted_edges = sorted(
            graph["edges"],
            key=lambda x: graph["edge_weights"][x],
            reverse=True
        )
        
        for edge in sorted_edges:
            n1, n2 = edge
            d1, d2 = n1[0], n2[0]  # 两个节点所属的无人机
            
            # 若两个无人机均未被选中，则保留该边
            if d1 not in selected_drones and d2 not in selected_drones:
                selected_edges.append(edge)
                selected_nodes.add(n1)
                selected_nodes.add(n2)
                selected_drones.add(d1)
                selected_drones.add(d2)
        
        # Step 3：计算最优子图的节点权重和边权重
        subgraph_edge_weights = {
            edge: graph["edge_weights"][edge] for edge in selected_edges
        }
        subgraph_node_weights = defaultdict(float)
        for node in selected_nodes:
            for (n1, n2), w in subgraph_edge_weights.items():
                if node == n1 or node == n2:
                    subgraph_node_weights[node] += w
        
        # 返回最优无冲突子图
        return {
            "nodes": selected_nodes,
            "edges": selected_edges,
            "edge_weights": subgraph_edge_weights,
            "node_weights": subgraph_node_weights
        }
    
    def print_conflict_free_result(self):
        """打印无冲突结果"""
        print("=" * 50)
        print("Conflict-Free Connected Graphs Result")
        print("=" * 50)
        for idx, subgraph in enumerate(self.conflict_free_set, 1):
            print(f"\nSubgraph {idx}:")
            print(f"- Nodes (drone_id, target_id): {sorted(subgraph['nodes'])}")
            print(f"- Edges (with weight): {[(e, f'{w:.2f}') for e, w in subgraph['edge_weights'].items()]}")
            print(f"- Node Weights: {[(n, f'{w:.2f}') for n, w in subgraph['node_weights'].items()]}")
            print(f"- Conflict Check: {self._has_conflict(subgraph)}")
        print("=" * 50)


