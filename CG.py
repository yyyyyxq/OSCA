import numpy as np
from collections import deque, defaultdict
import matplotlib.pyplot as plt
import networkx as nx  

class ConnectedGraphConstructor:
    def __init__(self, epsilon=1e-6):
        self.epsilon = epsilon  # 防止成本为0的参数（文章2.2节提到）
        self.graphs = []  # 最终生成的连通图集合（G_set）
    
    def build_graph(self, pairwise_associations):
        """
        从两两关联结果构建连通图
        :param pairwise_associations: 两两关联结果（generate_sample_pairwise_associations的输出）
        :return: list of dict，每个dict代表一个连通图（含nodes, edges, edge_weights, node_weights）
        """
        # 步骤1：初始化图元素存储
        all_nodes = set()  # 所有目标节点（格式：(drone_id, target_id)）
        edge_dict = defaultdict(dict)  # 边存储：edge_dict[(u1,t1)][(u2,t2)] = weight
        
        # 步骤2：处理每个关联对，计算边权重
        for assoc in pairwise_associations:
            # 提取关联对信息
            node1 = (assoc["drone_m"], assoc["target_m"])
            node2 = (assoc["drone_n"], assoc["target_n"])
            cost = assoc["cost"]
            
            # 计算边权重（成本倒数，加epsilon防止除以0）
            weight = 1.0 / (cost + self.epsilon)
            
            # 双向存储边（无向图）
            edge_dict[node1][node2] = weight
            edge_dict[node2][node1] = weight
            all_nodes.add(node1)
            all_nodes.add(node2)
        
        # 步骤3：查找连通分量（构建多个连通图）
        visited = set()
        for node in all_nodes:
            if node not in visited:
                # BFS查找连通分量
                q = deque([node])
                visited.add(node)
                component_nodes = set([node])
                
                while q:
                    current_node = q.popleft()
                    for neighbor in edge_dict.get(current_node, {}):
                        if neighbor not in visited:
                            visited.add(neighbor)
                            component_nodes.add(neighbor)
                            q.append(neighbor)
                
                # 步骤4：计算当前连通图的边权重和节点权重
                component_edges = []
                component_edge_weights = {}
                component_node_weights = defaultdict(float)
                
                # 收集当前分量的边和边权重
                for n1 in component_nodes:
                    for n2, w in edge_dict.get(n1, {}).items():
                        if n1 < n2:  # 避免重复存储无向边
                            component_edges.append((n1, n2))
                            component_edge_weights[(n1, n2)] = w
                        # 累加节点权重（所有关联边的权重和）
                        component_node_weights[n1] += w
                
                # 步骤5：保存当前连通图
                self.graphs.append({
                    "nodes": component_nodes,
                    "edges": component_edges,
                    "edge_weights": component_edge_weights,
                    "node_weights": component_node_weights
                })
        
        return self.graphs
    
    def visualize_graphs(self):
        """可视化所有连通图（可选，需安装networkx和matplotlib）"""
        for idx, graph in enumerate(self.graphs):
            G = nx.Graph()
            # 添加节点和边
            G.add_nodes_from(graph["nodes"])
            G.add_edges_from(graph["edges"])
            
            # 设置节点大小（与节点权重正相关）
            node_sizes = [graph["node_weights"][node] * 1000 for node in G.nodes()]
            
            # 绘制图
            plt.figure(figsize=(8, 6))
            pos = nx.spring_layout(G, seed=42)  # 固定布局
            nx.draw(G, pos, with_labels=True, node_size=node_sizes, 
                    node_color="lightblue", font_size=10, font_weight="bold")
            
            # 添加边权重标签
            edge_labels = {edge: f"{graph['edge_weights'][edge]:.2f}" for edge in graph["edges"]}
            nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=8)
            
            plt.title(f"Connected Graph {idx+1} (Conflict Check: {self._has_conflict(graph)})")
            plt.show()
    
    def _has_conflict(self, graph):
        """辅助函数：判断单个连通图是否存在冲突（2.3节冲突定义）"""
        drone_targets = defaultdict(set)
        for (drone_id, target_id) in graph["nodes"]:
            drone_targets[drone_id].add(target_id)
        # 若任一无人机有多个目标节点，则存在冲突
        return any(len(targets) > 1 for targets in drone_targets.values())
   