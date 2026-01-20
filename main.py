import numpy as np
from collections import deque, defaultdict
import matplotlib.pyplot as plt
import networkx as nx  
from CG import ConnectedGraphConstructor
from CRM import ConflictResolver
def generate_sample_pairwise_associations():
    """
    生成模拟两两关联结果
    返回格式：list of dict
    每个dict包含：drone_m(无人机m), target_m(无人机m的目标ID), 
                drone_n(无人机n), target_n(无人机n的目标ID), 
                cost(关联成本，越小相似度越高)
    """
    return [
        # 无人机1-2关联
        {"drone_m": 1, "target_m": 1, "drone_n": 2, "target_n": 1, "cost": 0.2},
        {"drone_m": 1, "target_m": 2, "drone_n": 2, "target_n": 2, "cost": 0.3},
        {"drone_m": 2, "target_m": 1, "drone_n": 3, "target_n": 1, "cost": 0.25},
        {"drone_m": 2, "target_m": 1, "drone_n": 3, "target_n": 2, "cost": 0.4},  
        {"drone_m": 2, "target_m": 2, "drone_n": 3, "target_n": 3, "cost": 0.35},
        {"drone_m": 1, "target_m": 1, "drone_n": 3, "target_n": 2, "cost": 0.28},  
        {"drone_m": 1, "target_m": 2, "drone_n": 3, "target_n": 3, "cost": 0.32}
    ]

def main():
    # 步骤1：生成模拟两两关联结果
    pairwise_assocs = generate_sample_pairwise_associations()
    print("Sample Pairwise Associations:")
    for assoc in pairwise_assocs:
        print(assoc)
    
    # 步骤2：执行2.2节连通图构建
    graph_builder = ConnectedGraphConstructor(epsilon=1e-6)
    graphs = graph_builder.build_graph(pairwise_assocs)
    print(f"\nBuilt {len(graphs)} connected graphs (2.2节完成)")
    
    # （可选）可视化构建的连通图
    graph_builder.visualize_graphs()
    
    # 步骤3：执行2.3节冲突解决
    resolver = ConflictResolver(graph_builder)
    conflict_free_graphs = resolver.resolve_conflicts()
    print(f"\nResolved to {len(conflict_free_graphs)} conflict-free graphs (2.3节完成)")
    
    # 打印最终无冲突结果
    resolver.print_conflict_free_result()

if __name__ == "__main__":
    main()