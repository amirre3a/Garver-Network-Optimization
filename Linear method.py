import cvxpy as cp
import numpy as np
import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt
from collections import defaultdict

# Define sets
bus = range(1, 7)
slack_bus = [1]
Gen = range(1, 7)
k_set = range(1, 5)

# Scalar definitions
Sbase = 100
M = 1000

# Define data
GenData = {
    1: {'b': 0, 'pmin': 0, 'pmax': 50},
    2: {'b': 0, 'pmin': 0, 'pmax': 0},
    3: {'b': 0, 'pmin': 0, 'pmax': 165},
    4: {'b': 0, 'pmin': 0, 'pmax': 0},
    5: {'b': 0, 'pmin': 0, 'pmax': 0},
    6: {'b': 0, 'pmin': 0, 'pmax': 545}
}

GBconect = {
    (1, 1): 1,
    (3, 3): 1,
    (6, 6): 1
}

BusData = {
    1: {'Pd': 80},
    2: {'Pd': 240},
    3: {'Pd': 40},
    4: {'Pd': 160},
    5: {'Pd': 240},
    6: {'Pd': 0}
}

branch_data = [
    (1, 2, 0.4, 100, 40, 1),
    (1, 3, 0.38, 100, 38, 0),
    (1, 4, 0.6, 80, 60, 1),
    (1, 5, 0.2, 100, 20, 1),
    (1, 6, 0.68, 70, 68, 0),
    (2, 3, 0.2, 100, 20, 1),
    (2, 4, 0.4, 100, 40, 1),
    (2, 5, 0.31, 100, 31, 0),
    (2, 6, 0.3, 100, 30, 0),
    (3, 4, 0.59, 82, 59, 0),
    (3, 5, 0.2, 100, 20, 1),
    (3, 6, 0.48, 100, 48, 0),
    (4, 5, 0.63, 75, 63, 0),
    (4, 6, 0.3, 100, 30, 0),
    (5, 6, 0.61, 78, 61, 0)
]

conex = {(b, n): 1 for b, n, x, lim, cost, stat in branch_data}
conex.update({(n, b): 1 for b, n, x, lim, cost, stat in branch_data})

branch = {(b, n): {'x': x, 'Limit': lim, 'Cost': cost, 'Stat': stat} for b, n, x, lim, cost, stat in branch_data}

# Create a list to store reverse branches to be added later
reverse_branches = []
for (b, n), v in branch.items():
    if (n, b) not in branch:
        reverse_branches.append((n, b, v))

# Update the branch dictionary with reverse branches
for n, b, v in reverse_branches:
    branch[(n, b)] = {'x': v['x'], 'Limit': v['Limit'], 'Cost': v['Cost'], 'Stat': v['Stat']}

bij = {(b, n): 1 / v['x'] for (b, n), v in branch.items()}

# Define variables
Pg = {g: cp.Variable() for g in Gen}
Pij = {(b, n, k): cp.Variable() for b in bus for n in bus for k in k_set if (b, n) in conex}
delta = {b: cp.Variable() for b in bus}
LS = {b: cp.Variable() for b in bus}
alpha = {(b, n, k): cp.Variable(boolean=True) for b in bus for n in bus for k in k_set if (b, n) in conex}

# Fixed variables
Pg[1] = 50 / Sbase
Pg[3] = 165 / Sbase
Pg[6] = 545 / Sbase
delta[1] = 0

# Objective function
objective = cp.Minimize(
    sum(Pg[g] * GenData[g]['b'] * Sbase for g in Gen if isinstance(Pg[g], cp.Variable)) + 
    sum(LS[b] for b in bus) + 
    sum(0.5 * branch[(b, n)]['Cost'] * alpha[b, n, k] for b in bus for n in bus for k in k_set if (b, n) in branch and (k > 1 or branch[(b, n)]['Stat'] == 0))
)

# Constraints
constraints = []

for b in bus:
    for n in bus:
        for k in k_set:
            if (n, b) in conex:
                constraints.append(Pij[b, n, k] - bij[(b, n)] * (delta[b] - delta[n]) <= M * (1 - alpha[b, n, k]))
                constraints.append(Pij[b, n, k] - bij[(b, n)] * (delta[b] - delta[n]) >= -M * (1 - alpha[b, n, k]))
                constraints.append(Pij[b, n, k] <= alpha[b, n, k] * branch[(b, n)]['Limit'] / Sbase)
                constraints.append(Pij[b, n, k] >= -alpha[b, n, k] * branch[(b, n)]['Limit'] / Sbase)
                constraints.append(alpha[b, n, k] == alpha[n, b, k])

for b in bus:
    constraints.append(
        LS[b] + sum(Pg[g] for g in Gen if (b, g) in GBconect) - BusData[b]['Pd'] / Sbase == 
        sum(Pij[b, n, k] for k in k_set for n in bus if (n, b) in conex)
    )

# Additional constraints
for b in bus:
    constraints.append(LS[b] >= 0)
    constraints.append(LS[b] <= BusData[b]['Pd'] / Sbase)

for g in Gen:
    if isinstance(Pg[g], cp.Variable):
        constraints.append(Pg[g] >= GenData[g]['pmin'] / Sbase)
        constraints.append(Pg[g] <= GenData[g]['pmax'] / Sbase)

for b in bus:
    for n in bus:
        for k in k_set:
            if (b, n) in conex:
                constraints.append(Pij[b, n, k] <= branch[(b, n)]['Limit'] / Sbase)
                constraints.append(Pij[b, n, k] >= -branch[(b, n)]['Limit'] / Sbase)

# Solve the problem
prob = cp.Problem(objective, constraints)
prob.solve()

# Collect results
results = {
    "Objective Function Value": [prob.value],
    "Pg": {g: Pg[g].value * Sbase if isinstance(Pg[g], cp.Variable) else Pg[g] * Sbase for g in Gen},
    "delta": {b: delta[b].value if isinstance(delta[b], cp.Variable) else delta[b] for b in bus},
    "LS": {b: LS[b].value * Sbase for b in bus},
    "Pij": {(b, n, k): Pij[b, n, k].value * Sbase for b in bus for n in bus for k in k_set if (b, n) in conex},
    "alpha": {(b, n, k): alpha[b, n, k].value for b in bus for n in bus for k in k_set if (b, n) in conex}
}

# Convert results to DataFrame
objective_df = pd.DataFrame(results["Objective Function Value"], columns=["Objective Function Value"])
Pg_df = pd.DataFrame(list(results["Pg"].items()), columns=["Generator", "Pg"])
delta_df = pd.DataFrame(list(results["delta"].items()), columns=["Bus", "Delta"])
LS_df = pd.DataFrame(list(results["LS"].items()), columns=["Bus", "LS"])
Pij_df = pd.DataFrame([(b, n, k, v) for (b, n, k), v in results["Pij"].items()], columns=["Bus", "Node", "K", "Pij"])
alpha_df = pd.DataFrame([(b, n, k, v) for (b, n, k), v in results["alpha"].items()], columns=["Bus", "Node", "K", "Alpha"])

# Display results
print("Objective Function Value:", f"{prob.value:.4f}")
for g in Gen:
    print(f"Pg[{g}]:", f"{results['Pg'][g]:.4f} MW")
for b in bus:
    print(f"delta[{b}]:", f"{results['delta'][b]:.4f}")
    print(f"LS[{b}]:", f"{results['LS'][b]:.4f} MW")
for (b, n, k), value in results["Pij"].items():
    print(f"Pij[{b}, {n}, {k}]:", f"{value:.4f} MW")
for (b, n, k), value in results["alpha"].items():
    print(f"alpha[{b}, {n, k}]:", f"{value:.4f}")

# Save results to Excel
with pd.ExcelWriter("optimization_results.xlsx") as writer:
    objective_df.to_excel(writer, sheet_name="Objective Function Value", index=False)
    Pg_df.to_excel(writer, sheet_name="Pg", index=False)
    delta_df.to_excel(writer, sheet_name="Delta", index=False)
    LS_df.to_excel(writer, sheet_name="LS", index=False)
    Pij_df.to_excel(writer, sheet_name="Pij", index=False)
    alpha_df.to_excel(writer, sheet_name="Alpha", index=False)

print("Results saved to optimization_results.xlsx")

# Aggregate the power flows and count multiple edges
edge_capacities = defaultdict(list)
for (b, n, k), value in results["Pij"].items():
    if value != 0:
        edge_capacities[(b, n)].append(abs(value))

# Create the network graph
G = nx.DiGraph()

# Add nodes (buses)
for b in bus:
    G.add_node(b)

# Add edges (connections) with aggregated capacities and multiple edges
for (b, n), capacities in edge_capacities.items():
    total_capacity = sum(capacities)
    edge_label = f'{total_capacity:.4f} MW\nn={len(capacities)}'
    G.add_edge(b, n, capacity=total_capacity, label=edge_label)

# Get edge labels for capacities
edge_labels = {(b, n): data['label'] for b, n, data in G.edges(data=True)}

# Draw the graph
pos = nx.spring_layout(G)  # Positions for all nodes
plt.figure(figsize=(12, 10))
nx.draw(G, pos, with_labels=True, node_size=700, node_color='skyblue', font_size=12, font_weight='bold', edge_color='gray')

# Draw edge labels manually for multi-edges
for (b, n), label in edge_labels.items():
    x = (pos[b][0] + pos[n][0]) / 2
    y = (pos[b][1] + pos[n][1]) / 2
    plt.text(x, y, s=label, fontdict=dict(color='red', size=10), bbox=dict(facecolor='white', edgecolor='none', alpha=0.7))

plt.title('Optimized Bus Network with Aggregated Line Capacities')
plt.show()
