import itertools
import pandas as pd

class CNF:
    def __init__(self):
        self.clauses = []

    def add_clause(self, clause):
        self.clauses.append(clause)

    def __str__(self):
        # Return the CNF in DIMACS format
        cnf_str = f"p cnf {self.count_variables()} {len(self.clauses)}\n"
        for clause in self.clauses:
            cnf_str += " ".join(map(str, clause)) + " 0\n"
        return cnf_str
    
    def get_metadata(self):
        meta_data = f"p cnf {self.count_variables()} {len(self.clauses)}\n"
        return meta_data

    def count_variables(self):
        # Get the count of unique variables in the CNF
        variables = set()
        for clause in self.clauses:
            variables.update(abs(literal) for literal in clause)
        return len(variables)


class CNFConstructor:
    def __init__(self, mdds, lazy_encoding=False, cnf = None, variable_map = None, next_variable_id = None):
        self.mdds = mdds
        
        self.cnf = CNF()
        if cnf is not None:
            for clause in cnf.clauses:
                self.cnf.add_clause(clause)
        
        self.variable_map = {}
        if variable_map is not None:
            for key, value in variable_map.items():
                self.variable_map[key] = value
        #self.variable_map2 = pd.DataFrame()
        if next_variable_id is None:
            self.next_variable_id = 1
        else:
            self.next_variable_id = next_variable_id
        self.lazy_encoding = lazy_encoding
        

    # 
    def add_variable(self, agent_id, node, timestep):
        # Add a new variable for the (agent, node, timestep) tuple
        key = (agent_id, node, timestep)
        if key not in self.variable_map:
            self.variable_map[key] = self.next_variable_id

            #self.variable_map2.loc[self.next_variable_id, 'agent_id'] = int(agent_id)
            #self.variable_map2.loc[self.next_variable_id, 'node_x'] = int(node[0])
            #self.variable_map2.loc[self.next_variable_id, 'node_y'] = int(node[1])
            #self.variable_map2.loc[self.next_variable_id, 'timestep'] = int(timestep)

            self.next_variable_id += 1
        return self.variable_map[key]

    def create_agent_path_clauses(self, agent_id, mdd):
        # For each level in the MDD, create clauses to ensure valid paths
        for timestep, nodes in mdd.levels.items():
            # Ensure the agent is at one of the possible nodes at this timestep
            valid_nodes_clause = [
                self.add_variable(agent_id, node.position, timestep)
                for node in nodes
            ]
            self.cnf.add_clause(valid_nodes_clause)

    def add_single_occupancy_clauses(self):
        #Add clauses to ensure each agent occupies only one position at each timestep
        for agent_id, mdd in self.mdds.items():
            for timestep in range(max(len(mdd.levels) for mdd in self.mdds.values())):
                nodes_at_timestep = [node for node in mdd.get_nodes_at_level(timestep)]
                if len(nodes_at_timestep) > 1:
                    # Create clauses that ensure only one of these nodes can be true at a time
                    for node1, node2 in itertools.combinations(nodes_at_timestep, 2):
                        clause = [
                            -self.add_variable(agent_id, node1.position, timestep),
                            -self.add_variable(agent_id, node2.position, timestep)
                        ]
                        self.cnf.add_clause(clause)

    #unlazy encoding, for lazy encoding just add the conflict clause for the pair of agents if needed
    def create_no_conflict_clauses(self):
        # Ensure no two agents are at the same node at the same timestep
        all_agent_ids = list(self.mdds.keys())
        for timestep in range(max(len(mdd.levels) for mdd in self.mdds.values())):
            # Generate all pairs of agents
            for agent1_id, agent2_id in itertools.combinations(all_agent_ids, 2):
                # Check which nodes they can be at in this timestep
                agent1_nodes = self.mdds[agent1_id].get_nodes_at_level(timestep)
                agent2_nodes = self.mdds[agent2_id].get_nodes_at_level(timestep)

                # Add clauses to ensure they don't occupy the same node
                for node1 in agent1_nodes:
                    for node2 in agent2_nodes:
                        if node1.position == node2.position:
                            # The negation ensures they don't both occupy the same node
                            conflict_clause = [
                                -self.add_variable(agent1_id, node1.position, timestep),
                                -self.add_variable(agent2_id, node2.position, timestep),
                            ]
                            self.cnf.add_clause(conflict_clause)

    def add_single_collision_clause(self, collision, adding_to_cnf = True):
        """
        Add a clause indicating that only one of two agents can be at a given position at a specific time.
        """
        agent1_id, agent2_id, position, timestep = collision
        variable1 = self.add_variable(agent1_id, position, timestep)
        variable2 = self.add_variable(agent2_id, position, timestep)

        # Negate the variables to ensure only one can occupy this position at this time
        clause = [-variable1, -variable2]
        
        if adding_to_cnf:
            self.cnf.add_clause(clause)
    
        return clause

    def add_transition_clauses(self):
        """
        For every agent and every timestep (except the last), enforce that if the agent is at a given node u at time t,
        then at time t+1 it must be at one of the children of u as defined in the MDD.
        """
        for agent_id, mdd in self.mdds.items():
            # For each timestep that has a subsequent timestep:
            for t in sorted(mdd.levels.keys()):
                if t + 1 not in mdd.levels:
                    continue
                # For every node at time t:
                for node in mdd.levels[t]:
                    if node.children:  # Only add if there is at least one valid move.
                        # Build a clause: ¬x(agent, u, t) ∨ (x(agent, child1, t+1) ∨ ... ∨ x(agent, child_k, t+1))
                        clause = [-self.add_variable(agent_id, node.position, t)]
                        for child in node.children:
                            clause.append(self.add_variable(agent_id, child.position, t+1))
                        self.cnf.add_clause(clause)
                    # If no children exist, the MDD would be incomplete—typically waiting is allowed.


    def construct_cnf(self, collisions = None):
        # Construct CNF from the given MDDs
        for agent_id, mdd in self.mdds.items():
            self.create_agent_path_clauses(agent_id, mdd)

        self.add_single_occupancy_clauses()

        if not self.lazy_encoding:
            self.create_no_conflict_clauses()
        self.add_transition_clauses()
        if collisions:
            for agent1_id, agent2_id, position, timestep in collisions:
                new_collison = self.add_single_collision_clause((agent1_id, agent2_id, position, timestep))

        return self.cnf