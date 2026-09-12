import subprocess
from mdd import MDDConstructor
from cnf_edges import CNFConstructor
import os
import subprocess
import pandas as pd
import numpy as np
import time
import hashlib
import re
import random
from pysat.formula import CNF
from pysat.solvers import Glucose4, Glucose3

class SATSolverMangerEdges:
    def __init__(self, map_path, scenario_path, probsat_executable, 
                 cnf_file_path, num_agents, base_max_flips=1000, 
                 base_max_tries=100, seed=42, output_dir='Data', save_name = '',
                 max_timesteps_threshold=100, time_results_file=None, 
                 use_flip_heuristic=False, amount_of_start_goal_sets=None,
                 map_name = ''):
        """
        Initialize the SATSolverManager with parameters.

        Parameters:
        - map_path (str): Path to the map file.
        - scenario_path (str): Path to the scenario file.
        - probsat_executable (str): Path to the probSAT executable.
        - cnf_file_path (str): Path to the CNF file.
        - num_agents (int): Number of agents.
        - base_max_flips (int): Base number of maximum flips for probSAT.
        - base_max_tries (int): Base number of maximum tries for probSAT.
        - seed (int): Seed for random number generation.
        - output_dir (str): Directory to store output files.
        - max_timesteps_threshold (int): Maximum number of timesteps.
        - time_results_file (str): File to save timing results.
        - use_flip_heuristic (bool): Whether to use heuristic-based max flips and tries.
        - amount_of_start_goal_sets (int or None): Number of start/goal sets to test, or None to use all.
        """
        self.map = self.load_map(map_path)
        self.map_name = map_name
        self.map_size = [len(self.map), len(self.map[0])]
        self.starts_and_goals = self.load_starts_and_goals_from_file(scenario_path, num_agents)
        if amount_of_start_goal_sets is not None:
            self.starts_and_goals = self.starts_and_goals[:amount_of_start_goal_sets]
        self.probsat_executable = probsat_executable
        self.cnf_file_path = cnf_file_path
        self.num_agents = num_agents
        self.base_max_flips = base_max_flips
        self.base_max_tries = base_max_tries
        self.seed = seed
        self.output_dir = output_dir
        self.save_name = save_name
        self.max_timesteps_threshold = max_timesteps_threshold
        self.time_results_file = time_results_file
        self.use_flip_heuristic = use_flip_heuristic
        os.makedirs(output_dir, exist_ok=True)
        self.parameters = {
            'map': self.map,
            'starts_and_goals': self.starts_and_goals,
            'probsat_executable': probsat_executable,
            'cnf_file_path': cnf_file_path,
            'num_agents': num_agents,
            'base_max_flips': base_max_flips,
            'base_max_tries': base_max_tries,
            'seed': seed,
            'output_dir': output_dir,
            'time_results_file': time_results_file,
            'use_flip_heuristic': use_flip_heuristic,
            'amount_of_start_goal_sets': amount_of_start_goal_sets
        }
        self.cnf_cache = {}
        self.initial_max_timesteps = 0
        self.initial_paths = None
        self.added_vertex_collisions = []
        self.added_edge_collisions = []

    @staticmethod
    def load_map(map_path):
        """
        Load the map from a file.

        Parameters:
        - map_path (str): Path to the map file.

        Returns:
        - map (list of list of str): 2D list representation of the map.
        """
        map_data = np.loadtxt(map_path, dtype='str', skiprows=4)
        
        map_2d = np.array([list(row) for row in map_data]).T
    

        #if the map contains 'S' or 'W' characters issue a warning that swamp and water will be handled as unpassable
        if 'S' in map_2d or 'W' in map_2d:
            print('Warning: Swamp and Water will be handled as unpassable terrain')
        return map_2d

    @staticmethod
    def create_dataframe_from_file(file_path):
        """
        Create a pandas DataFrame from a scenario file.

        Parameters:
        - file_path (str): Path to the scenario file.

        Returns:
        - df (pandas.DataFrame): DataFrame containing the scenario data.
        """
        with open(file_path, 'r') as file:
            lines = file.readlines()
        
        if lines[0].startswith('version'):
            lines = lines[1:]
        
        columns = [
            'bucket', 'map_name', 'map_width', 'map_height', 
            'start_x', 'start_y', 'goal_x', 'goal_y', 'optimal_length'
        ]
        
        data = [line.strip().split('\t') for line in lines]
        
        df = pd.DataFrame(data, columns=columns)
        
        numeric_columns = ['bucket', 'map_width', 'map_height', 'start_x', 'start_y', 'goal_x', 'goal_y', 'optimal_length']
        df[numeric_columns] = df[numeric_columns].apply(pd.to_numeric)
        
        return df

    @staticmethod
    def create_starts_and_goals(df, num_agents):
        """
        Create sets of starts and goals from the DataFrame.

        Parameters:
        - df (pandas.DataFrame): DataFrame containing the scenario data.
        - num_agents (int): Number of agents.

        Returns:
        - starts_and_goals (list of dict): List of dictionaries with 'starts' and 'goals'.
        """
        num_sets = len(df) // num_agents if len(df) % num_agents == 0 else len(df) // num_agents + 1
        starts_and_goals = []
        
        for i in range(num_sets):
            subset_df = df.iloc[i*num_agents:(i+1)*num_agents]
            
            if len(subset_df) == 0:
                continue
            
            starts = subset_df[['start_x', 'start_y']].values.tolist()
            goals = subset_df[['goal_x', 'goal_y']].values.tolist()
            
            starts_and_goals.append({'starts': starts, 'goals': goals})
        
        return starts_and_goals

    def load_starts_and_goals_from_file(self, scenario_path, num_agents):
        """
        Load starts and goals from a scenario file.

        Parameters:
        - scenario_path (str): Path to the scenario file.
        - num_agents (int): Number of agents.

        Returns:
        - starts_and_goals (list of dict): List of dictionaries with 'starts' and 'goals'.
        """
        scenario_df = self.create_dataframe_from_file(scenario_path)
        return self.create_starts_and_goals(scenario_df, num_agents)

    def compute_max_timesteps(self, starts, goals):
        """
        Compute the maximum number of timesteps required for the MDDs.

        Parameters:
        - starts (list of list of int): List of start positions.
        - goals (list of list of int): List of goal positions.

        Returns:
        - distance_matrices (list of dict): List of distance matrices for each agent.
        - max_timesteps (int): Maximum number of timesteps required.
        """
        distance_matrices = []
        goal_distances = []

        for start, goal in zip(starts, goals):
            start_tuple = tuple(start)  # Convert to tuple
            goal_tuple = tuple(goal)  # Convert to tuple
            constructor = MDDConstructor(self.map, start_tuple, goal_tuple, max_timesteps=None)
            distances = constructor.compute_all_distances()
            distance_matrices.append(distances)
            goal_distances.append(distances.get(start_tuple, 0))
            
        max_timesteps = max(goal_distances)

        return distance_matrices, max_timesteps

    def create_mdds(self, starts, goals, max_timesteps, distance_matrices):
        """
        Create MDDs for each agent.

        Parameters:
        - starts (list of list of int): List of start positions.
        - goals (list of list of int): List of goal positions.
        - max_timesteps (int): Maximum number of timesteps.
        - distance_matrices (list of dict): List of distance matrices for each agent.

        Returns:
        - mdds (dict): Dictionary of MDDs for each agent.
        - constructors (dict): Dictionary of MDD constructors for each agent.
        """
        mdds = {}
        constructors = {}

        for agent_id, (start, goal) in enumerate(zip(starts, goals)):
            start_tuple = tuple(start)  # Convert to tuple
            goal_tuple = tuple(goal)  # Convert to tuple
            distances = distance_matrices[agent_id]
            mdd_constructor = MDDConstructor(self.map, start_tuple, goal_tuple, max_timesteps, distances=distances)
            mdd = mdd_constructor.construct_mdd()
            mdds[agent_id] = mdd
            constructors[agent_id] = mdd_constructor

        return mdds, constructors
    
    def find_mdd_node(self, mdd, position, time_step):
        """
        Helper to find the MDDNode for (position, time_step) in mdd.
        Returns the node or None if not found.
        """

        for node in mdd.levels[time_step]:
            if node.position == position:
                return node
        print('node not found')
        return None
    
    def prune_edge_link_only(self, mdd, start_pos, end_pos, time_step):
        """
        Remove the single forward link from (start_pos, time_step) 
        to (end_pos, time_step+1). Does NOT remove the child node itself.
        """
        parent_node = self.find_mdd_node(mdd, start_pos, time_step)
        if not parent_node:
            print('couldnt find',parent_node)
            return
        
        child_node = self.find_mdd_node(mdd, end_pos, time_step+1)
        if child_node in parent_node.children:
            parent_node.children.remove(child_node)
        else:   
            print('couldnt find',child_node,'as child of', parent_node)

    def copy_all_mdds(mdds):
        """
        mdds is a dict of {agent_id: MDD}.
        Returns a new dict with {agent_id: MDD.copy()} so you don't mutate the original.
        """
        new_dict = {}
        for agent_id, original_mdd in mdds.items():
            new_dict[agent_id] = original_mdd.copy()
        return new_dict

    def prune_mdds(self, mdds, vertex_collisions, edge_collisions, make_copy=True, protected_agents=[]):
        """
        Prunes collisions from the MDDs:
        - vertex_collisions: list of (agentA, agentB, (vx, vy), t)
        - edge_collisions:   list of (agentA, agentB, (startPos, endPos), t)
        
        We randomly pick which agent's MDD to prune each time. 
        Vertex collisions => remove node (vx, vy, t) from that agent's MDD.
        Edge collisions   => remove forward link (start->end) from that agent's MDD.
        
        :param mdds: dict[agent_id -> MDD] (original MDDs)
        :param make_copy: if True, we'll copy each MDD before pruning, so the original remains untouched.
                        if False, we prune in-place.
        :return: a dict of {agent_id -> pruned MDD}
        """
        print('protecting agents', protected_agents)

        # 1) Optionally copy all MDDs so we don't mutate the originals
        if make_copy:
            pruned_mdds = {}
            for agent_id, original_mdd in mdds.items():
                pruned_mdds[agent_id] = original_mdd.copy()  # uses MDD.copy() method
        else:
            pruned_mdds = mdds  # in-place modification
        
        pruned_nodes = None
        pruned_nodes = set()
        # 2) Prune vertex collisions
        for (agentA, agentB, (vx, vy), t) in vertex_collisions:
            if (agentA, (vx, vy), t) in pruned_nodes or (agentB, (vx, vy), t) in pruned_nodes:
                continue
            if len(pruned_mdds[agentA].levels.get(t, [])) == 1:
                chosen_agent = agentB
            elif len(pruned_mdds[agentB].levels.get(t, [])) == 1:
                chosen_agent = agentA
            elif agentA in protected_agents:
                chosen_agent = agentB
            elif agentB in protected_agents:
                chosen_agent = agentA
            else:
                chosen_agent = random.choice([agentA, agentB])
            node = self.find_mdd_node(pruned_mdds[chosen_agent], (vx, vy), t)
            pruned_nodes.add((chosen_agent, (vx, vy), t))
            if node:
                pruned_mdds[chosen_agent].remove_node(node)
            

        pruned_edges = None
        pruned_edges = set()
        # 3) Prune edge collisions
        for (agentA, agentB, (startPos, endPos), t) in edge_collisions:
            #force choice if one agents node only has 1 child

            if (agentA, startPos, t) in pruned_nodes or (agentA, endPos, t+1) in pruned_nodes or (agentB, endPos, t) in pruned_nodes or (agentB, startPos, t+1) in pruned_nodes:
                continue

            if (agentA, startPos, endPos, t) in pruned_edges or (agentB, endPos, startPos, t) in pruned_edges:
                continue
            if self.find_mdd_node(pruned_mdds[agentA], startPos, t).children == 1:
                chosen_agent = agentB
            elif self.find_mdd_node(pruned_mdds[agentB], endPos, t).children == 1:
                chosen_agent = agentA
            elif agentA in protected_agents:
                chosen_agent = agentB
            elif agentB in protected_agents:
                chosen_agent = agentA
            else:#otherwise randomly choose
                chosen_agent = random.choice([agentA, agentB])
            # If chosen_agent == agentA => remove link (startPos, t) -> (endPos, t+1)
            # If chosen_agent == agentB => remove link (endPos, t) -> (startPos, t+1) (the reverse)
            if chosen_agent == agentA:
                self.prune_edge_link_only(pruned_mdds[agentA], startPos, endPos, t)
                pruned_edges.add((agentA, startPos, endPos, t))
            else:
                self.prune_edge_link_only(pruned_mdds[agentB], endPos, startPos, t)
                pruned_edges.add((agentB, endPos, startPos, t))

        return pruned_mdds
    
    def prune_mdds_all_conflictors(self, mdds, vertex_collisions, edge_collisions, make_copy=True):
        """
        Prunes collisions from the MDDs:
        - vertex_collisions: list of (agentA, agentB, (vx, vy), t)
        - edge_collisions:   list of (agentA, agentB, (startPos, endPos), t)
        
        We randomly pick which agent's MDD to prune each time. 
        Vertex collisions => remove node (vx, vy, t) from that agent's MDD.
        Edge collisions   => remove forward link (start->end) from that agent's MDD.
        
        :param mdds: dict[agent_id -> MDD] (original MDDs)
        :param make_copy: if True, we'll copy each MDD before pruning, so the original remains untouched.
                        if False, we prune in-place.
        :return: a dict of {agent_id -> pruned MDD}
        """

        # 1) Optionally copy all MDDs so we don't mutate the originals
        if make_copy:
            pruned_mdds = {}
            for agent_id, original_mdd in mdds.items():
                pruned_mdds[agent_id] = original_mdd.copy()  # uses MDD.copy() method
        else:
            pruned_mdds = mdds  # in-place modification
        
        pruned_nodes = set()
        # 2) Prune vertex collisions
        for (agentA, agentB, (vx, vy), t) in vertex_collisions:
            print('pruning')
            
            if not (agentA, (vx, vy), t) in pruned_nodes:
                node = self.find_mdd_node(pruned_mdds[agentA], (vx, vy), t)
                pruned_nodes.add((agentA, (vx, vy), t))
                if node:
                    pruned_mdds[agentA].remove_node(node)
                    print('pruned agent', agentB,'pos', vx, vy, 't',t)

            if not (agentB, (vx, vy), t) in pruned_nodes:
                node = self.find_mdd_node(pruned_mdds[agentB], (vx, vy), t)
                pruned_nodes.add((agentB, (vx, vy), t))
                if node:
                    pruned_mdds[agentB].remove_node(node)
                    print('pruned agent', agentB,'pos', vx, vy, 't',t)

        pruned_edges = set()
        # 3) Prune edge collisions
        for (agentA, agentB, (startPos, endPos), t) in edge_collisions:
            #force choice if one agents node only has 1 child
            print('pruning')
            print('agents', agentA, agentB, 'start', startPos, endPos, 't',t)

            if (agentA, startPos, t) in pruned_nodes or (agentA, endPos, t+1) in pruned_nodes or (agentB, endPos, t) in pruned_nodes or (agentB, startPos, t+1) in pruned_nodes:
                print('already pruned node')
                continue
            if (agentA, startPos, endPos, t) in pruned_edges or (agentB, endPos, startPos, t):
                print('areadyy pruned edge')
                continue
            if self.find_mdd_node(pruned_mdds[agentA], startPos, t).children == 1:
                chosen_agent = agentB
            elif self.find_mdd_node(pruned_mdds[agentB], endPos, t).children == 1:
                chosen_agent = agentA
            else:#otherwise randomly choose
                chosen_agents = [agentA, agentB]
            for chosen_agent in chosen_agents:
                if chosen_agent == agentA:
                    self.prune_edge_link_only(pruned_mdds[agentA], startPos, endPos, t)
                    pruned_edges.add((agentA, startPos, endPos, t))
                else:
                    self.prune_edge_link_only(pruned_mdds[agentB], endPos, startPos, t)
                    pruned_edges.add((agentB, endPos, startPos, t))

        return pruned_mdds

    def mdd_is_broken(self, mdd, max_timesteps):
        """
        Return True if this MDD fails to provide a path from layer 0 to layer max_timesteps.

        We do a BFS/propagation:
        - Let reachable[t] be the set of nodes reachable at time t
        - Start with reachable[0] = all nodes in mdd.levels[0]
        - For t in [0 .. max_timesteps-1]:
            build reachable[t+1] from children of reachable[t]
            (only those whose time_step == t+1)
            if at any point reachable[t+1] is empty => broken
        - Finally, if reachable[max_timesteps] is non-empty => not broken, else broken

        If you specifically require that time=0 has at least one node,
        we also check that initially.
        """

        if 0 not in mdd.levels or not mdd.levels[0]:
            # No starting nodes at time 0 => obviously broken
            return True

        # reachable[t] = set of nodes that are reachable at time t
        reachable = dict()
        reachable[0] = set(mdd.levels[0])  # all nodes in layer 0 are “reachable” initially

        for t in range(max_timesteps):
            if len(reachable.get(t, [])) == 0:
                # Means no node was reachable at layer t => broken
                return True

            # Build reachable[t+1] from children of nodes in reachable[t]
            next_layer = set()
            for node in reachable[t]:
                # Add all children whose time_step == t+1
                for child in node.children:
                    if child.time_step == t + 1:
                        next_layer.add(child)

            # if next_layer is empty, we can't proceed => broken
            if not next_layer:
                return True
            reachable[t + 1] = next_layer

        # After the loop, if reachable[max_timesteps] is empty => broken
        # Otherwise, we have at least one node at layer max_timesteps => not broken
        return (len(reachable.get(max_timesteps, [])) == 0)

    def create_and_save_cnf(self, mdds, filename = None, lazy_encoding=False):
        """
        Create and save the CNF for the MDDs.

        Parameters:
        - mdds (dict): Dictionary of MDDs for each agent.
        - lazy_encoding (bool): Whether to use lazy encoding.

        Returns:
        - cnf (CNF): CNF object representing the problem.
        - filename (str): Path to the saved CNF file.
        - cnf_constructor (CNFConstructor): CNFConstructor object.
        """
        cnf_constructor = CNFConstructor(mdds, lazy_encoding=lazy_encoding)
        cnf_construction_start_time = time.process_time()
        cnf = cnf_constructor.construct_cnf()
        cnf_text = str(cnf)
        cnf_construction_end_time = time.process_time()
        cnf_construction_time = cnf_construction_end_time - cnf_construction_start_time

        if filename is None:
            filename = self.get_unique_filename(f"{self.output_dir}/cnf_base_{lazy_encoding}_seed_{self.seed}.cnf")
        else:
            filename = self.get_unique_filename(f"{self.output_dir}/{filename}_base_cnf.cnf")
        with open(filename, 'w') as file:
            file.write(cnf_text)

        return cnf, filename, cnf_constructor, cnf_construction_time

    def calculate_max_flips_and_tries(self, cnf):
        """
        Calculate max flips and max tries based on the CNF size.

        Parameters:
        - cnf (CNF): CNF object representing the problem.

        Returns:
        - max_flips (int): Calculated maximum number of flips.
        - max_tries (int): Calculated maximum number of tries.
        """
        num_vars = len(cnf.variables)
        num_clauses = len(cnf.clauses)
        
        max_flips = int(self.base_max_flips * (num_vars / 100))
        max_tries = int(self.base_max_tries * (num_clauses / 50))
        
        return max_flips, max_tries

    def run_prosat_solver_with_optional_initial_assignment(self, cnf, cnf_file_path, init_assignment_path=None, save_output_path = None):
        """
        Run the probSAT solver with an optional initial assignment.

        Parameters:
        - cnf (CNF): CNF object representing the problem.
        - cnf_file_path (str): Path to the CNF file.
        - init_assignment_path (str): Path to the initial assignments file (optional).

        Returns:
        - stdout (str): Standard output from the probSAT solver.
        - stderr (str): Standard error from the probSAT solver.
        - elapsed_time (float): Time taken to run the solver.
        - max_flips (int): Maximum number of flips used.
        - max_tries (int): Maximum number of tries used.
        """
        if self.use_flip_heuristic:
            max_flips, max_tries = self.calculate_max_flips_and_tries(cnf)
        else:
            max_flips, max_tries = self.base_max_flips, self.base_max_tries

        command = [
            self.probsat_executable,
            '--runs', str(max_tries),
            '--maxflips', str(max_flips),
            '-a', 
            cnf_file_path,
            str(self.seed)
        ]

        if init_assignment_path is not None:
            command.insert(-2, '-i')
            command.insert(-2, init_assignment_path)

        try:
            start_time = time.process_time() 
            result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            end_time = time.process_time()
            

            if save_output_path is not None:
                #output_filename = self.get_unique_filename(f"{self.output_dir}/probsat_output_lazy_{'init' if init_assignment_path else 'no_init'}_max_flips_{max_flips}_max_tries_{max_tries}_seed_{self.seed}.txt")
                output_filename = self.get_unique_filename(self.output_dir+"/probSAT_output_"+save_output_path+".txt")
                with open(output_filename, 'w') as file:
                    file.write(result.stdout)
                    file.write(result.stderr)

            return result.stdout, result.stderr, end_time - start_time, max_flips, max_tries

        except Exception as e:
            print(f"Error executing probSAT: {e}")
            return None, str(e), None, max_flips, max_tries


    def extract_statistics(self, output):
        """
        Extract statistics from the solver output.

        Parameters:
        - output (str): The output string from the solver.

        Returns:
        - stats (dict): Dictionary containing the extracted statistics.
        """
        stats = {}

        # Regex patterns for the required fields
        patterns = {
            'number_of_variables': r'c number of variables\s*:\s*(\d+)',
            'number_of_literals': r'c number of literals\s*:\s*(\d+)',
            'number_of_clauses': r'c number of clauses\s*:\s*(\d+)',
            'max_clause_length': r'c max\. clause length\s*:\s*(\d+)',
            'num_flips': r'c numFlips\s*:\s*(\d+)',
            'avg_flips_per_variable': r'c avg\. flips/variable\s*:\s*([\d.]+)',
            'avg_flips_per_clause': r'c avg\. flips/clause\s*:\s*([\d.]+)',
            'flips_per_sec': r'c flips/sec\s*:\s*([\d.]+|inf)',
            'cpu_time': r'c CPU Time\s*:\s*([\d.]+)'
        }

        # Extract each field using the corresponding pattern
        for key, pattern in patterns.items():
            match = re.search(pattern, output)
            if match:
                stats[key] = match.group(1)

        return stats


    def invert_variable_map(self, variable_map):
            return {v: k for k, v in variable_map.items()}

    def parse_solver_output(self, assignments, variable_map):
        inverted_variable_map = self.invert_variable_map(variable_map)
        # Filter keys to only include vertex variables (length == 3)
        vertex_keys = [key for key in variable_map.keys() if len(key) == 3]
        if vertex_keys:
            max_agent_id = max(key[0] for key in vertex_keys)
        else:
            max_agent_id = 0
        agent_paths = [[] for _ in range(max_agent_id + 1)]
        true_vars = [var for var in assignments if var > 0]
        for var in true_vars:
            if var in inverted_variable_map:
                value = inverted_variable_map[var]
                # Process only if it's a vertex variable
                if len(value) == 3:
                    agent_id, node, timestep = value
                    agent_paths[agent_id].append((timestep, node))
        for path in agent_paths:
            path.sort()
        return agent_paths
    
    def parse_solver_output_only_edges(self, assignments, variable_map):
        inverted_variable_map = self.invert_variable_map(variable_map)
        # Filter keys to only include vertex variables (length == 3)
        vertex_keys = [key for key in variable_map.keys() if len(key) == 3]
        if vertex_keys:
            max_agent_id = max(key[0] for key in vertex_keys)
        else:
            max_agent_id = 0
        agent_paths = [[] for _ in range(max_agent_id + 1)]
        true_vars = [var for var in assignments if var > 0]
        for var in true_vars:
            if var in inverted_variable_map:
                value = inverted_variable_map[var]
                # Process only if it's a edge variable
                if len(value) == 4:
                    agent_id, start, end, timestep = value
                    agent_paths[agent_id].append((timestep, start, end))

        for path in agent_paths:
            path.sort()
        return agent_paths
        
    def find_collisions(self, agent_paths):
        # Infer max_timesteps from the longest path
        max_timesteps = max(len(path) for path in agent_paths)
        map_size = [len(self.map), len(self.map[0])]
        # Initialize a 3D numpy array with dimensions (timesteps, map_height, map_width)
        occupancy_grid = np.zeros([max_timesteps, map_size[0], map_size[1]], dtype=int)
        agent_grid = np.empty([max_timesteps, map_size[0], map_size[1]], dtype=object)
        
        # Mark the positions of each agent in the 3D array
        for agent_id, path in enumerate(agent_paths):
            for timestep, (x, y) in path:
                occupancy_grid[timestep, x, y] += 1
                if agent_grid[timestep, x, y] is None:
                    agent_grid[timestep, x, y] = []
                agent_grid[timestep, x, y].append(agent_id)
        
        collisions = []

        # Check for collisions by looking for values greater than 1 in the occupancy grid
        collision_indices = np.argwhere(occupancy_grid > 1)
        for timestep, x, y in collision_indices:
            agents_involved = agent_grid[timestep, x, y]
            if len(agents_involved) > 1:
                # Create a collision for every distinct pair
                for i in range(len(agents_involved)):
                    for j in range(i+1, len(agents_involved)):
                        collisions.append((agents_involved[i],agents_involved[j],(x, y),timestep))
            elif len(agents_involved) == 1:
                print(f"Collision detected with only one agent (agent {agents_involved[0]}) at ({x}, {y}) at timestep {timestep}")
            else:
                print(f"Collision detected with no agents at ({x}, {y}) at timestep {timestep}")        

        return collisions
    
    def find_edge_collisions(self, assignments, variable_map):
        """
        Detect swap conflicts (edge conflicts) directly from the solver's edge variable assignments.
        
        Assumes:
        - Edge variables are stored in the variable_map with keys as (agent, start, end, time)
        - assignments is a list of integers (positive means true)
        
        Returns:
        A list of tuples of the form:
            ((agent1, start, end, time), (agent2, end, start, time))
        indicating a swap conflict between agent1 and agent2.
        """
        # Invert the variable map: index -> key
        inverted_map = {v: k for k, v in variable_map.items()}

        # Build a dictionary keyed by (time, start, end) that maps to a list of agents taking that edge.
        '''edge_dict = {}
        conflict_dict = {}
        swap_conflicts =[]
        for var in assignments:
            if var > 0 and var in inverted_map:
                key = inverted_map[var]
                # Check if this is an edge variable (expected to be a 4-tuple)
                if len(key) == 4:
                    agent, start, end, t = key
                    #make possible conflic entry
                    possible_conflict = (end, start, t)
                    conflict_dict[possible_conflict] = agent
                    if (start, end, t) in conflict_dict:
                        agent2 = conflict_dict[(start, end, t)]
                        swap_conflicts.append(agent, agent2, (start, end), t)
        '''
        #Version 2
        # Build a dictionary keyed by (time, start, end) that maps to a list of agents taking that edge.
        edge_dict = {}
        for var in assignments:
            if var > 0 and var in inverted_map:
                key = inverted_map[var]
                # Check if this is an edge variable (expected to be a 4-tuple)
                if len(key) == 4:
                    agent, start, end, t = key
                    edge_dict.setdefault((t, start, end), []).append(agent)
                    
        
        swap_conflicts = []
        # For each edge, check if the reverse edge exists.
        for (t, start, end), agents in edge_dict.items():
            reverse_key = (t, end, start)
            if (start, end) < (end, start): #no conflict duplicates: Only handle conflicts if this edge is "less" than its reverse
                if reverse_key in edge_dict:
                    for agent1 in agents:
                        for agent2 in edge_dict[reverse_key]:
                            if agent1 != agent2:
                                swap_conflicts.append((agent1, agent2, (start, end), t))

        
        return swap_conflicts

    def process_solver_output_and_handle_collisions(self, solution, cnf_constructor):
        """
        Process the solver output and handle collisions by adding constraints.

        Parameters:
        - solution (list of int): Solution from the probSAT solver.
        - cnf_constructor (CNFConstructor): CNFConstructor object.

        Returns:
        - paths (list of list of tuple): Paths for each agent.
        - collisions (list of tuple): List of collisions detected.
        - cnf_constructor (CNFConstructor): Updated CNFConstructor object.
        """   
        
        paths = self.parse_solver_output(solution, cnf_constructor.variable_map)

        #edge_paths = self.parse_solver_output_with_edges(solution, cnf_constructor.variable_map)
        #for edge_path in edge_paths:
        #    print('edge path', edge_path)
        
        #first vertex collisions
        collisions = self.find_collisions(paths)
        
        for collision in collisions:
            collision_clause = cnf_constructor.add_single_collision_clause(collision)
        #then edge swap collisions
        swap_conflicts = self.find_edge_collisions(solution, cnf_constructor.variable_map)
        for swap_conflict in swap_conflicts:
            swap_collison_clause = cnf_constructor.add_edge_collision_clause(swap_conflict)
        return paths, collisions, swap_conflicts, cnf_constructor

    def generate_initial_assignments(self, agent_paths, variable_map):
        """
        Generate initial assignments for the probSAT solver.

        Parameters:
        - agent_paths (list of list of tuple): Paths for each agent.
        - variable_map (dict): Mapping of variables to their indices.

        Returns:
        - assignments (list of int): List of initial assignments.
        """
        assignments = [-var_index for var_index in variable_map.values()]
        for agent_id, path in enumerate(agent_paths):
            
            timestep = 0
            for node in path:
                #set nodes to true
                var_index = variable_map.get((agent_id, node, timestep))
                if var_index:
                    assignments[var_index - 1] *= -1  # Set only the variables on the path to True
                else:
                    print(f"Something went wrong, could not find", agent_id," ",node," ",timestep," in variable_map")

                #set edges to true
                if timestep < len(path)-1:
                    next_node = path[timestep+1]
                    edge_var = variable_map.get((agent_id, node, next_node, timestep))
                    if edge_var:
                        assignments[edge_var - 1] *= -1
                    else:
                        print(f"Something went wrong, could not find edge", agent_id," ",node," ",next_node," ",timestep," in variable_map")

                timestep += 1
        
        return assignments
    
    def generate_partial_initial_assignments(self, mdds, cnf_constructor):
        """
        Generate initial assignments for the pysat solver.

        Parameters:
        - mdds (dict): Dictionary of MDDs for each agent.
        - cnf_constructor (CNFConstructor): CNFConstructor object.

        Returns:
        - initial_assignments (list of int): List of initial assignments.
        """
        sample_paths = [mdd.sample_random_path() for mdd in mdds.values()]

        if len(sample_paths[0]) == self.initial_max_timesteps:
            if self.initial_paths == None:
                self.initial_paths = sample_paths
            else:
                sample_paths = self.initial_paths

        sample_paths_with_timesteps = []
        
        for path in sample_paths:
            temp_path = []
            timestep = 0
            for node in path:
                temp_path.append((timestep, node))
                timestep += 1
            sample_paths_with_timesteps.append(temp_path)

        #collisions
        collisions = self.find_collisions(sample_paths_with_timesteps)

        unique_conflicting_agents = set()
        for collision in collisions:
            unique_conflicting_agents.add(collision[0])
            unique_conflicting_agents.add(collision[1]) #excluding all conflicting agents is less restricitv fore the solution
            #excluding only one agent would be 'more help' but for CDCL solvers this tightens  the solution space too much.
        #get all paths of non conflicting agents
        good_paths = []
        for agent_id in range(len(sample_paths_with_timesteps)):
            if agent_id in unique_conflicting_agents:
                good_paths.append([])
            else:
                good_paths.append(sample_paths_with_timesteps[agent_id])

        initial_assignments = self.generate_partial_assignments(good_paths, cnf_constructor.variable_map, mdds)
        return initial_assignments
    

    def generate_partial_initial_assignments_with_random_colliders(self, mdds, cnf_constructor):
        """
        Generate initial assignments for the pysat solver, excluding paths of colliding agents and initializing all MDD variables randomly.

        Parameters:
        - mdds (dict): Dictionary of MDDs for each agent.
        - cnf_constructor (CNFConstructor): CNFConstructor object.

        Returns:
        - initial_assignments (list of int): List of initial assignments.
        """
        sample_paths = [mdd.sample_random_path() for mdd in mdds.values()]
        sample_paths_with_timesteps = []

        for path in sample_paths:
            temp_path = []
            timestep = 0
            for node in path:
                temp_path.append((timestep, node))
                timestep += 1
            sample_paths_with_timesteps.append(temp_path)

        # Find collisions
        collisions = self.find_collisions(sample_paths_with_timesteps)

        unique_conflicting_agents = set()
        for collision in collisions:
            unique_conflicting_agents.add(collision[0])
            unique_conflicting_agents.add(collision[1])

        # Get all paths of non-conflicting agents
        good_paths = []
        for agent_id in range(len(sample_paths_with_timesteps)):
            if agent_id in unique_conflicting_agents:
                good_paths.append([])
            else:
                good_paths.append(sample_paths_with_timesteps[agent_id])

        initial_assignments = self.generate_partial_assignments(good_paths, cnf_constructor.variable_map, mdds)

        # Initialize all MDD variables of colliding agents randomly
        for agent_id in unique_conflicting_agents:
            for timestep in mdds[agent_id].levels.keys():
                for node in mdds[agent_id].get_nodes_at_level(timestep):
                    var_index = cnf_constructor.variable_map.get((agent_id, node.position, timestep))
                    if var_index:
                        # Randomly set the variable to True or False
                        if np.random.rand() > 0.5:
                            initial_assignments.append(var_index)
                        else:
                            initial_assignments.append(-var_index)
        return initial_assignments
    
    def set_colliding_agents_to_random(self, initial_assignments, collisions, variable_map, mdds):
        """
        Set all variables of agents involved in collisions to random values.

        Parameters:
        - initial_assignments (list of int): List of initial assignments.
        - collisions (list of tuple): List of collisions detected.
        - variable_map (dict): Mapping of variables to their indices.
        - mdds (dict): Dictionary of MDDs for each agent.

        Returns:
        - updated_assignments (list of int): Updated list of initial assignments with random values for colliding agents.
        """
        # Create a set of unique conflicting agents
        unique_conflicting_agents = set()
        for collision in collisions:
            unique_conflicting_agents.add(collision[0])
            #unique_conflicting_agents.add(collision[1])

        # Initialize all MDD variables of colliding agents randomly
        updated_assignments = initial_assignments.copy()
        for agent_id in unique_conflicting_agents:
            for timestep in mdds[agent_id].levels.keys():
                for node in mdds[agent_id].get_nodes_at_level(timestep):
                    var_index = variable_map.get((agent_id, node.position, timestep))
                    if var_index:
                        # Randomly set the variable to True or False
                        if np.random.rand() > 0.5:
                            updated_assignments.append(var_index)
                        else:
                            updated_assignments.append(-var_index)

        return updated_assignments
    
    def generate_partial_assignments(self, agent_paths, variable_map, mdds, set_negative_variables = False):
        """
        Generate initial assignments for a pysat solver
        That doesnt assign variables for agents that have an empty path
        Parameters:
        - agent_paths (list of list of tuple): Paths for each agent.
        - variable_map (dict): Mapping of variables to their indices.

        Returns:
        - assignments (list of int): List of initial assignments of variables found in the path.
        """
        assignments = []


        for agent_id, path in enumerate(agent_paths):
            
            if len(path) == 0:
               
                continue
    
            for timestep, node in path:
                #get vertex
                var_index = variable_map.get((agent_id, node, timestep))
                #set all vertex variables on given paths to true
                if var_index:
                    assignments.append(var_index)
                #get edge (last vertex doesnt have any edges leading onward)
                if timestep < len(path)-1:
                    next_node = path[timestep+1]
                    edge_var_index = variable_map.get((agent_id, node, next_node, timestep))
                    #set edge var true
                    if edge_var_index:
                        assignments.append(edge_var_index)
                
                if set_negative_variables:
                    for not_at_node in mdds[agent_id].get_nodes_at_level(timestep):
                        #set all variables of alternative positions to false, prof reccomended to set as few variables as possible
                        if not_at_node.position != node:
                            not_var_index = variable_map.get((agent_id, not_at_node.position, timestep))
                            assignments.append(-not_var_index)
                        #NOT DOING IT FOR EDGES BC DONT USE IT ANYWAY
                
        return assignments

    def write_assignments_to_file(self, assignments, filename):
        """
        Write initial assignments to a file.

        Parameters:
        - assignments (list of int): List of initial assignments.
        - filename (str): Path to the file where assignments will be saved.
        """
        #print(assignments)
        with open(filename, 'w') as file:
            for var in assignments:
                if var < 0:
                    var = -var
                    val = 0
                else:
                    val = 1
                file.write(f"{var} {val}\n")

    

    def sample_paths_and_add_initial_collisions(self, mdds , cnf_constructor):
        """
        Sample random paths and add initial collisions to the CNF.

        Parameters:
        - mdds (dict): Dictionary of MDDs for each agent.
        - cnf_constructor (CNFConstructor): CNFConstructor object.

        Returns:
        - initial_assignments (list of int): List of initial assignments.
        """
        sample_paths = [mdd.sample_random_path() for mdd in mdds.values()]

        # all solvers get the same initial sample paths
        if len(sample_paths[0]) == self.initial_max_timesteps:
            if self.initial_paths == None:
                self.initial_paths = sample_paths
            else:
                sample_paths = self.initial_paths
        initial_assignments = self.generate_initial_assignments(sample_paths, cnf_constructor.variable_map)
        paths, initial_collisions, initial_swap_conflicts, cnf_constructor  = self.process_solver_output_and_handle_collisions(initial_assignments, cnf_constructor)
        
        for collision in initial_collisions:
            self.added_vertex_collisions.append(collision)
        for swap in initial_swap_conflicts:
            self.added_edge_collisions.append(swap)

        
        if len(initial_collisions) == 0:
            if len(initial_swap_conflicts) == 0:
                print("No initial collisions found SOLUTION FOUND IN INITIAL ASSIGNMENT")
            print("only vertex conflicts in initial solutions found")

        return initial_assignments
    
    def safety_sample_paths(self, mdds , cnf_constructor):
        """
        Sample random paths and add initial collisions to the CNF.

        Parameters:
        - mdds (dict): Dictionary of MDDs for each agent.
        - cnf_constructor (CNFConstructor): CNFConstructor object.

        Returns:
        - initial_assignments (list of int): List of initial assignments.
        """
        sample_paths = [mdd.sample_random_path() for mdd in mdds.values()]
        
        initial_assignments = self.generate_initial_assignments(sample_paths, cnf_constructor.variable_map)
        #paths, initial_collisions, initial_swap_conflicts, cnf_constructor  = self.process_solver_output_and_handle_collisions(initial_assignments, cnf_constructor)
        
        
        #if len(initial_collisions) == 0:
        #    if len(initial_swap_conflicts) == 0:
        #        print("No initial collisions found SOLUTION FOUND IN INITIAL ASSIGNMENT")
        #    print("only vertex conflicts in initial solutions found")

        return initial_assignments

    def get_unique_filename(self, filename):
        """
        Generate a unique filename by appending a counter if the file already exists.

        Parameters:
        - filename (str): Original filename.

        Returns:
        - unique_filename (str): Unique filename.
        """
        base, ext = os.path.splitext(filename)
        counter = 1
        while os.path.exists(filename):
            filename = f"{base}_{counter}{ext}"
            counter += 1
        return filename

    def generate_scenario_hash(self, starts, goals):
        """
        Generate a hash for the given starts and goals to use as a cache key.

        Parameters:
        - starts (list of list of int): List of start positions.
        - goals (list of list of int): List of goal positions.

        Returns:
        - scenario_hash (str): Hash string representing the starts and goals.
        """
        scenario_str = str(starts) + str(goals)
        return hashlib.md5(scenario_str.encode()).hexdigest()
    

    

    def solve_cnf_file_pysat(self, cnf_file_path, init_assignment_path = None):
        
        def load_initial_assignment_pysat(file_path):
            """
            Load initial assignments from a text file.

            Parameters:
            - file_path (str): Path to the file containing initial assignments.

            Returns:
            - init_assignment (list of int): List of variable assignments.
            """
            data = np.loadtxt(file_path, dtype=int)
            if len(data) == 0:
                print("No initial assignment found")
                return []
            var_ids = data[:, 0]
            values = data[:, 1]

            # Start with all variable IDs negative
            init_assignment = -var_ids

            # Make variable IDs positive where the value is 1
            init_assignment[values == 1] *= -1

            return init_assignment.tolist()

        # Load the CNF file
        time_with_loading_cnf_start = time.process_time()
        loading_time_start = time.process_time()
        cnf = CNF(from_file=cnf_file_path)
        
        # Initialize the Glucose solver
        solver = Glucose4(use_timer=True)
        
        # Add the CNF clauses to the solver
        solver.append_formula(cnf.clauses)
        

        init_assignment = []
        init_assignment_loading_time_start = time.process_time()
        if init_assignment_path is not None:
            # Load initial assignment
            init_assignment = load_initial_assignment_pysat(init_assignment_path)
        init_assignment_loading_time_end = time.process_time()
        init_assignment_loading_time = init_assignment_loading_time_end - init_assignment_loading_time_start
       
            
            
        # Solve the CNF
        pure_solving_time_start = time.process_time()
        solution, stats = None, None
        loading_time = time.process_time() - loading_time_start
        if solver.solve(assumptions=init_assignment):
            pure_solving_time_end = time.process_time()
            pure_solving_time = pure_solving_time_end - pure_solving_time_start
            
            solution = solver.get_model()

            time_with_loading_cnf_end = time.process_time()
            time_with_loading_cnf = time_with_loading_cnf_end - time_with_loading_cnf_start
            print("SATISFIABLE")
            stats = {"Total SAT Solver Time (s)": time_with_loading_cnf,
                    "Solving Time (s)": pure_solving_time, 
                    "Loading Time (s)": loading_time,
                    "Init Assignment Loading Time": init_assignment_loading_time,
                    "Conflicts": solver.accum_stats().get('conflicts'), 
                    "Decisions": solver.accum_stats().get('decisions'),
                    "Propagations": solver.accum_stats().get('propagations'),
                    "Restarts": solver.accum_stats().get('restarts'),
                    "Number of Variables": solver.nof_vars(),
                    "Number of Clauses": solver.nof_clauses(),
                    "Solving Time": solver.time()}
        else:
            pure_solving_time_end = time.process_time()
            pure_solving_time = pure_solving_time_end - pure_solving_time_start
            time_with_loading_cnf_end = time.process_time()
            time_with_loading_cnf = time_with_loading_cnf_end - time_with_loading_cnf_start
            print("UNSATISFIABLE")
            stats = {"Total SAT Solver Time (s)": time_with_loading_cnf,
                    "Solving Time (s)": pure_solving_time, 
                    "Loading Time (s)": loading_time,
                    "Init Assignment Loading Time": init_assignment_loading_time,
                    "Conflicts": solver.accum_stats().get('conflicts'), 
                    "Decisions": solver.accum_stats().get('decisions'),
                    "Propagations": solver.accum_stats().get('propagations'),
                    "Restarts": solver.accum_stats().get('restarts'),
                    "Number of Variables": solver.nof_vars(),
                    "Number of Clauses": solver.nof_clauses(),
                    "Solving Time": solver.time()}
        # Delete the solver instance
        solver.delete()
        return solution, stats

    def compare_performance(self):
        """
        Compare the performance of different probSAT configurations.

        Returns:
        - results (list of dict): List of results for each scenario and configuration.
        """
        results_key = "results_" + self.save_name + "_probSAT.csv"
        results = []
        results_columns = [
             'Set', 'Scenario', 'Lazy Encoding', 'Initial Assignment', 'Update Initial Assignment', 'Max Timesteps', 'Initial Max Timesteps',
             'Total Time (s)', 'Timesteps Total Time (s)', 'Distance Matrix Time (s)', 'MDD Time (s)', 'Timesteps MDD Time (s)', 
             'Base CNF Construction Time (s)', 'Timesteps Base CNF Construction Time (s)', 'CNF Construction Time (s)', 'Timesteps CNF Construction Time (s)',
             'Total SAT Solver Time (s)', 'Timestep Total Solver Times (s)', 'Timestep All Solver Times (s)', 
             'Timestep Initial Assignment Time (s)', 'Initial Assignment Time (s)', 'Additional Lazy Time (s)', 'Timestep Additional Lazy Time (s)',
             'Number of Added Collisions', 'Timestep Number of Added Collisions', 'Number of Variables', 'Number of Clauses', 'Max Flips', 'Max Tries',
             'Stdout', 'Number of Literals', 'Max Clause Length', 'Num Flips', 'Timestep Num Flips', 'Avg Flips per Variable', 'Avg Flips per Clause', 'Flips per Sec',
             'CPU Time', 'Timestep CPU Time', 'Timestep Sum of CPU Time', 'Loading Time', 'Timestep Loading Time', 'Timestep Sum of Loading Time', 
             'Solving Time', 'Timestep Solving Time', 'Timestep Sum of Solving Time', 'Solving Time (s)', 'Timestep Solving Time (s)', 'Timestep Sum of Solving Time (s)', 
             'Paths']
        probSAT_results_df = pd.DataFrame(columns=results_columns)

        results_key_pysat = "results_" + self.save_name + "_pysat.csv"
        pysat_results_columns = [
             'Set', 'Scenario', 'Lazy Encoding', 'Initial Assignment', 'Update Initial Assignment', 'Max Timesteps', 'Initial Max Timesteps', 
             'Total Time (s)', 'Timesteps Total Time (s)', 'Distance Matrix Time (s)', 'MDD Time (s)', 'Timesteps MDD Time (s)', 
             'Base CNF Construction Time (s)', 'Timesteps Base CNF Construction Time (s)', 'CNF Construction Time (s)', 'Timesteps CNF Construction Time (s)', 
             'Total SAT Solver Time (s)', 'Timestep Total Solver Time (s)', 'Timestep All Solver Time (s)', 'Timestep Initial Assignment Time (s)', 
             'Initial Assignment Time (s)', 'Additional Lazy Time (s)', 'Timestep Additional Lazy Time (s)', 'Number of Added Collisions', 
             'Timestep Number of Added Collisions', 'Number of Variables', 'Number of Clauses', 
             'Timestep Conflicts', 'Conflicts', 'Decisions', 'Timestep Decisions', 'Propagations', 'Timestep Propagations', 'Restarts', 'Timestep Restarts', 
             'Solving Time (s)', 'Timestep Solving Time (s)', 'Loading Time (s)', 'Timestep Loading Time (s)', 'Solving Time', 'Timestep Solving Time', 
             'Paths']
        pysat_results_df = pd.DataFrame(columns=pysat_results_columns)
                         



        for idx, starts_and_goals_set in enumerate(self.starts_and_goals):
            print(self.map_name, idx)
            if self.map_name == 'maze-32-32-4-even-10':
                if idx < 5: # or idx == 2: ##or idx == 6:
                    continue
              #      print('doing IT')#> 0 :#or idx > 4:
               # else:
                #    continue

            print("---------------------------------------------------------------------------------")
            print(f"Starting with starts_and_goals set {idx + 1}")
            print("---------------------------------------------------------------------------------")

            
            starts = starts_and_goals_set['starts']
            goals = starts_and_goals_set['goals']
            
            dist_start_time = time.process_time()
            distance_matrices, initial_max_timesteps = self.compute_max_timesteps(starts, goals)
            dist_end_time = time.process_time()
            distance_matrix_time = dist_end_time - dist_start_time


            base_cnf_constructor = None
            base_cnf_construction_time = 0
            created_base_cnf = False
            created_cnf_with_conflicts = False
            cnf_no_conflict_constructor = None
            cnf_constructor = None
            cnf_text = ""

            # Scenarios to test
            scenarios = [
                #probSAT with eager encoding without init
                #{'solver': 'probSAT', 'lazy_encoding': False, 'update_init_assignment': False, 'init_assignment': False, 'zero_init': False},
                #probSAT with eager encoding and init
                #{'solver': 'probSAT', 'lazy_encoding': False, 'update_init_assignment': False, 'init_assignment': True, 'zero_init': True},
                #{'solver': 'probSAT', 'lazy_encoding': False, 'update_init_assignment': False, 'init_assignment': True, 'zero_init': False},
                #probSAT with lazy encoding and no init
                #without init or update init assignment the lazy encoding needs a lot of updates
                #{'solver': 'probSAT', 'lazy_encoding': True, 'update_init_assignment': True, 'init_assignment': False, 'zero_init': False},
                #probSAT with lazy encoding and init
                #{'solver': 'probSAT', 'lazy_encoding': True, 'update_init_assignment': False, 'init_assignment': True, 'zero_init': True},
                #{'solver': 'probSAT', 'lazy_encoding': True, 'update_init_assignment': False, 'init_assignment': True, 'zero_init': False},
                #probSAT with lazy encoding and init and update init
                #{'solver': 'probSAT', 'lazy_encoding': True, 'update_init_assignment': True, 'init_assignment': True, 'zero_init': True},
                #{'solver': 'probSAT', 'lazy_encoding': True, 'update_init_assignment': True, 'init_assignment': True, 'zero_init': False},
                #Glucose without zero init
                #Glucose with eager encoding and no init
                #{'solver': 'Glucose4', 'lazy_encoding': False, 'update_init_assignment': False, 'init_assignment': False, 'zero_init': False},
                #Glucose with eager encoding and init
                #{'solver': 'Glucose4', 'lazy_encoding': False, 'update_init_assignment': False, 'init_assignment': True, 'zero_init': False},
                #Glucose with lazy encoding and no init
                ##{'solver': 'Glucose4', 'lazy_encoding': True, 'update_init_assignment': True, 'init_assignment': False, 'zero_init': False},
                #Glucose with lazy encoding and init
                #{'solver': 'Glucose4', 'lazy_encoding': True, 'update_init_assignment': False, 'init_assignment': True, 'zero_init': False},
                #Glucose with lazy encoding and init and update init
                ##{'solver': 'Glucose4', 'lazy_encoding': True, 'update_init_assignment': True, 'init_assignment': True, 'zero_init': False},
                #so glucose first
                {'solver': 'probSAT', 'lazy_encoding': True, 'update_init_assignment': True, 'init_assignment': True, 'zero_init': False},

            ]
            
            '''if self.map_name == 'empty-32-32-even-5' and idx == 7:
                 scenarios = [#probSAT with eager encoding without init
                #{'solver': 'probSAT', 'lazy_encoding': False, 'update_init_assignment': False, 'init_assignment': False, 'zero_init': False},
                #probSAT with eager encoding and init
                #{'solver': 'probSAT', 'lazy_encoding': False, 'update_init_assignment': False, 'init_assignment': True, 'zero_init': True},
                #{'solver': 'probSAT', 'lazy_encoding': False, 'update_init_assignment': False, 'init_assignment': True, 'zero_init': False},
                #probSAT with lazy encoding and no init
                #without init or update init assignment the lazy encoding needs a lot of updates
                #{'solver': 'probSAT', 'lazy_encoding': True, 'update_init_assignment': True, 'init_assignment': False, 'zero_init': False},
                #probSAT with lazy encoding and init
                #{'solver': 'probSAT', 'lazy_encoding': True, 'update_init_assignment': False, 'init_assignment': True, 'zero_init': True},
                #{'solver': 'probSAT', 'lazy_encoding': True, 'update_init_assignment': False, 'init_assignment': True, 'zero_init': False},
                #probSAT with lazy encoding and init and update init
                #{'solver': 'probSAT', 'lazy_encoding': True, 'update_init_assignment': True, 'init_assignment': True, 'zero_init': True},
                #{'solver': 'probSAT', 'lazy_encoding': True, 'update_init_assignment': True, 'init_assignment': True, 'zero_init': False},
                #Glucose with lazy encoding and no init
                {'solver': 'Glucose4', 'lazy_encoding': True, 'update_init_assignment': True, 'init_assignment': False, 'zero_init': False},
                #Glucose with lazy encoding and init
                #{'solver': 'Glucose4', 'lazy_encoding': True, 'update_init_assignment': False, 'init_assignment': True, 'zero_init': False},
                #Glucose with lazy encoding and init and update init
                {'solver': 'Glucose4', 'lazy_encoding': True, 'update_init_assignment': True, 'init_assignment': True, 'zero_init': False},
            ]'''
                



          
            for scenario in scenarios:
                
                max_timesteps = initial_max_timesteps
                solution_found = False

                last_paths = []
                last_cnf = ""

                all_timesteps_num_added_collisions = []
                all_timesteps_num_added_vertex_collisions = []
                all_timesteps_num_added_edge_collisions = []
                all_timesteps_num_collision_addings = []
                timestep_num_added_collisions = []
                timestep_num_added_vertex_collisions = []
                timestep_num_added_edge_collisions = []
                added_collisions = [] #gets reset every time max_timesteps is increased
                self.added_vertex_collisions = []
                self.added_edge_collisions = []
                
                timestep_stdouts_to_save = []


                #timings
                start_time = time.process_time()
                mdd_time = 0
                cnf_construction_time = 0
                total_sat_solver_time = 0
                timestep_all_solver_times = []
                initial_assignment_time = 0
                additional_clauses_time = 0
                timestep_all_additional_clause_times = []

                #timestep solver stats
                timestep_number_of_variables = []
                timestep_number_of_literals = []
                timestep_number_of_clauses = []
                timestep_max_clause_length = []
                timestep_num_flips = []
                timestep_avg_flips_per_variable = []
                timestep_avg_flips_per_clause = []
                timestep_flips_per_sec = []
                timestep_cpu_time = []
                timestep_loading_time = []
                timestep_solving_time = []
                timestep_solving_time_s = []
                timestep_num_conflicts = []
                timestep_num_decisions = []
                timestep_num_propagations = []
                timestep_num_restarts = []
                timestep_number_of_tries = []
                #timestep_initial_clauses_satisfied = []
                #timestep_initial_clauses_satisfied_percentage = []

                max_timesteps_list = []
                mdd_time_list = []
                base_cnf_construction_time_list = []
                cnf_construction_time_list = []
                initial_assignment_time_list = []
                
                #tracking stats across time steps
                individual_timesteps_total_time = []
                individual_timesteps_all_sat_solver_times = []
                individual_timesteps_sum_of_all_sat_solver_times = []
                sum_of_all_timesteps_additional_clauses_time = 0
                individual_timesteps_additional_clauses_time = []
                sum_of_all_timesteps_num_added_collisions = 0
                individual_timesteps_num_added_collisions = []
                individual_timesteps_num_added_vertex_collisions = []
                individual_timesteps_num_added_edge_collisions = []
                individual_timesteps_stdouts = []
                individual_timesteps_number_of_variables = []
                individual_timesteps_number_of_literals = []
                individual_timesteps_number_of_clauses = []
                individual_timesteps_max_clause_length = []
                sum_of_all_timesteps_num_flips  = 0
                individual_timesteps_num_flips  = []
                individual_timesteps_avg_flips_per_variable = []
                individual_timesteps_avg_flips_per_clause = []
                individual_timesteps_flips_per_sec  = []
                sum_of_all_timesteps_cpu_time   = 0
                individual_timesteps_cpu_time   = []
                individual_timesteps_sum_of_cpu_time = []
                sum_of_all_timesteps_loading_time   = 0
                individual_timesteps_loading_time   = []
                individual_timesteps_sum_of_loading_time = []
                sum_of_all_timesteps_solving_time  = 0
                individual_timesteps_solving_time = []
                individual_timesteps_sum_of_solving_time = []
                sum_of_all_timesteps_solving_time_s = 0
                individual_timesteps_solving_time_s = []
                individual_timesteps_sum_of_solving_time_s = []
                sum_of_all_timesteps_tries = 0
                individual_timesteps_tries = []
                #individual_timesteps_initial_clauses_satisfied = []
                #individual_timesteps_initial_clauses_satisfied_percentage = []
                
                
                #Pysat stats
                sum_of_all_timesteps_num_conflicts = 0
                individual_timesteps_num_conflicts = []

                sum_of_all_timesteps_num_decisions = 0
                individual_timesteps_num_decisions = []

                sum_of_all_timesteps_num_propagations = 0
                individual_timesteps_num_propagations = []

                sum_of_all_timesteps_num_restarts = 0
                individual_timesteps_num_restarts = []

                sum_of_all_timestep_solving_time_s = 0
                individual_timestep_solving_time_s = []

                sum_of_all_timestep_loading_time = 0
                individual_timestep_loading_time = []

                sum_of_all_timestep_solving_time = 0
                individual_timestep_solving_time = []


                
                while max_timesteps <= self.max_timesteps_threshold:

                    #stats for one timestep
                    timestep_total_time_start = time.process_time()


                    max_timesteps_list.append(max_timesteps)
                    if scenario['zero_init']:
                        key = self.save_name + f"starts_and_goals_{idx + 1}_{scenario['solver']}_lazy_{scenario['lazy_encoding']}_use_zero_init_{scenario['init_assignment']}_update_init_{scenario['update_init_assignment']}_timesteps_{max_timesteps}"
                    else:
                        key = self.save_name + f"starts_and_goals_{idx + 1}_{scenario['solver']}_lazy_{scenario['lazy_encoding']}_use_init_{scenario['init_assignment']}_update_init_{scenario['update_init_assignment']}_timesteps_{max_timesteps}"

                    print(f"Testing {scenario} with max_timesteps: {max_timesteps}")
                    # Create MDDs
                    mdd_start_time = time.process_time()
                    mdds, constructors = self.create_mdds(starts, goals, max_timesteps, distance_matrices)
                    mdd_end_time = time.process_time()
                    mdd_time += (mdd_end_time - mdd_start_time)
                    mdd_time_list.append(mdd_end_time - mdd_start_time)
                    print(f"MDDs created with max_timesteps: {max_timesteps}")
                
                    if max_timesteps == initial_max_timesteps:
                        # Create and save CNF
                        print('creating base cnf')
                        print(scenario)
                        if not created_base_cnf:
                            base_cnf, base_cnf_file_path, base_cnf_constructor, base_cnf_construction_time_current = self.create_and_save_cnf(mdds, 
                                                                                                                                    filename=f"{self.save_name}starts_and_goals_{idx + 1}_timesteps_{max_timesteps}_base_cnf", 
                                                                                                                        lazy_encoding=True)
                            
                            base_cnf_construction_time += base_cnf_construction_time_current 
                            base_cnf_construction_time_list.append(base_cnf_construction_time_current)
                            base_cnf = None
                            created_base_cnf = True
                        print('base cnf variables', base_cnf_constructor.next_variable_id-1)
                        print('base cnf clauses', len(base_cnf_constructor.cnf.clauses))
                        #create scenario specific cnf
                        cnf_constructor = CNFConstructor(mdds, 
                                                        lazy_encoding=scenario['lazy_encoding'], 
                                                        cnf=base_cnf_constructor.cnf, 
                                                        variable_map=base_cnf_constructor.variable_map, 
                                                        next_variable_id=base_cnf_constructor.next_variable_id)
                        cnf_construction_start_time = time.process_time()
                        if not scenario['lazy_encoding']:
                            if not created_cnf_with_conflicts:
                                cnf_constructor.create_no_conflict_clauses()  
                                cnf_no_conflict_constructor = CNFConstructor(mdds, 
                                                                            lazy_encoding=scenario['lazy_encoding'], 
                                                                            cnf=cnf_constructor.cnf, 
                                                                            variable_map=cnf_constructor.variable_map, 
                                                                            next_variable_id=cnf_constructor.next_variable_id)
                                created_cnf_with_conflicts = True
                            else:
                                cnf_constructor = CNFConstructor(mdds, 
                                                                lazy_encoding=scenario['lazy_encoding'], 
                                                                cnf = cnf_no_conflict_constructor.cnf, 
                                                                variable_map=cnf_no_conflict_constructor.variable_map, 
                                                                next_variable_id=cnf_no_conflict_constructor.next_variable_id)
        
                        cnf_text = str(cnf_constructor.cnf)
                        cnf_construction_end_time = time.process_time()
                        cnf_construction_time_current = base_cnf_construction_time + cnf_construction_end_time - cnf_construction_start_time
                        
                        #save cnf to file
                        cnf_file_path = self.get_unique_filename(f"{self.output_dir}/"+key+"_original.cnf")
                        with open(cnf_file_path, 'w') as file:
                            file.write(cnf_text)
                        print(f"CNF created and saved to {cnf_file_path}")
                    else:
                        # Create and save CNF
                        cnf, cnf_file_path, cnf_constructor, cnf_construction_time_current = self.create_and_save_cnf(mdds, filename=key,lazy_encoding=scenario['lazy_encoding'])
                        cnf = None
                        print(f"CNF created and saved to {cnf_file_path}")
                    cnf_construction_time += cnf_construction_time_current
                    cnf_construction_time_list.append(cnf_construction_time_current)
                    
                    init_assignment_path = None

                    if scenario['init_assignment'] :
                        
                        init_assignment_path = self.get_unique_filename(f"{self.output_dir}/"+key+"_initial_assignment.txt")
                        if scenario['zero_init']:
                            init_assignment_path = self.get_unique_filename(f"{self.output_dir}/"+key+"_zero_initial_assignment.txt")
                            init_assignment_time_start = time.process_time()
                            initial_assignments = [-var_index for var_index in cnf_constructor.variable_map.values()]
                            init_assignment_time_end = time.process_time()
                        else:
                            if scenario['solver'] == 'probSAT':
                                init_assignment_time_start = time.process_time()
                                initial_assignments = self.sample_paths_and_add_initial_collisions(mdds, cnf_constructor)
                                #initial_assignments = self.set_colliding_agents_to_random(initial_assignments, added_collisions, cnf_constructor.variable_map, mdds)
                                init_assignment_time_end = time.process_time()


                            elif scenario['solver'] == 'Glucose4':
                                init_assignment_time_start = time.process_time()
                                initial_assignments = self.generate_partial_initial_assignments(mdds, cnf_constructor)
                                init_assignment_time_end = time.process_time()
                        #print(f"Initial assignments {initial_assignments}")
                        self.write_assignments_to_file(initial_assignments, init_assignment_path)
                        print(f"Initial assignments saved to {init_assignment_path}")
                        
                        initial_assignment_time += init_assignment_time_end - init_assignment_time_start
                        initial_assignment_time_list.append(init_assignment_time_end - init_assignment_time_start)

                    if scenario['lazy_encoding']:
                        
                        cnf_text = str(cnf_constructor.cnf)
                        original_cnf_file_path = cnf_file_path
                        filename_addition = 'used_instance_of_cnf'
                        cnf_file_path = self.get_unique_filename(f"{cnf_file_path[:-4]}"+filename_addition+".cnf")
                        with open(cnf_file_path, 'w') as file:
                            file.write(cnf_text)
                        print(f" working version of CNF created and saved to {cnf_file_path}")

                    number_of_collision_addings = 0
                    timestep_number_of_added_collisions = 0
                    timestep_number_of_added_vertex_collision = 0
                    timestep_number_of_added_edge_collision = 0

                    no_conflicting_agents = False
                    safety_run_done = False
                    while True:
                        if scenario['solver'] == 'probSAT':
                            print('running probSAT')
                        
                                    
                            # Run ProbSAT solver
                            print("using cnf at ", cnf_file_path)
                            
                            stdout, stderr, elapsed_time, max_flips, max_tries = self.run_prosat_solver_with_optional_initial_assignment(cnf_constructor.cnf, cnf_file_path, init_assignment_path=init_assignment_path)#, save_output_path=key)
                            
                            if len(stderr) > 0:
                                print(f"Error running probSAT: {stderr}")
                            total_sat_solver_time += elapsed_time

                            timestep_all_solver_times.append(elapsed_time)
                           
                            #get solver stats
                            #print(f"stdout: {stdout}")
                            #stdout_stats = self.extract_statistics([stdout])
                            stats = {}

                            # Regex patterns for the required fields
                            patterns = {
                                'number_of_variables': r'c number of variables\s*:\s*(\d+)',
                                'number_of_literals': r'c number of literals\s*:\s*(\d+)',
                                'number_of_clauses': r'c number of clauses\s*:\s*(\d+)',
                                'max_clause_length': r'c max\. clause length\s*:\s*(\d+)',
                                'num_flips': r'c numFlips\s*:\s*(\d+)',
                                'avg_flips_per_variable': r'c avg\. flips/variable\s*:\s*([\d.]+)',
                                'avg_flips_per_clause': r'c avg\. flips/clause\s*:\s*([\d.]+)',
                                'flips_per_sec': r'c flips/sec\s*:\s*([\d.]+|inf)',
                                'cpu_time': r'c CPU Time\s*:\s*([\d.]+)',
                                'loading_time': r'c Loading Time\s*:\s*([\d.]+)',
                                'solver_time': r'c Solver Time\s*:\s*([\d.]+)',
                                'solver_time_s': r'c Clock Time\s*:\s*([\d.]+)',
                                #'initial_satisfied_clauses': r'c Satisfied clauses \(initial\)\s*:\s*(\d+)',
                                #'initial_percentage_satisfied': r'c Percentage satisfied \(initial\)\s*:\s*([\d.]+)%',
                                'numTries': r'c numTries\s*:\s*(\d+)',
                            }   

                            # Extract each field using the corresponding pattern
                            for clue, pattern in patterns.items():
                                match = re.search(pattern, stdout)
                                if match:
                                    stats[clue] = match.group(1)
                            stdout_stats = stats
                            nov = int(stdout_stats.get('number_of_variables'))
                            nol = int(stdout_stats.get('number_of_literals'))
                            noc = int(stdout_stats.get('number_of_clauses'))
                            mcl = int(stdout_stats.get('max_clause_length'))
                            nf = int(stdout_stats.get('num_flips'))
                            afpv = float(stdout_stats.get('avg_flips_per_variable'))
                            afpc = float(stdout_stats.get('avg_flips_per_clause'))
                            #isc = int(stdout_stats.get('initial_satisfied_clauses'))
                            #ips = float(stdout_stats.get('initial_percentage_satisfied'))
                            nt = int(stdout_stats.get('numTries'))
                            if stdout_stats.get('flips_per_sec') == '-nan' or stdout_stats.get('flips_per_sec') == 'inf':
                                fps = 0
                            elif stdout_stats.get('flips_per_sec') == None:
                                fps = 0
                            else:
                                fps = float(stdout_stats.get('flips_per_sec'))
                            if stdout_stats.get('cpu_time') == None:
                                ct = 0
                            else:
                                ct = float(stdout_stats.get('cpu_time'))
                            if stdout_stats.get('loading_time') == None:
                                lt = 0
                            else:
                                lt = float(stdout_stats.get('loading_time'))
                            if stdout_stats.get('solver_time') == None:
                                st = 0
                            else:
                                st = float(stdout_stats.get('solver_time'))
                            if stdout_stats.get('solver_time_s') == None:
                                sts = 0
                            else:
                                sts = float(stdout_stats.get('solver_time_s'))
                            if stdout_stats.get('numTries') == None:
                                nt = 0
                            else:
                                nt = int(stdout_stats.get('numTries'))
                            #if stdout_stats.get('initial_satisfied_clauses') == None:
                            #    isc = 0
                            #else:
                            #    isc = int(stdout_stats.get('initial_satisfied_clauses'))
                            #if stdout_stats.get('initial_percentage_satisfied') == None:
                            #    ips = 0
                            #else:
                            #    ips = float(stdout_stats.get('initial_percentage_satisfied'))
                            timestep_number_of_variables.append(nov)
                            timestep_number_of_literals.append(nol)
                            timestep_number_of_clauses.append(noc)
                            timestep_max_clause_length.append(mcl)
                            timestep_num_flips.append(nf)
                            timestep_avg_flips_per_variable.append(afpv)
                            timestep_avg_flips_per_clause.append(afpc)
                            timestep_flips_per_sec.append(fps)
                            timestep_cpu_time.append(ct)
                            timestep_loading_time.append(lt)
                            timestep_solving_time.append(st)
                            timestep_solving_time_s.append(sts)
                            timestep_number_of_tries.append(nt)
                            #timestep_initial_clauses_satisfied.append(isc)
                            #timestep_initial_clauses_satisfied_percentage.append(ips)
                           

                            # Check if a solution was found
                            solution = None
                            stdout_to_save = ''
                            if "s SATISFIABLE" in stdout:
                                stdout_lines = stdout.split('\n')
                                solution = []
                                for line in stdout_lines:
                                    if line.startswith('v'):
                                        vars = line.split(' ')
                                        solution.extend(int(var) for var in vars if (var != '0' and var != '' and var != 'v'))
                                    else:
                                        stdout_to_save += line + '\n'
                            timestep_stdouts_to_save.append(stdout_to_save)

                        elif scenario['solver'] == 'Glucose4':
                            # Run Glucose solver
                            print('running glucose')
                            
                            ##IF SAFETY RUN NEEDED INIT was deleted
                            sat_solver_start_time = time.process_time()
                            solution, stats = self.solve_cnf_file_pysat(cnf_file_path, init_assignment_path)
                            sat_solver_end_time = time.process_time()
                            total_sat_solver_time += (sat_solver_end_time - sat_solver_start_time)
                            timestep_all_solver_times.append(sat_solver_end_time - sat_solver_start_time)

                            timestep_number_of_variables.append(stats.get('Number of Variables'))
                            timestep_number_of_clauses.append(stats.get('Number of Clauses'))
                            if stats.get('Conflicts') == None:
                                timestep_num_conflicts.append(0)
                            else:
                                timestep_num_conflicts.append(stats.get('Conflicts'))
                            if stats.get('Decisions') == None:
                                timestep_num_decisions.append(0)
                            else:
                                timestep_num_decisions.append(stats.get('Decisions'))
                            if stats.get('Propagations') == None:
                                timestep_num_propagations.append(0)
                            else:
                                timestep_num_propagations.append(stats.get('Propagations'))
                            if stats.get('Restarts') == None:
                                timestep_num_restarts.append(0)
                            else:
                                timestep_num_restarts.append(stats.get('Restarts'))
                            if stats.get('Solving Time') == None:
                                timestep_solving_time.append(0)
                            else:
                                timestep_solving_time.append(stats.get('Solving Time'))
                            if stats.get('Solving Time (s)') == None:
                                timestep_solving_time_s.append(0)
                            else:
                                timestep_solving_time_s.append(stats.get('Solving Time (s)'))
                            if stats.get('Loading Time') == None:
                                timestep_loading_time.append(0)
                            else:
                                timestep_loading_time.append(stats.get('Loading Time (s)'))
                            

                        if solution != None:
                            print(f"Solution found in {scenario} with max_timesteps: {max_timesteps}")
                            #this alread add colllisons to the cnf
                            paths, vertex_collisions, edge_collisions, cnf_constructor = self.process_solver_output_and_handle_collisions(solution, cnf_constructor)
                            last_paths = paths

                            # Reset the safety run flag regardless of collisions
                            safety_run_done = False
                        
                            if not vertex_collisions and not edge_collisions:
                                print("No collisions found in the solution")
                                solution_found = True
                                break
                            else:

                                
                                # Initialize separate tracking lists for new vertex and edge collisions.
                                new_vertex_collisions = []
                                new_edge_collisions = []
                                # Process vertex collisions
                                for collision in vertex_collisions:
                                    if collision not in self.added_vertex_collisions:
                                        self.added_vertex_collisions.append(collision)
                                        new_vertex_collisions.append(collision)
                                    else:
                                        print(f"Collision {collision} already added But it was detected again")
                                # Process edge collisions
                                for collision in edge_collisions:
                                    if collision not in self.added_edge_collisions:
                                        self.added_edge_collisions.append(collision)
                                        new_edge_collisions.append(collision)

                                print("changing cnf at ", cnf_file_path)
                                if (len(new_vertex_collisions) > 0 or len(new_edge_collisions) > 0):
                                    additional_clauses_start_time = time.process_time()
                                    vertex_collision_clauses = []
                                    if new_vertex_collisions:
                                        for collision in new_vertex_collisions:
                                            clause = cnf_constructor.add_single_collision_clause(collision, adding_to_cnf=False)
                                            if clause is not None:
                                                vertex_collision_clauses.append(clause)                
                                            # Write new vertex collision clauses to file
                                            with open(cnf_file_path, 'a') as file: 
                                                file.write(" ".join(map(str, clause)) + " 0\n")

                                        print(f"Added {len(new_vertex_collisions)} new vertex collisions to the CNF")

                                    # Process edge collisions
                                    
                                    edge_collision_clauses = []
                                    if new_edge_collisions:
                                        for collision in new_edge_collisions:
                                            clause = cnf_constructor.add_edge_collision_clause(collision, adding_to_cnf=False)
                                            if clause is not None:
                                                edge_collision_clauses.append(clause)
                                            # Write new edge collision clauses to file
                                            with open(cnf_file_path, 'a') as file:
                                                file.write(" ".join(map(str, clause)) + " 0\n")
                                        print(f"Added {len(new_edge_collisions)} new edge collisions to the CNF")


                                    # Update the CNF metadata line after appending new clauses.
                                    with open(cnf_file_path, 'r+') as file:
                                        first_line = file.readline()
                                        print('old', first_line)
                                        first_line_len = len(first_line)
                                        new_first_line = cnf_constructor.cnf.get_metadata()
                                        print('new', new_first_line)
                                        if len(new_first_line) > first_line_len:
                                            print("Rewrite whole CNF")
                                            file.write(str(cnf_constructor.cnf))
                                        else:
                                            file.seek(0)
                                            file.write(new_first_line)
                                            if len(new_first_line) < first_line_len:
                                                file.write(' ' * (first_line_len - len(new_first_line)))
                            
                                            
                                    additional_clauses_end_time = time.process_time()
                                    additional_clauses_time += additional_clauses_end_time - additional_clauses_start_time
                                    timestep_all_additional_clause_times.append(additional_clauses_end_time - additional_clauses_start_time)
                                    timestep_num_added_collisions.append(len(new_vertex_collisions)+len(new_edge_collisions))
                                    timestep_num_added_vertex_collisions.append(len(new_vertex_collisions))
                                    timestep_num_added_edge_collisions.append(len(new_edge_collisions))
                                    number_of_collision_addings += 1
                                    timestep_number_of_added_collisions += len(new_vertex_collisions)+len(new_edge_collisions)
                                    timestep_number_of_added_vertex_collision += len(new_vertex_collisions)
                                    timestep_number_of_added_edge_collision += len(new_edge_collisions)
                                    #print(f"Added {len(new_collisions)} new collisions to the CNF")

                                    if scenario['update_init_assignment']:
                                        print('update init assignment')
                                        if init_assignment_path is not None:
                                            #clean up
                                            os.remove(init_assignment_path)
                                        init_assignment_path = self.get_unique_filename(f"{self.output_dir}/"+key+"_initial_assignment.txt")
                                        if scenario['solver'] == 'probSAT':
                                            initial_assignments = solution
                                            self.write_assignments_to_file(initial_assignments, init_assignment_path)
                                            print(f"New Initial assignments saved to {init_assignment_path}")
                                        if scenario['solver'] == 'Glucose4':
                                            unique_conflicting_agents = set()
                                            for collision in vertex_collisions:
                                                unique_conflicting_agents.add(collision[0])
                                                unique_conflicting_agents.add(collision[1]) 
                                            for collision in edge_collisions:
                                                unique_conflicting_agents.add(collision[0])
                                                unique_conflicting_agents.add(collision[1])
                                            #get all paths of non conflicting agents
                                            good_paths = []
                                            for agent_id in range(len(paths)):
                                                if agent_id in unique_conflicting_agents:
                                                    good_paths.append([])
                                                else:
                                                    good_paths.append(paths[agent_id])

                                            initial_assignments = self.generate_partial_assignments(good_paths, cnf_constructor.variable_map, mdds)
                                            self.write_assignments_to_file(initial_assignments, init_assignment_path)
                                            print(f"New Initial assignments saved to {init_assignment_path}")
                                else:
                                    print(f"No new collisions found for {scenario} with starts_and_goals set {idx + 1}")
       
                        else:
                            #safety runs for Glucose if no solution is found in lazy encoding
                            if scenario['solver'] == 'Glucose4' and scenario['lazy_encoding']:
                                if not safety_run_done:
                                    print('SAFETY RUN needed: Trying Glucose4 without an initial assignment')
                                    safety_run_done = True
                                    # Remove the current initial assignment file, if any
                                    if init_assignment_path is not None:
                                        try:
                                            os.remove(init_assignment_path)
                                            print(f"Removed initial assignment file {init_assignment_path}")
                                        except Exception as e:
                                            print(f"Error removing {init_assignment_path}: {e}")
                                        init_assignment_path = None
                                    # Now let the loop continue: this iteration (or the next) will run Glucose4 without any init.
                                    # Your logging will indicate that the solver is running without an initial assignment.
                                else:
                                    print("Safety run already attempted and still no solution found. Increasing max_timesteps.")
                                    break  # Break out of the inner loop so that max_timesteps can be increased.
                          
                            elif scenario['init_assignment'] and scenario['solver'] == 'probSAT':

                                print('SAFETY RUN with new Init')
                                
                                if init_assignment_path is not None:
                                    #clean up
                                    os.remove(init_assignment_path)
                                init_assignment_path = self.get_unique_filename(f"{self.output_dir}/"+key+"_initial_assignment.txt")
                                #remove conflicting nodes and edges from one of the conflicting agents for every collision
                                
                                any_mdd_broken = False
                                protected_agents = []
                                while True:
                                    print("Pruning with protected agents:", protected_agents)
                                    pruned_mdds = self.prune_mdds(
                                        mdds,
                                        self.added_vertex_collisions,
                                        self.added_edge_collisions,
                                        protected_agents=protected_agents
                                    )
                                    
                                    broken_agents = []
                                    for agent_id, mdd in pruned_mdds.items():
                                        mdd.prune_dead_ends(max_timesteps)
                                        if self.mdd_is_broken(mdd, max_timesteps):
                                            broken_agents.append(agent_id)
                                    
                                    # Exit if no MDD is broken at all.
                                    if not broken_agents:
                                        any_mdd_broken = False
                                        print("No MDDs are broken. Exiting loop.")
                                        break

                                    # Identify new broken agents (ones not already protected).
                                    new_broken = [agent for agent in broken_agents if agent not in protected_agents]
                                    
                                    # If all broken MDDs were already protected, then exit the loop.
                                    if not new_broken:
                                        any_mdd_broken = True
                                        print("All broken MDDs were already protected. Exiting loop.")
                                        break
                                    else:
                                        print("New broken agents detected:", new_broken)
                                        protected_agents.extend(new_broken)

                                print("Loop ended. any_mdd_broken =", any_mdd_broken)

                                

                                if not any_mdd_broken:
                                    initial_assignments = self.safety_sample_paths(pruned_mdds , cnf_constructor)
                                    self.write_assignments_to_file(initial_assignments, init_assignment_path)
                                    print(f"New Initial assignments saved to {init_assignment_path}")
                                    edgepaths = self.parse_solver_output_only_edges(initial_assignments, cnf_constructor.variable_map)
                                    
                                    paths = self.parse_solver_output(initial_assignments, cnf_constructor.variable_map)

                                    #first vertex collisions
                                    collisions = self.find_collisions(paths)
                                    
                                    for collision in collisions:
                                        if collision in self.added_vertex_collisions:
                                            print('innit assignment has old vertex collision', collision)
                                    
                                    #then edge swap collisions
                                    swap_conflicts = self.find_edge_collisions(initial_assignments, cnf_constructor.variable_map)
                                    for swap_conflict in swap_conflicts:
                                        if swap_conflict in self.added_edge_collisions:
                                            print('innit assignment has old edge colliison', swap_conflict)

                                else:
                                    print("MDDs are broken, no new init assignment created, have to increase max timesteps")
                                    init_assignment_path = None

                                    break
                               
                            else:
                                print(safety_run_done, "Saftey Runs also Failed")
                                print(f"No solution found in this iteration with max_timesteps: {max_timesteps}")
                                break

                    # Store results
                    #key = f"starts_and_goals_{idx + 1}_lazy_{scenario['lazy_encoding']}_init_{scenario['init_assignment']}_timesteps_{max_timesteps}"

                    total_time = time.process_time() - start_time
                    timestep_total_time = time.process_time() - timestep_total_time_start

                    all_timesteps_num_added_collisions.append(timestep_num_added_collisions)
                    all_timesteps_num_added_vertex_collisions.append(timestep_num_added_vertex_collisions)
                    all_timesteps_num_added_edge_collisions.append(timestep_num_added_edge_collisions)
                    all_timesteps_num_collision_addings.append(number_of_collision_addings)

                    
                      
                    individual_timesteps_total_time.append(timestep_total_time)

                    #same for all timesteps, is only done once for all scenarios timesteps
                    distance_matrix_time = distance_matrix_time
                    
                    individual_timesteps_all_sat_solver_times.append(timestep_all_solver_times)
                    individual_timesteps_sum_of_all_sat_solver_times.append(sum(timestep_all_solver_times))

                    sum_of_all_timesteps_additional_clauses_time += sum(timestep_all_additional_clause_times)
                    individual_timesteps_additional_clauses_time.append(timestep_all_additional_clause_times)

                    sum_of_all_timesteps_num_added_collisions += timestep_number_of_added_collisions
                    #each timestep has multipl iterations of adding collisions each iteration is a list each timestep is a list of iteration lists
                    individual_timesteps_num_added_collisions.append(timestep_num_added_collisions)
                    individual_timesteps_num_added_vertex_collisions.append(timestep_num_added_vertex_collisions)
                    individual_timesteps_num_added_edge_collisions.append(timestep_num_added_edge_collisions)

                    individual_timesteps_stdouts.append(timestep_stdouts_to_save)
                    individual_timesteps_number_of_variables.append(timestep_number_of_variables)
                    individual_timesteps_number_of_literals.append(timestep_number_of_literals)
                    individual_timesteps_number_of_clauses.append(timestep_number_of_clauses)
                    individual_timesteps_max_clause_length.append(timestep_max_clause_length)

                    sum_of_all_timesteps_num_flips += sum(timestep_num_flips)
                    individual_timesteps_num_flips.append(timestep_num_flips)
                    sum_of_all_timesteps_tries += sum(timestep_number_of_tries)
                    individual_timesteps_tries.append(timestep_number_of_tries)

                    individual_timesteps_avg_flips_per_variable.append(timestep_avg_flips_per_variable)

                    individual_timesteps_avg_flips_per_clause.append(timestep_avg_flips_per_clause)

                    individual_timesteps_flips_per_sec.append(timestep_flips_per_sec)

                    #individual_timesteps_initial_clauses_satisfied.append(timestep_initial_clauses_satisfied)
                    #individual_timesteps_initial_clauses_satisfied_percentage.append(timestep_initial_clauses_satisfied_percentage)


                    sum_of_all_timesteps_cpu_time += sum(timestep_cpu_time)
                    individual_timesteps_cpu_time.append(timestep_cpu_time)
                    individual_timesteps_sum_of_cpu_time.append(sum(timestep_cpu_time))

                    sum_of_all_timesteps_loading_time += sum(timestep_loading_time)
                    individual_timesteps_loading_time.append(timestep_loading_time)
                    individual_timesteps_sum_of_loading_time.append(sum(timestep_loading_time))
                    

                    sum_of_all_timesteps_solving_time += sum(timestep_solving_time)
                    individual_timesteps_solving_time.append(timestep_solving_time)
                    individual_timesteps_sum_of_solving_time.append(sum(timestep_solving_time))

                    sum_of_all_timesteps_solving_time_s += sum(timestep_solving_time_s)
                    individual_timesteps_solving_time_s.append(timestep_solving_time_s)
                    individual_timesteps_sum_of_solving_time_s.append(sum(timestep_solving_time_s))

                    #Pysat stats
                    sum_of_all_timesteps_num_conflicts += sum(timestep_num_conflicts)
                    individual_timesteps_num_conflicts.append(timestep_num_conflicts)

                    sum_of_all_timesteps_num_decisions += sum(timestep_num_decisions)
                    individual_timesteps_num_decisions.append(timestep_num_decisions)

                    sum_of_all_timesteps_num_propagations += sum(timestep_num_propagations)
                    individual_timesteps_num_propagations.append(timestep_num_propagations)

                    sum_of_all_timesteps_num_restarts += sum(timestep_num_restarts)
                    individual_timesteps_num_restarts.append(timestep_num_restarts)

                    sum_of_all_timestep_solving_time_s += sum(timestep_solving_time_s)
                    individual_timestep_solving_time_s.append(timestep_solving_time_s)

                    sum_of_all_timestep_loading_time += sum(timestep_loading_time)
                    individual_timestep_loading_time.append(timestep_loading_time)

                    sum_of_all_timestep_solving_time += sum(timestep_solving_time)
                    individual_timestep_solving_time.append(timestep_solving_time)

                    
                    #clean up CNF and init assignment txt files
                    os.remove(cnf_file_path)
                    if scenario['lazy_encoding']:
                        os.remove(original_cnf_file_path)
                    if scenario['init_assignment']:
                        if init_assignment_path is not None:
                            os.remove(init_assignment_path)
                            init_assignment_path = None
                    if scenario['update_init_assignment']:
                        if init_assignment_path is not None:
                            os.remove(init_assignment_path)


                    if solution_found:
                        print(f" {all_timesteps_num_added_collisions} collisions added in {all_timesteps_num_collision_addings} iterations")
                        print(f"Found solution for {scenario} with starts_and_goals set {idx + 1} and max_timesteps: {max_timesteps}")
                        print('----------------------------------------------------------------------------------------------------------')
                        break
                    else:
                        # Increase max_timesteps
                        max_timesteps += 1
                        #important so collisions are empty again
                        #if not empty then the next timestep will cant add collisions from the previous timestep to the new cnf
                        added_collisions = []
                        self.added_vertex_collisions = []
                        self.added_edge_collisions = []
                        #reset all timestep stats
                        #need resest after timestep
                        timestep_all_solver_times = []
                        timestep_all_additional_clause_times = []
                        timestep_num_added_collisions = []
                        timestep_num_added_vertex_collisions = []
                        timestep_num_added_edge_collisions = []
                        timestep_number_of_added_collisions = 0
                        timestep_stdouts_to_save = []
                        timestep_number_of_variables = []
                        timestep_number_of_literals = []
                        timestep_number_of_clauses = []
                        timestep_max_clause_length = []
                        timestep_num_flips = []
                        timestep_avg_flips_per_clause = []
                        timestep_flips_per_sec = []
                        timestep_cpu_time = []
                        timestep_loading_time = []
                        timestep_solving_time = []
                        timestep_solving_time_s = []
                        #Pysat stats
                        timestep_num_conflicts = []
                        timestep_num_decisions = []
                        timestep_num_propagations = []
                        timestep_num_restarts = []
                        timestep_solving_time_s = []
                        timestep_loading_time = []
                        timestep_solving_time = []
                        

                        print(f"Incrementing max_timesteps to {max_timesteps} for {scenario} with starts_and_goals set {idx + 1}")
                
                if scenario['solver'] == 'probSAT':
                    new_row ={
                        'Set': idx + 1,
                        'Scenario': key,
                        'Solver': scenario['solver'],
                        'Map Name': self.map_name,
                        'Lazy Encoding': scenario['lazy_encoding'],
                        'Initial Assignment': scenario['init_assignment'],
                        'Update Initial Assignment': scenario['update_init_assignment'],
                        'Zero Init': scenario['zero_init'],
                        'Max Timesteps': max_timesteps_list,
                        'Initial Max Timesteps': initial_max_timesteps,
                        'Total Time (s)': total_time,
                        'Timesteps Total Time (s)': individual_timesteps_total_time,
                        'Distance Matrix Time (s)': distance_matrix_time,
                        'MDD Time (s)': mdd_time,
                        'Timesteps MDD Time (s)': mdd_time_list,
                        'Base CNF Construction Time (s)': base_cnf_construction_time,
                        'Timesteps Base CNF Construction Time (s)': base_cnf_construction_time_list,
                        'CNF Construction Time (s)': cnf_construction_time,
                        'Timesteps CNF Construction Time (s)': cnf_construction_time_list,
                        'Total SAT Solver Time (s)': total_sat_solver_time,
                        'Timestep Total Solver Times (s)': individual_timesteps_sum_of_all_sat_solver_times,
                        'Timestep All Solver Times (s)': individual_timesteps_all_sat_solver_times,
                        'Timestep Initial Assignment Time (s)': initial_assignment_time_list,
                        'Initial Assignment Time (s)': initial_assignment_time,
                        'Additional Lazy Time (s)': sum_of_all_timesteps_additional_clauses_time,
                        'Timestep Additional Lazy Time (s)': individual_timesteps_additional_clauses_time,
                        'Number of Added Collisions': sum_of_all_timesteps_num_added_collisions,
                        'Timestep Number of Added Collisions': individual_timesteps_num_added_collisions,
                        'Timestep Number of Added Vertex Collisions': individual_timesteps_num_added_vertex_collisions,
                        'Timestep Number of Added Edge Collisions': individual_timesteps_num_added_edge_collisions,
                        'Number of Variables':individual_timesteps_number_of_variables,
                        'Number of Clauses': individual_timesteps_number_of_clauses,
                        'Max Flips': max_flips,
                        'Max Tries': max_tries,
                        'Stdout': individual_timesteps_stdouts,
                        'Number of Variables': individual_timesteps_number_of_variables,
                        'Number of Literals': individual_timesteps_number_of_literals,
                        'Number of Clauses': individual_timesteps_number_of_clauses,
                        'Max Clause Length': individual_timesteps_max_clause_length,
                        #'Initial Clauses Satisfied': individual_timesteps_initial_clauses_satisfied,
                        #'Initial Clauses Satisfied Percentage': individual_timesteps_initial_clauses_satisfied_percentage,
                        'Num Flips': sum_of_all_timesteps_num_flips,
                        'Timestep Num Flips': individual_timesteps_num_flips,
                        'Num Tries': sum_of_all_timesteps_tries,
                        'Timestep Num Tries': individual_timesteps_tries,
                        'Avg Flips per Variable': individual_timesteps_avg_flips_per_variable,
                        'Avg Flips per Clause': individual_timesteps_avg_flips_per_clause,
                        'Flips per Sec': individual_timesteps_flips_per_sec,
                        'CPU Time': sum_of_all_timesteps_cpu_time,
                        'Timestep CPU Time': individual_timesteps_cpu_time,
                        'Timestep Sum of CPU Time': individual_timesteps_sum_of_cpu_time,
                        'Loading Time': sum_of_all_timesteps_loading_time,
                        'Timestep Loading Time': individual_timesteps_loading_time,
                        'Timestep Sum of Loading Time': individual_timesteps_sum_of_loading_time,
                        'Solving Time': sum_of_all_timesteps_solving_time,
                        'Timestep Solving Time': individual_timesteps_solving_time,
                        'Timestep Sum of Solving Time': individual_timesteps_sum_of_solving_time,
                        'Solving Time (s)': sum_of_all_timesteps_solving_time_s,
                        'Timestep Solving Time (s)': individual_timesteps_solving_time_s,
                        'Timestep Sum of Solving Time (s)': individual_timesteps_sum_of_solving_time_s,
                        'Timestep '
                        'Paths': last_paths
                        #'CNF': str(cnf_constructor.cnf) if scenario['lazy_encoding'] else ""
                    }
                    new_row_df = pd.DataFrame([new_row])
                    probSAT_results_df = pd.concat([probSAT_results_df, new_row_df], ignore_index=True)
                    # Save results to a CSV file
                    probSAT_results_df.to_csv(self.output_dir+"/"+results_key, index=False)

                if scenario['solver'] == 'Glucose4':   
                    new_row = {
                        'Set': idx + 1,
                        'Scenario': key,
                        'Solver': scenario['solver'],
                        'Map Name': self.map_name,
                        'Lazy Encoding': scenario['lazy_encoding'],
                        'Initial Assignment': scenario['init_assignment'],
                        'Update Initial Assignment': scenario['update_init_assignment'],
                        'Zero Init': scenario['zero_init'],
                        'Max Timesteps': max_timesteps_list,
                        'Initial Max Timesteps': initial_max_timesteps,
                        'Total Time (s)': total_time,
                        'Timesteps Total Time (s)': individual_timesteps_total_time,
                        'Distance Matrix Time (s)': distance_matrix_time,
                        'MDD Time (s)': mdd_time,
                        'Timesteps MDD Time (s)': mdd_time_list,
                        'Base CNF Construction Time (s)': base_cnf_construction_time,
                        'Timesteps Base CNF Construction Time (s)': base_cnf_construction_time_list,
                        'CNF Construction Time (s)': cnf_construction_time,
                        'Timesteps CNF Construction Time (s)': cnf_construction_time_list,
                        'Total SAT Solver Time (s)': total_sat_solver_time,
                        'Timestep Total Solver Time (s)': individual_timesteps_sum_of_all_sat_solver_times,
                        'Timestep All Solver Time (s)': individual_timesteps_all_sat_solver_times,
                        'Timestep Initial Assignment Time (s)': initial_assignment_time_list,
                        'Initial Assignment Time (s)': initial_assignment_time,
                        'Additional Lazy Time (s)': sum_of_all_timesteps_additional_clauses_time,
                        'Timestep Additional Lazy Time (s)': individual_timesteps_additional_clauses_time,
                        'Number of Added Collisions': sum_of_all_timesteps_num_added_collisions,
                        'Timestep Number of Added Collisions': individual_timesteps_num_added_collisions,
                        'Timestep Number of Added Vertex Collisions': individual_timesteps_num_added_vertex_collisions,
                        'Timestep Number of Added Edge Collisions': individual_timesteps_num_added_edge_collisions,
                        'Number of Variables':individual_timesteps_number_of_variables,
                        'Number of Clauses': individual_timesteps_number_of_clauses,
                        'Timestep Conflicts': individual_timesteps_num_conflicts,
                        'Conflicts': sum_of_all_timesteps_num_conflicts,
                        'Decisions': sum_of_all_timesteps_num_decisions,
                        'Timestep Decisions': individual_timesteps_num_decisions,
                        'Propagations': sum_of_all_timesteps_num_propagations,
                        'Timestep Propagations': individual_timesteps_num_propagations,
                        'Restarts':sum_of_all_timesteps_num_restarts,
                        'Timestep Restarts': individual_timesteps_num_restarts,
                        'Solving Time (s)': sum_of_all_timestep_solving_time_s,
                        'Timestep Solving Time (s)': individual_timestep_solving_time_s,
                        'Loading Time': sum_of_all_timesteps_loading_time,
                        'Timestep Loading Time': individual_timesteps_loading_time,
                        'Timestep Loading Time': individual_timesteps_loading_time,
                        'Timestep Sum of Loading Time': individual_timesteps_sum_of_loading_time,
                        'Loading Time (s)': sum_of_all_timestep_loading_time,
                        'Timestep Loading Time (s)': individual_timestep_loading_time,
                        'Solving Time': sum_of_all_timestep_solving_time,
                        'Timestep Solving Time': individual_timestep_solving_time,
                        'Paths': last_paths
                        #'CNF': str(cnf_constructor.cnf) if scenario['lazy_encoding'] else ""
                    }
                    new_row_df = pd.DataFrame([new_row])
                    pysat_results_df = pd.concat([pysat_results_df, new_row_df], ignore_index=True)
                    # Save results to a CSV file
                    pysat_results_df.to_csv(self.output_dir+"/"+results_key_pysat, index=False)
                
            print(f"Finished with starts_and_goals set {idx + 1}")
            #clean up base cnf
            if created_base_cnf:
                os.remove(base_cnf_file_path)
             

       
        
            
        
        return results, key












