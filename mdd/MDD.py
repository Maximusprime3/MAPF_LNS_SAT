import heapq
import random
import collections
from collections import defaultdict

class MDDNode:
    def __init__(self, position, time_step):
        self.position = position
        self.time_step = time_step
        self.children = []

    def add_child(self, child):
        self.children.append(child)

class MDD:
    def __init__(self):
        self.levels = collections.defaultdict(list)

    def add_node(self, node):
        self.levels[node.time_step].append(node)

    def remove_node(self, node):
        """
        Remove this 'node' from the MDD entirely:
        1) Remove it from self.levels[node.time_step]
        2) Remove it from all parents' .children references
        this leaves the subtree intact but unreachable
        """
        time_level = node.time_step

        # 1) Remove from that level's node list
        if node in self.levels[time_level]:
            self.levels[time_level].remove(node)

        # 2) Remove from parents' children
        if time_level > 0:
            parent_list = self.levels[time_level - 1]
            for pnode in parent_list:
                if node in pnode.children:
                    pnode.children.remove(node)

    def prune_dead_ends(self, max_timesteps):
        """
        Iteratively remove nodes that have no children unless they are 
        in the final layer (time_step == max_timesteps).

        We repeat until no more nodes are removed in a pass, because 
        removing one node can make its parent a new dead-end.
        """
        changed = True
        while changed:
            changed = False
            # For each layer from 0 to max_timesteps-1,
            # remove nodes that have zero children
            for t in range(max_timesteps):
                if t not in self.levels:
                    continue
                to_remove = []
                for node in self.levels[t]:
                    if len(node.children) == 0:
                        to_remove.append(node)
                if to_remove:
                    for node in to_remove:
                        self.remove_node(node)
                    changed = True

    def is_broken(self, max_timesteps):
        """
        Checks whether the MDD is broken – that is, whether any layer
        from time 0 to max_timesteps is empty.
        
        :param max_timesteps: The maximum time index that a full path should reach.
        :return: True if any layer is empty (i.e. the MDD is broken), otherwise False.
        """
        # We expect to have nodes at every time from 0 to max_timesteps inclusive.
        for t in range(max_timesteps + 1):
            if t not in self.levels or len(self.levels[t]) == 0:
                return True
        return False

    def copy(self):
        """
        Create and return a DEEP copy of this MDD, with new MDDNode
        objects but the same positions, time_steps, and child relationships.
        """
        new_mdd = MDD()
        old_to_new = {}

        # (A) Create all new nodes, same position/time_step, but empty children
        for t, node_list in self.levels.items():
            for old_node in node_list:
                new_node = MDDNode(old_node.position, old_node.time_step)
                old_to_new[old_node] = new_node
                new_mdd.add_node(new_node)

        # (B) Rebuild children references
        for t, node_list in self.levels.items():
            for old_node in node_list:
                new_node = old_to_new[old_node]
                for old_child in old_node.children:
                    new_child = old_to_new[old_child]
                    new_node.children.append(new_child)

        return new_mdd

    def get_nodes_at_level(self, level):
        return self.levels[level]
    
    def sample_random_path(self, start=None):
        path = []
        if not start:
            start = self.levels[0][0]
        
        current_node = start

        while current_node:
            path.append(current_node.position)

            if not current_node.children:
                break  # No children left, stop

            # Randomly choose the next child
            next_node = random.choice(current_node.children)

            current_node = next_node

        return path

    def __str__(self):
        # Output a human-readable representation of the MDD
        output = []
        for level, nodes in self.levels.items():
            node_positions = [str(node.position) for node in nodes]
            output.append(f"Time Step {level}: {', '.join(node_positions)}")
        return "\n".join(output)

class MDDConstructor:
    def __init__(self, grid, start, goal, max_timesteps = None, distances=None):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.rows = len(grid)
        self.cols = len(grid[0]) if grid else 0
        self.max_timesteps = max_timesteps
        self.mdd = MDD()
        self.distances = distances if distances else {}

        # Validate grid dimensions
        if self.rows == 0:
            raise ValueError("Grid cannot be empty")
        
        # Validate start position
        if (start[0] < 0 or start[0] >= self.rows or 
            start[1] < 0 or start[1] >= self.cols):
            raise ValueError("Start position is out of bounds")
        if (self.grid[start[0]][start[1]] != '.' and 
            self.grid[start[0]][start[1]] != 'G'):
            raise ValueError("Start position is not on a free cell")
        
        # Validate goal position
        if (goal[0] < 0 or goal[0] >= self.rows or 
            goal[1] < 0 or goal[1] >= self.cols):
            raise ValueError("Goal position is out of bounds")
        if (self.grid[goal[0]][goal[1]] != '.' and 
            self.grid[goal[0]][goal[1]] != 'G'):
            raise ValueError("Goal position is not on a free cell")

        # Dictionary to store nodes based on (position, time_step) keys
        self.nodes = defaultdict(dict)

    # Get neighbors of a position that are not obstacles aka '.'
    def get_neighbors(self, pos):
        x, y = pos
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right
        neighbors = []
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.rows and 0 <= ny < self.cols and (self.grid[nx][ny] == '.' or self.grid[nx][ny] == 'G'):
                neighbors.append((nx, ny))
        return neighbors

    def compute_all_distances(self):
        # Use modified Dijkstra to compute minimum distances to all nodes
        heap = [(0, self.goal)]
        while heap:
            distance, position = heapq.heappop(heap)
            if position in self.distances:
                continue
            self.distances[position] = distance
            for neighbor in self.get_neighbors(position):
                if neighbor not in self.distances:
                    heapq.heappush(heap, (distance + 1, neighbor))
        return self.distances
    
    def construct_mdd(self):
        if not self.distances:  # Compute only if not provided
            self.distances = self.compute_all_distances()
        queue = [(self.start, 0)]
        visited = set()

        # Handle scenario where max_timesteps is not defined
        if self.max_timesteps is None:
            self.max_timesteps = self.distances.get(self.start, 0)

        # Initialize the starting node and store it
        start_node = MDDNode(self.start, 0)
        self.nodes[0][self.start] = start_node
        self.mdd.add_node(start_node)

        while queue:
            position, current_time_step = queue.pop(0)
            if (position, current_time_step) in visited:
                continue
            visited.add((position, current_time_step))

            node = self.nodes[current_time_step][position]

            if current_time_step < self.max_timesteps:
                # Extend to neighbors and the same position (waiting)
                for next_pos in self.get_neighbors(position) + [position]:
                    next_time_step = current_time_step + 1

                    # Check distance to goal at this next position
                    if self.distances.get(next_pos, float('inf')) + next_time_step <= self.max_timesteps:
                        # Ensure the node exists at the next level
                        if next_pos not in self.nodes[next_time_step]:
                            next_node = MDDNode(next_pos, next_time_step)
                            self.nodes[next_time_step][next_pos] = next_node
                            self.mdd.add_node(next_node)
                            queue.append((next_pos, next_time_step))
                        else:
                            next_node = self.nodes[next_time_step][next_pos]

                        # Add the existing node as a child
                        node.add_child(next_node)

        return self.mdd
    
    def old_construct_mdd(self):
        if not self.distances:  # Compute only if not provided
            self.distances = self.compute_all_distances()
        queue = [(self.start, 0)]
        visited = set()

        # Handle scenario where max_timesteps is not defined
        if self.max_timesteps is None:
            self.max_timesteps = self.distances.get(self.start, 0)

        while queue:
            position, current_time_step = queue.pop(0)
            if (position, current_time_step) in visited:
                continue
            visited.add((position, current_time_step))
            if current_time_step > self.max_timesteps:
                continue

            node = MDDNode(position, current_time_step)
            self.mdd.add_node(node)
            
            if current_time_step < self.max_timesteps:
                # Extend to neighbors and the same position (waiting)
                for next_pos in self.get_neighbors(position) + [position]:
                    if self.distances.get(next_pos, float('inf')) + current_time_step + 1 <= self.max_timesteps:
                        node.add_child(MDDNode(next_pos, current_time_step + 1))
                        queue.append((next_pos, current_time_step + 1))

        return self.mdd