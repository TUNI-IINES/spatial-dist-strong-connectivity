from nebosim.model import Dynamic
import numpy as np



# Define new dynamic for a single integrator with communication disc
class SI_discComm(Dynamic):
    """
    A single integrator (kinematic) model for a robot in 3 dimension 
    with controllable communication radius
    Also ofter refered as point dynamics
    State: Position as pos = [q_x q_y q_z], communication radius comR = r 
    Input: Velocity Input as vel = [u_x u_y u_z], radius change speed vR = v
    Dynamics: 
        dot(pos) = vel
        dot(comR) = vR
    """
    
    def __init__(self, dt, *, 
                 init_pos=np.array([0., 0., 0.]), init_comR = 0.,
                 init_vel=np.array([0., 0., 0.]), init_vR = 0.,
                 robot_ID = None):
        """
        model_name: the class name, 'SI_discComm'
        :param dt: (default) time sampling for update
        :param init_pos: robot's initial position in numpy array 
        :param init_comR: robot's initial communication radius in float
        :param init_vel: robot's initial velocity input in numpy array 
        :param init_vR: robot's initial communication radius speed changes in float
        :param robot_ID: to identify different robot for multi-robot scenarios
        """
        super().__init__(dt) 
        self.robot_ID = robot_ID

        self.state["pos"], self.state["comR"] = init_pos, init_comR
        self.input["vel"], self.input["vR"] = init_vel, init_vR
        self.dot_state["pos"] = np.array([0., 0., 0.])
        self.dot_state["comR"] = 0.

    def compute_dot_state(self):
        """
        Dynamics: 
            dot(pos) = vel
            dot(comR) = vR
        """
        self.dot_state["pos"] = self.input["vel"]
        self.dot_state["comR"] = self.input["vR"]

    def set_input(self, vel, vR): 
        """
        :param vel: velocity input in numpy array 
        :param vR: communication radius speed changes in float
        """        
        self.input["vel"] = vel
        self.input["vR"] = vR


# TODO: add more description on the methods
class DistributedConnectivity:
    """
    A class for estimating:
    - accessible nodes from itself (node)
    - strongly connected components it belongs to (set_SCC)
    - changes in the network
    """
    def __init__(self, robot_ID, bar_n, list_in_neighbours = []):
        """
        :param robot_ID: to identify different robot for multi-robot scenarios
        :param bar_n: maximum number of agent that can be considered, robot_ID < bar_n
        """
        assert (robot_ID < bar_n), f"robot_ID={robot_ID} should be less than bar_n={bar_n}"

        self.robot_ID = robot_ID
        self.bar_n = bar_n
        self.initialize_x()

        self.m = 0
        self.m_prev = 0
        self.list_in_neighbours = list_in_neighbours

    def initialize_x(self):
        """
        Reset the estimation for state x (accessible nodes)
        """
        self.x = np.zeros([self.bar_n, 1])
        self.x[self.robot_ID] = 1
        self.list_SCC = None
        self.list_preSCC = None

        # Saved variable for compute_control_position_radius 
        self.id_neigh_to_reach = None


    def maxcons_incoming_states(self, incoming_x, incoming_m): 
        """
        Maximum consensus computation
        :param incoming_x: other node's state x 
        :param incoming_m: other node's state m (time stamp of last network changes)
        """
        for i in range(self.bar_n): self.x[i] = max(self.x[i], incoming_x[i])
        self.m = max(self.m, incoming_m)

    def process_information(self, iteration_number, list_in_neighbours):
        """
        Maximum consensus computation
        :param incoming_x: other node's state x 
        :param incoming_m: other node's state m (time stamp of last network changes)
        """
        if self.list_in_neighbours != list_in_neighbours: # Changes in neighbours
            self.m = iteration_number
            self.list_in_neighbours = list_in_neighbours

        if self.m > self.m_prev: # Changes happen in the network
            self.initialize_x()
            self.m_prev = self.m

        else: # no changes in the network
            accessible_nodes = list(np.where(self.x > 0)[0])
            information_number = len(accessible_nodes)
            self.x[self.robot_ID] = information_number

            if iteration_number >= self.m + 3*self.bar_n:
               if self.list_SCC is None: # Estimation can be done
                   self.list_SCC = list(np.where(self.x == information_number)[0])
                   self.list_preSCC = list(np.where((self.x > 0) & (self.x < information_number))[0]) 

                   print('iter:', iteration_number, 'ID:', self.robot_ID, ' SCC:', self.list_SCC, ' preSCC:', self.list_preSCC)
               # else: There is no need to do estimation anymore, as the graph is not changing
        

    def compute_control_radius(self):
        """
        According to the known information of the network
        Compute the required increase in the communication range to ensure strong connectivity
        Currently defaulted to 0.1m/s increase if needed.
        """
        # Increase communication range to ensure strong connectivity
        c = 0.
        if (self.list_SCC is not None):
            in_neigh_to_connect = list(set(self.list_preSCC) & set(self.list_in_neighbours))
            if len(in_neigh_to_connect) > 0: c = 0.1

        return c


    def compute_control_position_radius(self, i_pos, i_comR, dict_neigh_pos, dict_neigh_comR):
        """
        According to the known information of the network
        Compute the required:
        - velocity input to control its position, and 
        - increase in the communication range 
        to ensure strong connectivity
        
        :param i_pos: this robot's current position
        :param i_comR: this robot's current communication range
        :param dict_neigh_pos: dictinoary of neighbour robot's position
        :param dict_neigh_comR: dictinoary of neighbour robot's communication range
        """
        # Increase communication range to ensure strong connectivity
        v, c = np.zeros(3), 0.

        # Maintain previous nominal control until the targeted neighbors is inside a certain epsilon from i's comR 
        if self.id_neigh_to_reach is not None:
            epsilon = 0.1
            dist_ir = np.linalg.norm(dict_neigh_pos[self.id_neigh_to_reach] - i_pos)
            if dist_ir < i_comR - epsilon: 
                self.id_neigh_to_reach = None

        # Decide on the next in_neighbour outside of SCC to reach
        if (self.id_neigh_to_reach is None) and (self.list_SCC is not None):
            in_neigh_to_connect = list(set(self.list_preSCC) & set(self.list_in_neighbours))

            if len(in_neigh_to_connect) > 0: 
                # Filter shortest in_neigh_to_connect
                shortest_id = None
                min_dist = 1000 # arbitrary large number compared to any node's communication radius
                for j in in_neigh_to_connect:
                    dist_ij = np.linalg.norm(dict_neigh_pos[j] - i_pos)
                    if dist_ij < min_dist: shortest_id, min_dist = j, dist_ij
                # Assign in_neighbors to connect
                self.id_neigh_to_reach = shortest_id
                # Will be computed in the next iteration

        v_nom = np.zeros(3)
        if self.id_neigh_to_reach is not None:
            # Compute nominal proportional control with saturation
            # kP, sat = 1., 0.05
            vec = dict_neigh_pos[self.id_neigh_to_reach] - i_pos

            # Time-varying P-gain control
            v0, beta = 0.1, 1.
            norm_er_pos = np.linalg.norm(vec)
            kP = v0*(1 - np.exp(-beta*norm_er_pos))/norm_er_pos

            v_nom = kP*vec
            # norm = np.hypot(v_nom[0], v_nom[1])
            # if norm > sat: v_nom = sat* v_nom / norm # max 

        v_edge = np.zeros(3)
        # Edge Preserving Control
        for k in self.list_in_neighbours:
            vect = dict_neigh_pos[k] - i_pos
            dist_ik = np.linalg.norm(vect)
            Rs = dict_neigh_comR[k]

            k_ik = 0.4
            ups_ik = 0
            lb, ub = Rs-0.1, Rs
            if dist_ik > lb:
                if dist_ik > ub: ups_ik = 1
                else: 
                    ups_ik = 1 - np.exp( - ((dist_ik - lb)/(ub - lb))**2)
            v_edge += k_ik*ups_ik*vect

        v = v_nom + v_edge

        # Decision on when to increase communication
        v_norm = np.hypot(v[0], v[1])
        # if v_norm > 0.5: v = 0.5* v / v_norm # max 
        v_th = 0.01
        if (self.id_neigh_to_reach is not None) and (v_norm <= v_th):
            c = 0.1 # increase communication range

        return v, c


def evaluateAdjacency(list_SI_discComm, bar_n):
    """
    Given a list of robots with a certain communication range,
    evaluate the resulting adjacency matrix of the proximity graphs.

    A_ij is 1 for an edge from j to i.
    For each row i, the column j is 1 if j is in-neighbours if i

    :param list_SI_discComm: list of instances from SI_discComm class
    :param bar_n: maximum number of agent that can be considered, robot_ID < bar_n
    """
    A = np.zeros([bar_n, bar_n])
    n = len(list_SI_discComm)

    edge_list = {}

    for robot_i in list_SI_discComm:
        pos_i, comR_i = robot_i.state["pos"], robot_i.state["comR"]
        i = robot_i.robot_ID
        
        for robot_j in list_SI_discComm:
            j = robot_j.robot_ID
            if j != i:
                pos_j = robot_j.state["pos"]
                dist_ij = np.linalg.norm(pos_j - pos_i)
                            
                if dist_ij < comR_i:
                    A[j, i] = 1
                    edge_list[f'{j}_{i}'] = {'from': pos_i, 'to': pos_j}

    return A, edge_list
