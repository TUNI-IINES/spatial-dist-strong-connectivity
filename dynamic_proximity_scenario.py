from disc_connectivity import SI_discComm, DistributedConnectivity, evaluateAdjacency, draw2DPointSIDisc
import numpy as np
import matplotlib.pyplot as plt        
# from qpsolvers import solve_qp

if __name__ == '__main__':
    Ts = 0.02 # seconds
    list_robots = [
        SI_discComm(Ts, robot_ID=1, init_pos=np.array([ 1.5, -0.5, 0]), init_comR=0.8),
        SI_discComm(Ts, robot_ID=2, init_pos=np.array([-1.2, -0.5, 0]), init_comR=1.),
        SI_discComm(Ts, robot_ID=3, init_pos=np.array([ 0.,   1.,  0]), init_comR=0.6),
        SI_discComm(Ts, robot_ID=4, init_pos=np.array([ 1.5, -1.5, 0]), init_comR=0.8),
        # no robot_ID=5
        SI_discComm(Ts, robot_ID=6, init_pos=np.array([ 0.,  -1.5, 0]), init_comR=0.6),
        SI_discComm(Ts, robot_ID=7, init_pos=np.array([ 1.,  -1.,  0]), init_comR=0.8),
        SI_discComm(Ts, robot_ID=8, init_pos=np.array([ 0.5,  1.5, 0]), init_comR=1.),
        SI_discComm(Ts, robot_ID=9, init_pos=np.array([ 0.,  -0.2, 0]), init_comR=1.4),
        # no robot_ID=10
    ]
    bar_n = 10

    # Create mapping to locate list index from robot id
    map_id_list = {}
    for id, robot_i in enumerate(list_robots):
        map_id_list[robot_i.robot_ID] = id

    adj_mat, edge_list = evaluateAdjacency(list_robots, bar_n)
    # Initiate list of controller for each robot
    # ENSURE that the ordering follow the list_robots properly
    list_conn_est = []
    for robot in list_robots:
        list_conn_est.append( DistributedConnectivity(robot.robot_ID, bar_n, list(np.where(adj_mat[robot.robot_ID] > 0)[0])) )

    # Define Iterations and times
    it, time = 0, 0.
    Tmax = 10

    # Initialize plot
    fig = plt.figure(1)
    plotter = draw2DPointSIDisc( plt.gca() )
    plotter.plot_dist_connectivity(list_robots, edge_list, bar_n, time)
    # TODO: resolve map Id with list index 
    # or ENSURE that the robot_ID follow the list index properly

    while time < Tmax:
        # Recompute adjacency matrix
        # The connection can changes over time due to changes of states
        adj_mat, edge_list = evaluateAdjacency(list_robots, bar_n)

        # Distributed computation for each robot
        for id, robot_i in enumerate(list_robots):
            conn_est_i = list_conn_est[id]
            i = robot_i.robot_ID

            i_neigh = list(np.where(adj_mat[i] > 0)[0])
            for j in i_neigh:
                conn_est_j = list_conn_est[ map_id_list[j] ]
                conn_est_i.maxcons_incoming_states( conn_est_j.x, conn_est_j.m )

            conn_est_i.process_information(it, i_neigh)


            # TODO: change the controller into a class
            # Add the case that the node keep going towards the new neighbor until they are within radius-epsilon 
            # AND feedback is retrieved

            # Increase communication range to ensure strong connectivity
            v = np.zeros(3)
            c = 0.
            if (conn_est_i.list_SCC is not None):
                in_neigh_to_connect = list(set(conn_est_i.list_preSCC) & set(i_neigh))

                v_nom = np.zeros(3)
                if len(in_neigh_to_connect) > 0: 
                    id_neigh_to_reach = in_neigh_to_connect[0]
                    robot_j = list_robots[ map_id_list[id_neigh_to_reach] ]
                    v_nom = (robot_j.state['pos'] - robot_i.state['pos'])

                    norm = np.hypot(v_nom[0], v_nom[1])
                    if norm > 0.5: v_nom = 0.5* v_nom / norm # max 

                v_edge = np.zeros(3)
                # Edge Preserving Control
                for k in i_neigh:
                    robot_k = list_robots[ map_id_list[k] ]
                    vect = robot_k.state['pos'] - robot_i.state['pos']
                    dist_ik = np.linalg.norm(vect)
                    Rs = robot_k.state['comR']

                    # k_ik = (2*Rs - dist_ik)/((Rs - dist_ik)**2)
                    k_ik = 1
                    ups_ik = 0
                    lb, ub = Rs-0.1, Rs
                    if dist_ik > lb:
                        if dist_ik > ub: ups_ik = 1
                        else: 
                            ups_ik = 1 - np.exp( - ((dist_ik - lb)/(ub - lb))**2)
                    v_edge += k_ik*ups_ik*vect

                v = v_nom + v_edge

                v_norm = np.hypot(v[0], v[1])
                # if v_norm > 0.5: v = 0.5* v / v_norm # max 

                v_th = 0.1
                if (len(in_neigh_to_connect) > 0) and (v_norm <= v_th):
                    c = 1 # increase communication range

                # NOTE: There should be a consideration on the maximum velocity for the movement

                # norm = np.hypot(v[0], v[1])
                # if norm > 0.5: v = 0.5* v / norm # max 
                
                # TODO: reinvestigate this again, it is really weird.
                # And for some reason it does not work without the saturation

            robot_i.set_input(v, c)

        # Update Plot
        plotter.plot_dist_connectivity(list_robots, edge_list, bar_n, time)
        plt.pause(0.000001) # The pause is needed to show the plot

        # Update time and robots done separately after all robots done sending command
        it += 1
        time = it*Ts
        for robot in list_robots: robot.update()

    print('Simulation Done')


