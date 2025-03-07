from nebosim.logger import dataLogger
from disc_connectivity import SI_discComm, DistributedConnectivity, evaluateAdjacency
from scenario_plotvis import plot_visualizer

import numpy as np
import os 


if __name__ == '__main__':
    dir_path = os.path.dirname(os.path.realpath(__file__))

    Ts = 0.05 # seconds
    list_robots = [
        SI_discComm(Ts, robot_ID=1, init_pos=np.array([ 1.5, -0.4, 0]), init_comR=0.6),
        SI_discComm(Ts, robot_ID=2, init_pos=np.array([-1.2, -0.5, 0]), init_comR=1.),
        SI_discComm(Ts, robot_ID=3, init_pos=np.array([ 0.,   1.,  0]), init_comR=0.6),
        SI_discComm(Ts, robot_ID=4, init_pos=np.array([ 1.5, -1.5, 0]), init_comR=0.8),
        SI_discComm(Ts, robot_ID=6, init_pos=np.array([ 0.,  -1.5, 0]), init_comR=0.6),
        SI_discComm(Ts, robot_ID=7, init_pos=np.array([ 1.,  -1.,  0]), init_comR=0.8),
        SI_discComm(Ts, robot_ID=8, init_pos=np.array([ 1.,  1.5, 0]), init_comR=1.2),
        SI_discComm(Ts, robot_ID=9, init_pos=np.array([ 0.,  -0.2, 0]), init_comR=1.4),
    ]
    bar_n = 10
    true_n = len(list_robots)

    SCENARIO_STATIONARY_NODES = 'stationary'
    SCENARIO_MOBILE_NODES = 'mobile'

    selected_scenario, Tmax = SCENARIO_STATIONARY_NODES, 16 + Ts
    # selected_scenario, Tmax = SCENARIO_MOBILE_NODES, 43 + Ts
    # Tmax = 1.1

    VIDEO_SETUP = True
    # VIDEO_SETUP = False

    # Create mapping to locate list index from robot id
    map_id_list = {}
    for idx, robot_i in enumerate(list_robots):
        map_id_list[robot_i.robot_ID] = idx

    adj_mat, edge_list = evaluateAdjacency(list_robots, bar_n)
    # Initiate list of controller for each robot
    # ENSURE that the ordering follow the list_robots properly
    list_conn_est = []
    for robot in list_robots:
        list_conn_est.append( DistributedConnectivity(robot.robot_ID, bar_n, list(np.where(adj_mat[robot.robot_ID] > 0)[0])) )

    # Define Iterations and times
    it, time = 0, 0.

    # Initialize logger
    log = dataLogger(round(Tmax/Ts))
    # Initialize visualizer
    plotVis = plot_visualizer(VIDEO_SETUP, list_robots, bar_n, true_n, Tmax, dir_path, selected_scenario, Ts,
                              log, edge_list, time, it)


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

            # Log cardinality of SCC estimation
            card_scc_i = None
            if conn_est_i.list_SCC is not None:
                card_scc_i = len(conn_est_i.list_SCC) 

            log.store_data("card_scc_"+str(i), card_scc_i)

            # Compute control input
            v, c = np.zeros(3), 0.
            if selected_scenario == SCENARIO_STATIONARY_NODES:
                c = conn_est_i.compute_control_radius()

            if selected_scenario ==  SCENARIO_MOBILE_NODES:
                # Collect position and radius information
                dict_neigh_pos, dict_neigh_comR = {}, {}
                for j in i_neigh:
                    robot_j = list_robots[ map_id_list[j] ]
                    dict_neigh_pos[j] = robot_j.state["pos"]
                    dict_neigh_comR[j] = robot_j.state['comR']

                v, c = conn_est_i.compute_control_position_radius( robot_i.state["pos"], robot_i.state["comR"],
                                                                  dict_neigh_pos, dict_neigh_comR )

            # Send input to the robot
            robot_i.set_input(v, c)

        # Stamp log
        log.time_stamp(time)

        # Update the visualization
        plotVis.update(log, edge_list, time, it)

        # Update time and robots done separately after all robots done sending command
        it += 1
        time = it*Ts
        for robot in list_robots: robot.update()


    plotVis.finalize()
    # POST PLOTTING
    plotVis.post_plotting(log)









