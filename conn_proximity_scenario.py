from nebosim.logger import dataLogger
from disc_connectivity import SI_discComm, DistributedConnectivity, evaluateAdjacency
from conn_tools import video, draw2DPointSIDisc

import numpy as np
import matplotlib.pyplot as plt        
from matplotlib.gridspec import GridSpec
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

    # selected_scenario, Tmax = SCENARIO_STATIONARY_NODES, 16 + Ts
    selected_scenario, Tmax = SCENARIO_MOBILE_NODES, 43 + Ts
    # Tmax = 1.1

    VIDEO_SETUP = True
    # VIDEO_SETUP = False

    snapshot, snap_num = [0, 5, 10, 15, 20, 25, 30, 35, 40], 0

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

    if VIDEO_SETUP:
        # Initialize plot
        # For now plot 2D with 2x2 grid space, to allow additional plot later on
        rowNum, colNum = 2, 4
        fig = plt.figure(figsize=(3 * colNum, 3 * rowNum), dpi=100)
        gs = GridSpec(rowNum, colNum, figure=fig)

        # MAIN 2D PLOT FOR UNICYCLE ROBOTS
        # ------------------------------------------------------------------------------------
        ax_2D = fig.add_subplot(gs[0:2, 0:2])  # Always on
        FS = 10  # font size
        LW = 2.5
        leg_size = 10
        plt.rcParams.update({'font.size': FS})
        _colorList = plt.rcParams['axes.prop_cycle'].by_key()['color']

        # plt.rcParams['text.usetex'] = True
        plotter = draw2DPointSIDisc( ax_2D, field_x = [-2.5, 2.5], field_y = [-2.5, 2.5])
        plotter.plot_dist_connectivity(list_robots, edge_list, bar_n, time, it)
        
        # AUXILARY PLOTS
        ax_ts0 = fig.add_subplot(gs[0, 2:4])
        ax_ts1 = fig.add_subplot(gs[1, 2:4])

        pre_string = 'card_scc_'
        id_ax_0 = [1, 2, 3, 4]
        id_ax_1 = [6, 7, 8, 9]

        ax_ts0.plot([0., Tmax], [true_n, true_n], '--', color='k', alpha=0.5, linewidth=LW, label='$|\mathcal{V}|$')
        ax_ts1.plot([0., Tmax], [true_n, true_n], '--', color='k', alpha=0.5, linewidth=LW, label='$|\mathcal{V}|$')

        # VARIABLE TO STORE DASHED DATA
        dashed_data = {}
        dashed_time = {}

        def process_scc_dash(key_id, dashed_data, dashed_time, log, it):
            if key_id in dashed_data:
                data = log._stored_data[pre_string+str(id)]
                time_data = log._stored_data['time']

                if data[it] is None: # No Data, should be Dash
                    if dashed_data[key_id][-1] is None: # past is line
                        dashed_data[key_id].append(data[it-1])
                        dashed_time[key_id].append(time_data[it])
                    # else: # past is dash, do nothing                    
                else: # There is data. should be Line
                    if dashed_data[key_id][-1] is not None: # past is dash
                        dashed_data[key_id].append(data[it])
                        dashed_time[key_id].append(time_data[it-1])
                        dashed_data[key_id].append(None)
                        dashed_time[key_id].append(time_data[it])
                    # else: # past is line, do nothing 
            else:
                dashed_data[key_id], dashed_time[key_id] = [1.], [0.]


        pl_SCC_dash, pl_SCC_line = {}, {}
        for id in id_ax_0:
            key_id = 'card_scc_'+str(id)
            process_scc_dash(key_id, dashed_data, dashed_time, log, it)

            pl_SCC_dash[key_id], = ax_ts0.plot(dashed_time[key_id], dashed_data[key_id], ':', color=_colorList[id], linewidth=LW)
            pl_SCC_line[key_id], = ax_ts0.plot(0., 0., color=_colorList[id], linewidth=LW, label=id)

        for id in id_ax_1:
            key_id = 'card_scc_'+str(id)
            process_scc_dash(key_id, dashed_data, dashed_time, log, it)

            pl_SCC_dash[key_id], = ax_ts1.plot(dashed_time[key_id], dashed_data[key_id], ':', color=_colorList[id], linewidth=LW)
            pl_SCC_line[key_id], = ax_ts1.plot(0., 0., color=_colorList[id], linewidth=LW, label=id)

        from matplotlib.ticker import MaxNLocator
        ax_ts0.set(ylabel='$|\mathcal{C}_i|$')
        ax_ts0.legend(loc='lower right', prop={'size': leg_size})
        ax_ts0.grid(True)
        ax_ts0.set(xlim=(0.1, Tmax+0.1), ylim=(-0.1, bar_n + 0.1))
        ax_ts0.xaxis.set_ticklabels([])
        ax_ts0.yaxis.set_major_locator(MaxNLocator(integer=True))
        ax_ts0.xaxis.set_major_locator(MaxNLocator(integer=True))
        # ax_ts0.yaxis.set_ticklabels([0, 2, 4, 6, 8, 10])

        ax_ts1.set(xlabel="time [s]", ylabel='$|\mathcal{C}_i|$')
        ax_ts1.legend(loc='lower right', prop={'size': leg_size})
        ax_ts1.grid(True)
        ax_ts1.set(xlim=(0.1, Tmax+0.1), ylim=(-0.1, bar_n + 0.1))
        ax_ts1.xaxis.set_major_locator(MaxNLocator(integer=True))
        ax_ts1.yaxis.set_major_locator(MaxNLocator(integer=True))
        # ax_ts1.tick_params(axis='both', labelsize=FS)
        
        plt.tight_layout(pad=2)

        # Initialize video recorder
        vid_fname = dir_path + '/results/' + selected_scenario + '.avi'
        cv2Video = video(fig, vid_fname, round(1/Ts))
        print('Initiate saving video into: ', vid_fname)

    else: # Normal Plot
        fig = plt.figure(figsize=(6.4, 6.4)) # OLD SETUP
        FS = 16  # font size
        plt.rcParams.update({'font.size': FS})        
        # plt.rc('text', usetex=True)
        # plt.rc('font', family='serif')        
        plt.rcParams['pdf.fonttype'] = 42
        plt.rcParams['ps.fonttype'] = 42
        ax_2D = plt.gca()
        plotter = draw2DPointSIDisc( ax_2D, field_x = [-2.5, 2.5], field_y = [-2.5, 2.5])
        plotter.plot_dist_connectivity(list_robots, edge_list, bar_n, time, it)


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

                # NOTE: The response of the quiver plot is buggy.
                # plotter.plot_vector_input(i, robot_i.state["pos"], v*100)

            # Send input to the robot
            robot_i.set_input(v, c)

        # Stamp log
        log.time_stamp(time)

        # Update Plot
        plotter.plot_dist_connectivity(list_robots, edge_list, bar_n, time, it)

        if VIDEO_SETUP:
            for id in id_ax_0:
                key_id = 'card_scc_'+str(id)
                process_scc_dash(key_id, dashed_data, dashed_time, log, it)

                pl_SCC_dash[key_id].set_data(dashed_time[key_id], dashed_data[key_id])
                pl_SCC_line[key_id].set_data(log._stored_data['time'], log._stored_data[key_id])

            for id in id_ax_1:
                key_id = 'card_scc_'+str(id)
                process_scc_dash(key_id, dashed_data, dashed_time, log, it)

                pl_SCC_dash[key_id].set_data(dashed_time[key_id], dashed_data[key_id])
                pl_SCC_line[key_id].set_data(log._stored_data['time'], log._stored_data[key_id])

            plt.pause(0.000001) # The pause is needed to show the plot
            cv2Video.save_image()

        else: 
            plt.pause(0.000001) # The pause is needed to show the plot

        # Save snapshot
        if (snap_num < len(snapshot)) and (int(time) == snapshot[snap_num]):
            fname = dir_path + '/results/' + selected_scenario + '_' + str(time) + '.pdf'
            plt.savefig(fname, bbox_inches="tight", dpi=300)
            print('saving into: ', fname)
            snap_num += 1


        # Update time and robots done separately after all robots done sending command
        it += 1
        time = it*Ts
        for robot in list_robots: robot.update()

    # Save final plot
    fname = dir_path + '/results/' + selected_scenario + '_' + str(Tmax) + '.pdf'
    plt.savefig(fname, bbox_inches="tight", dpi=300)

    if VIDEO_SETUP:
        cv2Video.close_editor()
        print('Finished saving video into: ', vid_fname)


    # POST PLOTTING
    import copy

    figure_short = (6.4, 3.4)
    FS = 14  # font size
    LW = 2.5  # line width
    leg_size = 12

    fig, ax = plt.subplots(2, figsize=figure_short)
    plt.rcParams.update({'font.size': FS})
    plt.rcParams['text.usetex'] = True
    _colorList = plt.rcParams['axes.prop_cycle'].by_key()['color']
    
    pre_string = 'card_scc_'
    id_ax_0 = [1, 2, 3, 4]
    id_ax_1 = [6, 7, 8, 9]
    
    time_data = log._stored_data['time']

    ax[0].plot([0., Tmax], [true_n, true_n], '--', color='k', alpha=0.5, linewidth=LW, label='$|\mathcal{V}|$')
    ax[1].plot([0., Tmax], [true_n, true_n], '--', color='k', alpha=0.5, linewidth=LW) #, label='$|\mathcal{V}|$')

    for id in id_ax_0:
        data = log._stored_data[pre_string+str(id)]

        dashed_data, dashed_time = [1.], [0.]
        for idx, value in enumerate(data):
            if value is None: # No Data, should be Dash
                if dashed_data[-1] is None: # past is line
                    dashed_data.append(data[idx-1])
                    dashed_time.append(time_data[idx])
                # else: # past is dash, do nothing                    
            else: # There is data. should be Line
                if dashed_data[-1] is not None: # past is dash
                    dashed_data.append(value)
                    dashed_time.append(time_data[idx-1])
                    dashed_data.append(None)
                    dashed_time.append(time_data[idx])
                # else: # past is line, do nothing 

        ax[0].plot(dashed_time, dashed_data, ':', color=_colorList[id], linewidth=LW)
        ax[0].plot(time_data, data, color=_colorList[id], linewidth=LW, label=id)

    for id in id_ax_1:
        data = log._stored_data[pre_string+str(id)]

        dashed_data, dashed_time = [1.], [0.]
        for idx, value in enumerate(data):
            if value is None: # No Data, should be Dash
                if dashed_data[-1] is None: # past is line
                    dashed_data.append(data[idx-1])
                    dashed_time.append(time_data[idx])
                # else: # past is dash, do nothing                    
            else: # There is data. should be Line
                if dashed_data[-1] is not None: # past is dash
                    dashed_data.append(value)
                    dashed_time.append(time_data[idx-1])
                    dashed_data.append(None)
                    dashed_time.append(time_data[idx])
                # else: # past is line, do nothing 

        ax[1].plot(dashed_time, dashed_data, ':', color=_colorList[id], linewidth=LW)
        ax[1].plot(time_data, data, color=_colorList[id], linewidth=LW, label=id)

    # label
    from matplotlib.ticker import MaxNLocator
    ax[0].set(ylabel='$|\mathcal{C}_i|$')
    # ax[0].legend(loc='lower right', prop={'size': leg_size})
    ax[0].grid(True)
    ax[0].set(xlim=(0.1, Tmax+0.1), ylim=(-0.1, bar_n + 0.1))
    ax[0].yaxis.set_ticks(np.arange(0, bar_n, 4))    
    ax[0].xaxis.set_major_locator(MaxNLocator(integer=True))
    ax[0].xaxis.set_ticklabels([])
    # Shrink current axis by 20%
    box = ax[0].get_position()
    ax[0].set_position([box.x0, box.y0, box.width * 0.8, box.height])
    # Put a legend to the right of the current axis
    ax[0].legend(loc='center left', bbox_to_anchor=(1, 0.5), prop={'size': leg_size})

    ax[1].set(xlabel="time [s]", ylabel='$|\mathcal{C}_i|$')
    ax[1].legend(loc='lower right', prop={'size': leg_size})
    ax[1].grid(True)
    ax[1].set(xlim=(0.1, Tmax+0.1), ylim=(-0.1, bar_n + 0.1))
    ax[1].yaxis.set_ticks(np.arange(0, bar_n, 4))    
    ax[1].xaxis.set_major_locator(MaxNLocator(integer=True))    
    # Shrink current axis by 20%
    box = ax[1].get_position()
    ax[1].set_position([box.x0, box.y0, box.width * 0.8, box.height])
    # Put a legend to the right of the current axis
    ax[1].legend(loc='center left', bbox_to_anchor=(1, 0.5), prop={'size': leg_size})

    plt.tight_layout(pad=0.5)

    fname = dir_path + '/results/' + selected_scenario + '_SCC.pdf'
    plt.savefig(fname, bbox_inches="tight", dpi=300)

    print('Simulation Done')








