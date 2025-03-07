from conn_tools import video, draw2DPointSIDisc

import matplotlib.pyplot as plt        
from matplotlib.gridspec import GridSpec
import numpy as np


class plot_visualizer():

    def __init__(self, VIDEO_SETUP, list_robots, bar_n, true_n, Tmax, dir_path, selected_scenario, Ts,
                 log, edge_list, time, it):
        # Parameters on visualization
        self.snapshot, self.snap_num = [0, 5, 10, 15, 20, 25, 30, 35, 40], 0

        self.VIDEO_SETUP = VIDEO_SETUP
        self.list_robots = list_robots
        self.bar_n = bar_n
        self.true_n = true_n
        self.Tmax = Tmax
        self.dir_path = dir_path
        self.selected_scenario = selected_scenario

        if self.VIDEO_SETUP:
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
            self.plotter = draw2DPointSIDisc( ax_2D, field_x = [-2.5, 2.5], field_y = [-2.5, 2.5])
            self.plotter.plot_dist_connectivity(self.list_robots, edge_list, self.bar_n, time, it)
            
            # AUXILARY PLOTS
            ax_ts0 = fig.add_subplot(gs[0, 2:4])
            ax_ts1 = fig.add_subplot(gs[1, 2:4])

            pre_string = 'card_scc_'
            self.id_ax_0 = [1, 2, 3, 4]
            self.id_ax_1 = [6, 7, 8, 9]

            ax_ts0.plot([0., self.Tmax], [self.true_n, self.true_n], '--', color='k', alpha=0.5, linewidth=LW, label='$|\mathcal{V}|$')
            ax_ts1.plot([0., self.Tmax], [self.true_n, self.true_n], '--', color='k', alpha=0.5, linewidth=LW, label='$|\mathcal{V}|$')

            # VARIABLE TO STORE DASHED DATA
            self.dashed_data = {}
            self.dashed_time = {}

            self.pl_SCC_dash, self.pl_SCC_line = {}, {}
            for id in self.id_ax_0:
                key_id = 'card_scc_'+str(id)
                self.process_scc_dash(key_id, log, it)

                self.pl_SCC_dash[key_id], = ax_ts0.plot(self.dashed_time[key_id], self.dashed_data[key_id], ':', color=_colorList[id], linewidth=LW)
                self.pl_SCC_line[key_id], = ax_ts0.plot(0., 0., color=_colorList[id], linewidth=LW, label=id)

            for id in self.id_ax_1:
                key_id = 'card_scc_'+str(id)
                self.process_scc_dash(key_id, log, it)

                self.pl_SCC_dash[key_id], = ax_ts1.plot(self.dashed_time[key_id], self.dashed_data[key_id], ':', color=_colorList[id], linewidth=LW)
                self.pl_SCC_line[key_id], = ax_ts1.plot(0., 0., color=_colorList[id], linewidth=LW, label=id)

            from matplotlib.ticker import MaxNLocator
            ax_ts0.set(ylabel='$|\mathcal{C}_i|$')
            ax_ts0.legend(loc='lower right', prop={'size': leg_size})
            ax_ts0.grid(True)
            ax_ts0.set(xlim=(0.1, self.Tmax+0.1), ylim=(-0.1, self.bar_n + 0.1))
            ax_ts0.xaxis.set_ticklabels([])
            ax_ts0.yaxis.set_major_locator(MaxNLocator(integer=True))
            ax_ts0.xaxis.set_major_locator(MaxNLocator(integer=True))
            # ax_ts0.yaxis.set_ticklabels([0, 2, 4, 6, 8, 10])

            ax_ts1.set(xlabel="time [s]", ylabel='$|\mathcal{C}_i|$')
            ax_ts1.legend(loc='lower right', prop={'size': leg_size})
            ax_ts1.grid(True)
            ax_ts1.set(xlim=(0.1, self.Tmax+0.1), ylim=(-0.1, self.bar_n + 0.1))
            ax_ts1.xaxis.set_major_locator(MaxNLocator(integer=True))
            ax_ts1.yaxis.set_major_locator(MaxNLocator(integer=True))
            # ax_ts1.tick_params(axis='both', labelsize=FS)
            
            plt.tight_layout(pad=2)

            # Initialize video recorder
            self.vid_fname = dir_path + '/results/' + selected_scenario + '.avi'
            self.cv2Video = video(fig, self.vid_fname, round(1/Ts))
            print('Initiate saving video into: ', self.vid_fname)

        else: # Normal Plot
            fig = plt.figure(figsize=(6.4, 6.4)) # OLD SETUP
            FS = 16  # font size
            plt.rcParams.update({'font.size': FS})        
            # plt.rc('text', usetex=True)
            # plt.rc('font', family='serif')        
            plt.rcParams['pdf.fonttype'] = 42
            plt.rcParams['ps.fonttype'] = 42
            ax_2D = plt.gca()
            self.plotter = draw2DPointSIDisc( ax_2D, field_x = [-2.5, 2.5], field_y = [-2.5, 2.5])
            self.plotter.plot_dist_connectivity(self.list_robots, edge_list, self.bar_n, time, it)

    def process_scc_dash(self, key_id, log, it):
        if key_id in self.dashed_data:
            data = log._stored_data[key_id]
            time_data = log._stored_data['time']

            if data[it] is None: # No Data, should be Dash
                if self.dashed_data[key_id][-1] is None: # past is line
                    self.dashed_data[key_id].append(data[it-1])
                    self.dashed_time[key_id].append(time_data[it])
                # else: # past is dash, do nothing                    
            else: # There is data. should be Line
                if self.dashed_data[key_id][-1] is not None: # past is dash
                    self.dashed_data[key_id].append(data[it])
                    self.dashed_time[key_id].append(time_data[it-1])
                    self.dashed_data[key_id].append(None)
                    self.dashed_time[key_id].append(time_data[it])
                # else: # past is line, do nothing 
        else:
            self.dashed_data[key_id], self.dashed_time[key_id] = [1.], [0.]


    def update(self, log, edge_list, time, it):
        # Update Plot
        self.plotter.plot_dist_connectivity(self.list_robots, edge_list, self.bar_n, time, it)

        if self.VIDEO_SETUP:
            for id in self.id_ax_0:
                key_id = 'card_scc_'+str(id)
                self.process_scc_dash(key_id, log, it)

                self.pl_SCC_dash[key_id].set_data(self.dashed_time[key_id], self.dashed_data[key_id])
                self.pl_SCC_line[key_id].set_data(log._stored_data['time'], log._stored_data[key_id])

            for id in self.id_ax_1:
                key_id = 'card_scc_'+str(id)
                self.process_scc_dash(key_id, log, it)

                self.pl_SCC_dash[key_id].set_data(self.dashed_time[key_id], self.dashed_data[key_id])
                self.pl_SCC_line[key_id].set_data(log._stored_data['time'], log._stored_data[key_id])

            plt.pause(0.000001) # The pause is needed to show the plot
            self.cv2Video.save_image()

        else: 
            plt.pause(0.000001) # The pause is needed to show the plot

        # Save snapshot
        if (self.snap_num < len(self.snapshot)) and (int(time) == self.snapshot[self.snap_num]):
            fname = self.dir_path + '/results/' + self.selected_scenario + '_' + str(time) + '.pdf'
            plt.savefig(fname, bbox_inches="tight", dpi=300)
            print('saving into: ', fname)
            self.snap_num += 1

    def finalize(self):
        # Save final plot
        fname = self.dir_path + '/results/' + self.selected_scenario + '_' + str(self.Tmax) + '.pdf'
        plt.savefig(fname, bbox_inches="tight", dpi=300)

        if self.VIDEO_SETUP:
            self.cv2Video.close_editor()
            print('Finished saving video into: ', self.vid_fname)



    # ---------------------------------------
    # POST-PLOTTING for paper
    # ---------------------------------------
    def post_plotting(self, log):
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

        ax[0].plot([0., self.Tmax], [self.true_n, self.true_n], '--', color='k', alpha=0.5, linewidth=LW, label='$|\mathcal{V}|$')
        ax[1].plot([0., self.Tmax], [self.true_n, self.true_n], '--', color='k', alpha=0.5, linewidth=LW) #, label='$|\mathcal{V}|$')

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
        ax[0].set(xlim=(0.1, self.Tmax+0.1), ylim=(-0.1, self.bar_n + 0.1))
        ax[0].yaxis.set_ticks(np.arange(0, self.bar_n, 4))    
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
        ax[1].set(xlim=(0.1, self.Tmax+0.1), ylim=(-0.1, self.bar_n + 0.1))
        ax[1].yaxis.set_ticks(np.arange(0, self.bar_n, 4))    
        ax[1].xaxis.set_major_locator(MaxNLocator(integer=True))    
        # Shrink current axis by 20%
        box = ax[1].get_position()
        ax[1].set_position([box.x0, box.y0, box.width * 0.8, box.height])
        # Put a legend to the right of the current axis
        ax[1].legend(loc='center left', bbox_to_anchor=(1, 0.5), prop={'size': leg_size})

        plt.tight_layout(pad=0.5)

        fname = self.dir_path + '/results/' + self.selected_scenario + '_SCC.pdf'
        plt.savefig(fname, bbox_inches="tight", dpi=300)

        print('Simulation Done')
