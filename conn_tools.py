from nebosim.plotter import draw2DPointSI
import numpy as np
import copy
import cv2

# Move the video to library
class video():
    """
    A class for generating video from simulation.
    Utilizing the OpenCV2 library cv2.VideoWriter
    For now it is defaulting to MJPG format.
    """

    def __init__(self, fig, fname, frame_rate):
        # fname = fpath + 'VideoWriter' + '.avi'

        # create OpenCV video writer
        self.fig = fig
        fourcc = cv2.VideoWriter_fourcc('M','J','P','G') 

        self.writer = cv2.VideoWriter(fname, fourcc, frame_rate, self.fig.canvas.get_width_height())


    def save_image(self):
        self.fig.canvas.draw()
        # put pixel buffer in numpy array
        mat = np.array(self.fig.canvas.renderer._renderer)
        mat = cv2.resize(mat, self.fig.canvas.get_width_height())
        mat = cv2.cvtColor(mat, cv2.COLOR_RGBA2BGR)
        # write frame to video
        self.writer.write(mat)


    def close_editor(self):
        # close video writer
        self.writer.release()
        cv2.destroyAllWindows()



# Define new plotter for a single integrator with communication disc
class draw2DPointSIDisc(draw2DPointSI):
    """
    A class for plotting for multi-robots
    with single integrator (kinematic) model with communication disc 
    in 2D field (x,y plane).
    Inheritted from draw2DPointSI in nebolabsim.plotter.
    """

    def __init__(self, ax, *, field_x = None, field_y = None, pos_trail_nums = 0):
        """
        Initialization for draw2DPointSIDisc

        Args:
            ax (_type_): axis for the plot??
            field_x (list, optional): [minimum x axis, maximum x axis]. Defaults to None.
            field_y (list, optional): [minimum y axis, maximum y axis]. Defaults to None.
            pos_trail_nums (int, optional): Number of past position data to show as trail. Defaults to 0.
        """

        super().__init__(ax, field_x = field_x, field_y = field_y, pos_trail_nums = pos_trail_nums) 

        # plot placeholder for communication Range
        lin_theta = np.linspace(0., 2*np.pi, 360, endpoint=True)
        self._origin_unit_circle = np.array([np.cos(lin_theta), np.sin(lin_theta)])
        self._pl_comR = {}
        self._txt_robotID = {} 
    
        # plot placeholder for network
        self._prev_edge_list = {}
        self._pl_links = {}

        self._pl_input_vec = {}


    def plot_robot_comR(self, key_id, pos, comR):
        """
        Draw the robot's position and its communication range

        Args:
            key_id (_type_): _description_
            pos (_type_): _description_
            comR (_type_): _description_
        """        
        # Compute points to draw communication circles
        com_circle = comR*self._origin_unit_circle

        # Update data on existing plot
        if key_id in self._pl_comR: 
            self._pl_comR[key_id].set_data(pos[0] + com_circle[0], pos[1] + com_circle[1])
            self._txt_robotID[key_id].set_position((pos[0], pos[1]))

        # Initiate plot the first time
        else:
            color_id = key_id % (len(self._colorList)) # Adjust color
            # Draw first position
            self._pl_comR[key_id], = self._ax.plot( pos[0] + com_circle[0], pos[1] + com_circle[1], '-', alpha=0.5,
                                                    color=self._colorList[color_id], linewidth=2.5 )
            
            self._txt_robotID[key_id] = self._ax.text(pos[0], pos[1], str(key_id), weight='bold',
                                                      color=self._colorList[color_id],
                                                      horizontalalignment='center', verticalalignment='center')
            

    def draw_communication_graph(self, edge_list, bar_n, offset=0.1):
        for i in range(bar_n):
            for j in range(bar_n):
                if i != j:
                    key = f'{i}_{j}'
                    if key in edge_list: # Edge in current graph
                        ori_pos_fr, ori_pos_to = edge_list[key]['from'], edge_list[key]['to']
                        ori_vec = ori_pos_to - ori_pos_fr
                        ori_vec_dist = np.linalg.norm(ori_vec)
                        unit_vec = ori_vec/ori_vec_dist

                        vec_dist = ori_vec_dist - 2*offset
                        pos_fr = ori_pos_fr + offset*unit_vec
                        vec = vec_dist*unit_vec

                        if key in self._prev_edge_list:
                            # Update link position
                            self._pl_links[key].set_offsets( [pos_fr[0], pos_fr[1]] )
                            self._pl_links[key].set_UVC( vec[0], vec[1] )
                        else:
                            # Draw a new link
                            head_scale = 1
                            self._pl_links[key] = self._ax.quiver( pos_fr[0], pos_fr[1], vec[0], vec[1], 
                                scale_units='xy', scale=1.0, color='k', width=0.01, alpha=0.3,
                                headwidth = 3*head_scale, headlength = 3*head_scale, 
                                headaxislength=3*head_scale)

                    else: # Not a link currently
                        if key in self._prev_edge_list: 
                            self._pl_links[key].remove() # Remove link
        
        self._prev_edge_list = copy.deepcopy(edge_list)


    def plot_vector_input(self, i, pos, vec, offset = 0.1):
        key_id = f'{i}'
        print(key_id, vec)
        unit_vec = vec/np.linalg.norm(vec)
        pos_fr = pos + offset*unit_vec

        if key_id in self._pl_input_vec:
            # Update link position
            self._pl_input_vec[key_id].set_offsets( [pos_fr[0], pos_fr[1]] )
            self._pl_input_vec[key_id].set_UVC( vec[0], vec[1] )
        else:
            # Draw a new link
            head_scale = 2
            self._pl_input_vec[key_id] = self._ax.quiver( pos_fr[0], pos_fr[1], vec[0], vec[1], 
                scale_units='xy', scale=1.0, color='g', width=0.005,
                headwidth = 2*head_scale, headlength = 3*head_scale, 
                headaxislength=3*head_scale, zorder=50)


    def plot_time_iter(self, time, it):
        if self._drawn_time is None:
            # Display simulation time
            self._drawn_time = self._ax.text(0.02, 0.99, 'time = ' + f"{time:.2f}" + ' s', color='k', fontsize='large',
                                        horizontalalignment='left', verticalalignment='top', transform=self._ax.transAxes)
            self._drawn_iter = self._ax.text(0.02, 0.94, 'iter = ' + f"{it}" + ' s', color='k', fontsize='large',
                                        horizontalalignment='left', verticalalignment='top', transform=self._ax.transAxes)
        else:
            self._drawn_time.set_text('time = ' + f"{time:.2f}" + ' s')
            self._drawn_iter.set_text('iter = ' + f"{it}")


    def plot_dist_connectivity(self, list_robots, edge_list, bar_n, time, it):
        
        for robot in list_robots: 
            # self.plot_robot_pos(robot.robot_ID, robot.state['pos'])
            self.plot_robot_comR(robot.robot_ID, robot.state['pos'], robot.state['comR'])
        
        self.draw_communication_graph(edge_list, bar_n)
        self.plot_time_iter(time, it)


