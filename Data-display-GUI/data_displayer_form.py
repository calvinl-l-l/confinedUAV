# Form layer for data displayer

from pyqtgraph import QtGui, QtCore
from pyqtgraph.dockarea import *
import pyqtgraph as pg
import numpy as np
import data_processor as dp
import math
import collections


class disp_form(QtGui.QWidget):
    def __init__(self):
        QtGui.QWidget.__init__(self)
        self.setWindowTitle('Data displayer')
        # self.setSizePolicy(QtGui.QSizePolicy.Ignored, QtGui.QSizePolicy.Ignored)

# form global variables
        self.file_openned = 0
        self.button = ''
        self.mouse_pos = QtCore.QPointF()
        self.mousePoint_x = 0
        self.mousePoint_y = 0
        self.mousePoint_z = 0
        self.mousePoint_roll = 0
        self.mousePoint_yaw = 0
        self.vLine_x = pg.InfiniteLine(angle=90, movable=False)
        self.vLine_y = pg.InfiniteLine(angle=90, movable=False)
        self.vLine_z = pg.InfiniteLine(angle=90, movable=False)
        self.vLine_roll = pg.InfiniteLine(angle=90, movable=False)
        self.vLine_yaw = pg.InfiniteLine(angle=90, movable=False)
        self.manual_slide = 0
        self.plotscene = 'y'

        # operation state
        self.state = 'stop'
            # replay -> auto play data, infinite loop
            # stop   -> stop replay
            # slide  -> manually slide through data set

    # file variable
        self.f_x = np.array([])
        self.f_y = np.array([])
        self.f_z = np.array([])
        self.f_raw_ldata_angle = np.array([])
        self.f_raw_ldata_range = np.array([])
        self.f_roll = np.array([])
        self.f_yaw = np.array([])
        self.f_area = np.array([])
        self.f_sys_time = np.array([])
        self.n_f_loop = 0

# layout
        layout = QtGui.QHBoxLayout(self)
        layout_R = QtGui.QVBoxLayout()
        layout_R_plot = QtGui.QVBoxLayout()
        layout_R_control = QtGui.QHBoxLayout()
        layout_L = QtGui.QVBoxLayout()
        layout_L_info = QtGui.QVBoxLayout()
        layout_L_btn = QtGui.QHBoxLayout()

        layout_L.addLayout(layout_L_info)
        layout_L.addLayout(layout_L_btn)

        layout_R.addLayout(layout_R_plot,10)
        layout_R.addLayout(layout_R_control,1)

        layout.addLayout(layout_L, 3)
        layout.addLayout(layout_R, 7)

# label
        self.lb_info = QtGui.QLabel('All info')
        self.lb_info.setFrameStyle(0x01)
        self.lb_info.setLineWidth(2)
        self.lb_info.setFixedWidth(400)
        self.lb_info.setAlignment(QtCore.Qt.AlignTop)
        f = QtGui.QFont("Calbri", 11)
        self.lb_info.setFont(f)

        lb_roll = QtGui.QLabel('roll vs time')
        lb_yaw = QtGui.QLabel('yaw vs time')
        lb_y_pos = QtGui.QLabel('y pos vs time')
        lb_z_pos = QtGui.QLabel('z pos vs time')
        lb_x_pos = QtGui.QLabel('x pos vs time')
        lb_scan = QtGui.QLabel('cross section scan data')
        lb_time_stamp = QtGui.QLabel('time frame (ms)')
# text edit
        self.txt_time_stamp = QtGui.QTextEdit()
        f = QtGui.QFont("Calbri", 11)
        self.txt_time_stamp.setFont(f)



# button
        self.btn_open_file = QtGui.QPushButton('open file')
        self.btn_replay = QtGui.QPushButton('play')
        self.btn_stop = QtGui.QPushButton('stop')


# slide bar
        self.slide_frame = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.slide_frame.setMinimum(0)
        self.slide_frame.setMaximum(100)
        self.slide_frame.setValue(0)
        self.slide_frame.setTickPosition(QtGui.QSlider.TicksBelow)
        self.slide_frame.setTickInterval(2)



# plot widget
        self.ps = pg.PlotWidget() # scan
        self.px = pg.PlotWidget()  # x
        self.py = pg.PlotWidget()  # y
        self.pz = pg.PlotWidget()  # z
        self.proll = pg.PlotWidget()  # scan
        self.pyaw = pg.PlotWidget()  # scan

        self.py.addItem(self.vLine_y, ignoreBounds=True)
        self.pz.addItem(self.vLine_z, ignoreBounds=True)
        self.px.addItem(self.vLine_x, ignoreBounds=True)
        self.proll.addItem(self.vLine_roll, ignoreBounds=True)
        self.pyaw.addItem(self.vLine_yaw, ignoreBounds=True)

# dock widget -----------> still need to add plots
        # dock roll
        dock_roll = Dock("Roll")
        dock_roll_layout = pg.LayoutWidget()
        dock_roll_layout.addWidget(lb_roll, row=0, col=0)
        dock_roll_layout.addWidget(self.proll, row=1, col=0)
        dock_roll.addWidget(dock_roll_layout)

        # dock yaw
        dock_yaw = Dock("Yaw")
        dock_yaw_layout = pg.LayoutWidget()
        dock_yaw_layout.addWidget(lb_yaw, row=0, col=0)
        dock_yaw_layout.addWidget(self.pyaw, row=1, col=0)
        dock_yaw.addWidget(dock_yaw_layout)

        # dock y pos
        dock_y_pos = Dock("pos y")
        dock_y_pos_layout = pg.LayoutWidget()
        dock_y_pos_layout.addWidget(lb_y_pos, row=0, col=0)
        dock_y_pos_layout.addWidget(self.py, row=1, col=0)
        dock_y_pos.addWidget(dock_y_pos_layout)

        # dock z pos
        dock_z_pos = Dock("pos z")
        dock_z_pos_layout = pg.LayoutWidget()
        dock_z_pos_layout.addWidget(lb_z_pos, row=0, col=0)
        dock_z_pos_layout.addWidget(self.pz, row=1, col=0)
        dock_z_pos.addWidget(dock_z_pos_layout)

        # dock x pos
        dock_x_pos = Dock("pos x")
        dock_x_pos_layout = pg.LayoutWidget()
        dock_x_pos_layout.addWidget(lb_x_pos, row=0, col=0)
        dock_x_pos_layout.addWidget(self.px, row=1, col=0)
        dock_x_pos.addWidget(dock_x_pos_layout)

        # dock tab
        plot_tab = DockArea()
        plot_tab.addDock(dock_roll)
        plot_tab.addDock(dock_yaw)
        plot_tab.addDock(dock_y_pos)
        plot_tab.addDock(dock_z_pos)
        plot_tab.addDock(dock_x_pos)
        plot_tab.moveDock(dock_z_pos, 'above', dock_x_pos)
        plot_tab.moveDock(dock_y_pos, 'above', dock_z_pos)
        plot_tab.moveDock(dock_yaw, 'above', dock_y_pos)
        plot_tab.moveDock(dock_roll, 'above', dock_yaw)

        dock_y_pos.show()

# assign widgets to layouts
        layout_L_btn.addWidget(self.btn_replay, 0, QtCore.Qt.AlignTop)
        layout_L_btn.addWidget(self.btn_stop, 0, QtCore.Qt.AlignTop)
        layout_L_info.addWidget(self.lb_info, 1, QtCore.Qt.AlignTop)
        layout_L_info.addWidget(self.btn_open_file, 0, QtCore.Qt.AlignTop)

        layout_R_plot.addWidget(lb_scan,0,QtCore.Qt.AlignHCenter)
        layout_R_plot.addWidget(self.ps)
        layout_R_plot.addWidget(plot_tab)
        layout_R_plot.addWidget(lb_time_stamp, 0, QtCore.Qt.AlignHCenter)
        layout_R_control.addWidget(self.slide_frame,4)
        layout_R_control.addWidget(self.txt_time_stamp,1)

# mouse interaction
        self.proxy_move_x = pg.SignalProxy(self.px.scene().sigMouseMoved, rateLimit=60, slot=self.mouseMoved)
        self.proxy_move_y = pg.SignalProxy(self.py.scene().sigMouseMoved, rateLimit=60, slot=self.mouseMoved)
        self.proxy_move_z = pg.SignalProxy(self.pz.scene().sigMouseMoved, rateLimit=60, slot=self.mouseMoved)
        self.proxy_move_roll = pg.SignalProxy(self.proll.scene().sigMouseMoved, rateLimit=60, slot=self.mouseMoved)
        self.proxy_move_yaw = pg.SignalProxy(self.pyaw.scene().sigMouseMoved, rateLimit=60, slot=self.mouseMoved)
        self.proxy_click_x = pg.SignalProxy(self.px.scene().sigMouseClicked, rateLimit=60, slot=self.mouseClicked_scene_x)
        self.proxy_click_y = pg.SignalProxy(self.py.scene().sigMouseClicked, rateLimit=60, slot=self.mouseClicked_scene_y)
        self.proxy_click_z = pg.SignalProxy(self.pz.scene().sigMouseClicked, rateLimit=60, slot=self.mouseClicked_scene_z)
        self.proxy_click_roll = pg.SignalProxy(self.proll.scene().sigMouseClicked, rateLimit=60, slot=self.mouseClicked_scene_roll)
        self.proxy_click_yaw = pg.SignalProxy(self.pyaw.scene().sigMouseClicked, rateLimit=60, slot=self.mouseClicked_scene_yaw)



        # ====================       END OF INIT     ====================================================


    def interface(self):
        self.btn_open_file.clicked.connect(self.getfile)
        self.btn_replay.clicked.connect(self.replay_data)
        self.btn_stop.clicked.connect(self.stop_replay)

# button interface
        if self.button != '':
            if self.button == 'getfile':
                fname = QtGui.QFileDialog.getOpenFileName(self, 'Open file',
                                                          '', "Text files (*.txt)")

                self.file_openned = 1

                fname = str(fname)
                ldata_content = []
                scan_content = []

                if fname != '':
                    print fname
                    if fname.find('ldata') != -1:
                        ldata_fd = open(fname, 'r')
                        ldata_fd.readline()
                        ldata_fd.readline()
                        ldata_fd.readline()
                        ldata_content = ldata_fd.readlines()

                        fname = fname.replace('ldata', 'scan')

                        scan_fd = open(fname, 'r')
                        scan_content = scan_fd.readlines()

                        ldata_fd.close()
                        scan_fd.close()

                    elif fname.find('scan') != -1:
                        scan_fd = open(fname, 'r')
                        scan_content = scan_fd.readlines()

                        fname = fname.replace('scan', 'ldata')
                        ldata_fd = open(fname, 'r')
                        ldata_fd.readline()
                        ldata_fd.readline()
                        ldata_fd.readline()
                        ldata_content = ldata_fd.readlines()

                        ldata_fd.close()
                        scan_fd.close()

                    self.f_x, self.f_y, self.f_z, self.f_raw_ldata_angle, self.f_raw_ldata_range, \
                    self.f_roll, self.f_yaw, self.f_area, self.f_sys_time\
                    = dp.file_data_processer(scan_content, ldata_content)

                    self.init_plot()

            elif self.button == 'replay':
                self.state = 'replay'

            elif self.button == 'stop':
                self.state = 'stop'


            self.button = ''

# mouse interface
        vb_x = self.px.plotItem.vb
        vb_y = self.py.plotItem.vb
        vb_z = self.pz.plotItem.vb
        vb_roll = self.proll.plotItem.vb
        vb_yaw = self.pyaw.plotItem.vb

        if self.manual_slide == 1:
            self.state = 'slide'

            self.mousePoint_x = vb_x.mapSceneToView(self.mouse_pos)
            self.vLine_x.setPos(self.mousePoint_x.x())

            self.mousePoint_y = vb_y.mapSceneToView(self.mouse_pos)
            self.vLine_y.setPos(self.mousePoint_y.x())

            self.mousePoint_z = vb_z.mapSceneToView(self.mouse_pos)
            self.vLine_z.setPos(self.mousePoint_z.x())

            self.mousePoint_roll = vb_roll.mapSceneToView(self.mouse_pos)
            self.vLine_roll.setPos(self.mousePoint_roll.x())

            self.mousePoint_yaw = vb_yaw.mapSceneToView(self.mouse_pos)
            self.vLine_yaw.setPos(self.mousePoint_yaw.x())

        self.state_handler(self.state)

        # ======================     END OF INTERFACE      ==============================================

    def state_handler(self, state):
        fdata = dp.data_class()

        if state == 'replay':
            fdata, self.n_f_loop = dp.f_data2plot_data(self.f_x, self.f_y, self.f_z,
                                                       self.f_raw_ldata_angle,
                                                       self.f_raw_ldata_range,
                                                       self.f_roll, self.f_yaw, self.f_area,
                                                       self.f_sys_time, self.n_f_loop, self.state)


            self.vLine_x.setPos(fdata.time * 1000)
            self.vLine_y.setPos(fdata.time * 1000)
            self.vLine_z.setPos(fdata.time * 1000)
            self.vLine_roll.setPos(fdata.time * 1000)
            self.vLine_yaw.setPos(fdata.time * 1000)


        elif state == 'stop':
            a =1

        elif state == 'slide':
            if self.plotscene == 'x':
                fdata, self.n_f_loop = dp.f_data2plot_data(self.f_x, self.f_y, self.f_z,
                                                           self.f_raw_ldata_angle,
                                                           self.f_raw_ldata_range,
                                                           self.f_roll, self.f_yaw, self.f_area,
                                                           self.f_sys_time,
                                                           int(math.floor(self.mousePoint_x.x())/100), self.state)

            elif self.plotscene == 'y':
                fdata, self.n_f_loop = dp.f_data2plot_data(self.f_x, self.f_y, self.f_z,
                                                           self.f_raw_ldata_angle,
                                                           self.f_raw_ldata_range,
                                                           self.f_roll, self.f_yaw, self.f_area,
                                                           self.f_sys_time,
                                                           int(math.floor(self.mousePoint_y.x()) / 100), self.state)

            elif self.plotscene == 'z':
                fdata, self.n_f_loop = dp.f_data2plot_data(self.f_x, self.f_y, self.f_z,
                                                           self.f_raw_ldata_angle,
                                                           self.f_raw_ldata_range,
                                                           self.f_roll, self.f_yaw, self.f_area,
                                                           self.f_sys_time,
                                                           int(math.floor(self.mousePoint_z.x()) / 100), self.state)

            elif self.plotscene == 'roll':
                fdata, self.n_f_loop = dp.f_data2plot_data(self.f_x, self.f_y, self.f_z,
                                                           self.f_raw_ldata_angle,
                                                           self.f_raw_ldata_range,
                                                           self.f_roll, self.f_yaw, self.f_area,
                                                           self.f_sys_time,
                                                           int(math.floor(self.mousePoint_roll.x()) / 100), self.state)

            elif self.plotscene == 'yaw':
                fdata, self.n_f_loop = dp.f_data2plot_data(self.f_x, self.f_y, self.f_z,
                                                           self.f_raw_ldata_angle,
                                                           self.f_raw_ldata_range,
                                                           self.f_roll, self.f_yaw, self.f_area,
                                                           self.f_sys_time,
                                                           int(math.floor(self.mousePoint_yaw.x()) / 100), self.state)


        if self.file_openned == 1 and state != 'stop':
            self.update_plot(fdata)

            percentage = dp.plot_percentage(self.f_y, self.n_f_loop)
            self.slide_frame.setValue(percentage)

    def init_plot(self):

        self.ps.clear()
        self.px.clear()
        self.py.clear()
        self.pz.clear()
        self.proll.clear()
        self.pyaw.clear()


        roll_curve = pg.PlotCurveItem()
        yaw_curve = pg.PlotCurveItem()
        x_curve = pg.PlotCurveItem()
        y_curve = pg.PlotCurveItem()
        z_curve = pg.PlotCurveItem()
        zero_curve = pg.PlotCurveItem(size=4, pen='r', brush='r')


        roll_curve.setData(self.f_sys_time, self.f_roll)
        yaw_curve.setData(self.f_sys_time, self.f_yaw)
        x_curve.setData(self.f_sys_time, self.f_x)
        y_curve.setData(self.f_sys_time, self.f_y)
        z_curve.setData(self.f_sys_time, self.f_z)
        zero_curve.setData(self.f_sys_time, np.array([0]*len(self.f_sys_time)))

        self.proll.addItem(zero_curve)
        self.proll.addItem(roll_curve)
        self.proll.addItem(self.vLine_roll)
        self.pyaw.addItem(yaw_curve)
        self.pyaw.addItem(self.vLine_yaw)
        self.py.addItem(y_curve)
        self.py.addItem(self.vLine_y)
        self.py.addItem(zero_curve)
        self.pz.addItem(z_curve)
        self.pz.addItem(self.vLine_z)
        self.px.addItem(x_curve)
        self.px.addItem(self.vLine_x)

        self.proll.setYRange(-30, 30, 0)
        self.proll.setXRange(0, 30000, 0)

        self.ps.setXRange(-1200, 1200, 0)
        self.ps.setYRange(-1200, 1200, 0)
        self.ps.setAspectLocked(True, 1)

    def update_plot(self, pdata):

        self.ps.clear()  # only clearing raw plot, keep overlapped plot

        s1 = pg.ScatterPlotItem(size=5, pen='w', brush='w')
        lscan_1 = dp.compute_raw_scan(pdata)
        s1.addPoints(lscan_1[:, 0], lscan_1[:, 1])

        self.ps.addItem(s1)

        # draw quad model
        r_wing, l_wing, r_motor, l_motor, r_prop, l_prop = dp.draw_quad_model(pdata.y, pdata.z, pdata.roll)
        self.ps.addItem(r_wing)
        self.ps.addItem(l_wing)
        self.ps.addItem(r_motor)
        self.ps.addItem(l_motor)
        self.ps.addItem(r_prop)
        self.ps.addItem(l_prop)

        self.ps.repaint()

        info = 'Drone status:\n\n'
        info += 'system time: ' + str(pdata.time) + "s\n"
        info += 'Position (y,z) -> ' + '(' + str(round(pdata.y,2)) + ' , ' + str(round(pdata.z,2)) + ')' + ' mm\n'
        info += 'Position x -> ' + str(pdata.x) + ' m\n'
        info += 'Area = ' + str(round(pdata.area,2)) + ' m^2\n'
        info += 'Roll = ' + str(round(pdata.roll,2)) + ' deg\n'
        info += 'Yaw = ' + str(round(pdata.yaw,2)) + ' deg\n'

        self.lb_info.setText(info)


    def mouseClicked_scene_x(self):
        self.manual_slide = 1 - self.manual_slide
        self.plotscene = 'x'

    def mouseClicked_scene_y(self):
        self.manual_slide = 1 - self.manual_slide
        self.plotscene = 'y'

    def mouseClicked_scene_z(self):
        self.manual_slide = 1 - self.manual_slide
        self.plotscene = 'z'

    def mouseClicked_scene_roll(self):
        self.manual_slide = 1 - self.manual_slide
        self.plotscene = 'roll'

    def mouseClicked_scene_yaw(self):
        self.manual_slide = 1 - self.manual_slide
        self.plotscene = 'yaw'

    def mouseMoved(self, evt):
        self.mouse_pos = evt[0]

# button interface ==============================================================
    def getfile(self):
        self.button = 'getfile'

    def replay_data(self):
        self.button = 'replay'

    def stop_replay(self):
        self.button = 'stop'