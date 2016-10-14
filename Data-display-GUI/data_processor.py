import numpy as np
import pyqtgraph as pg


class data_class:

    def __init__(self):
        self.ldata = np.array([[]*2]*500)
        self.nraw = 0
        self.x    = 0
        self.y    = 0
        self.z    = 0
        self.roll = 0
        self.yaw  = 0
        self.time = 0
        self.mode = 0
        self.area = 0
        self.msg = ''
        self.kp = 0
        self.ki = 0
        self.kd = 0
        self.RFdist = 0

def f_data2plot_data(x, y, z, raw_ldata_angle, raw_ldata_range, roll, yaw, area, sys_time, nloop, state):
    fdata = data_class()


    if nloop >= len(y):
        nloop = 0

    else:
        fdata.x = x[nloop]
        fdata.y = y[nloop]
        fdata.z = z[nloop]
        fdata.ldata = np.vstack([raw_ldata_angle[nloop][:], raw_ldata_range[nloop][:]])
        fdata.ldata = fdata.ldata.swapaxes(0,1)
        fdata.nraw = len(raw_ldata_range[nloop][:])
        fdata.area = area[nloop]
        fdata.roll = roll[nloop]
        fdata.yaw = yaw[nloop]
        fdata.time = round(sys_time[nloop]/1000.0,1)

        if state == 'replay':
            nloop += 1



    return fdata, nloop

def file_data_processer(scan_content, ldata_content):
    for n in xrange(len(scan_content)):
        scan_content[n] = scan_content[n].replace("\n","")
        scan_content[n] = scan_content[n].split(",")

    x = np.array([0] * len(ldata_content), np.dtype(np.float))
    y = np.array([0]*len(ldata_content),np.dtype(np.float))
    z = np.array([0] * len(ldata_content),np.dtype(np.float))
    alt = np.array([0] * len(ldata_content),np.dtype(np.float))
    CH1 = np.array([0] * len(ldata_content))
    CH3 = np.array([0] * len(ldata_content))
    is_manual = np.array([0] * len(ldata_content))
    roll = np.array([0] * len(ldata_content),np.dtype(np.float))
    yaw = np.array([0] * len(ldata_content),np.dtype(np.float))
    area = np.array([0] * len(ldata_content),np.dtype(np.float))
    sys_time = np.array([0] * len(ldata_content))

    for n in xrange(len(ldata_content)):
        ldata_content[n] = ldata_content[n].replace("\n","")
        ldata_content[n] = ldata_content[n].split(',')

        x[n] = ldata_content[n][10]
        y[n] = ldata_content[n][0]
        z[n] = ldata_content[n][1]
        alt[n] = ldata_content[n][2]
        CH1[n] = ldata_content[n][3]
        CH3[n] = ldata_content[n][4]
        is_manual[n] = ldata_content[n][5]
        roll[n] = ldata_content[n][6]
        yaw[n] = ldata_content[n][7]
        area[n] = ldata_content[n][8]
        sys_time[n] = ldata_content[n][9]

# sorting data set
    n = 0
    last_s = 0

    scan_set = np.array([0]*len(ldata_content))

    for s in xrange(len(scan_content)):
        if s != 0 and scan_content[s][3] != scan_content[s-1][3]:
            scan_set[n] = s - last_s
            last_s = s
            n += 1

        scan_set[len(ldata_content)-1] = s - last_s

    nSpoint = min(scan_set)

    p = 0
    n = 0

    raw_ldata_range = np.array([[1] * nSpoint] * len(scan_set), np.dtype(np.float))
    raw_ldata_angle = np.array([[1] * nSpoint] * len(scan_set), np.dtype(np.float))


    for a in xrange(len(scan_content)):
        if p > nSpoint -1:
            p = 0
            n += 1

        if int(scan_content[a][0]) > nSpoint:
            continue

        # angle is 0 idx
        raw_ldata_angle[n][p] = float(scan_content[a][1])
        raw_ldata_range[n][p] = float(scan_content[a][2])
        p += 1



    return x, y, z, raw_ldata_angle, raw_ldata_range, roll, yaw, area, sys_time

def draw_quad_model(y,z, roll):
    wing_span = 113
    motor_span = 50
    prop_span = 76
    roll = -roll
    r_wing = pg.PlotCurveItem()
    l_wing = pg.PlotCurveItem()
    r_motor = pg.PlotCurveItem()
    l_motor = pg.PlotCurveItem()
    r_prop = pg.PlotCurveItem(size=3, pen='g', brush='g')
    l_prop = pg.PlotCurveItem(size=3, pen='g', brush='g')

    rw_y = y + wing_span * cosd(roll)
    lw_y = y - wing_span * cosd(roll)
    rw_z = z + wing_span * sind(roll)
    lw_z = z - wing_span * sind(roll)

    rm_y = rw_y + motor_span * cosd(roll+90)
    lm_y = lw_y + motor_span * cosd(roll+90)
    rm_z = rw_z + motor_span * sind(roll+90)
    lm_z = lw_z + motor_span * sind(roll+90)

    rrp_y = rm_y + prop_span * cosd(roll)
    rlp_y = rm_y - prop_span * cosd(roll)
    rrp_z = rm_z + prop_span * sind(roll)
    rlp_z = rm_z - prop_span * sind(roll)

    lrp_y = lm_y + prop_span * cosd(roll)
    llp_y = lm_y - prop_span * cosd(roll)
    lrp_z = lm_z + prop_span * sind(roll)
    llp_z = lm_z - prop_span * sind(roll)

    r_wing.setData([y, rw_y],[z, rw_z])
    l_wing.setData([y, lw_y],[z, lw_z])
    r_motor.setData([rw_y, rm_y],[rw_z, rm_z])
    l_motor.setData([lw_y, lm_y],[lw_z, lm_z])

    r_prop.setData([rrp_y, rlp_y],[rrp_z, rlp_z])
    l_prop.setData([lrp_y, llp_y], [lrp_z, llp_z])

    return r_wing, l_wing, r_motor, l_motor, r_prop, l_prop

def compute_raw_scan(pdata):

    temp = np.array([[0] * 2] * pdata.nraw)
    p = 0
    for n in xrange(pdata.nraw):
        y = pdata.ldata[n][1] * -1 * sind(pdata.ldata[n][0]) # y
        z = pdata.ldata[n][1] * 1 * cosd(pdata.ldata[n][0]) # z

        if round(y) != 0 and round(z) != 0:
            temp[p][0] = round(y)
            temp[p][1] = round(z)
            p += 1

    pdata.nraw = p
    lscan = np.array([[0] * 2] * pdata.nraw)
    for n in xrange(pdata.nraw):
        lscan[n][0] = temp[n][0]
        lscan[n][1] = temp[n][1]


    lscan[:,0] = lscan[:,0] + pdata.y
    lscan[:,1] = lscan[:,1] - pdata.z

    for n in xrange(pdata.nraw):
        lscan[n][0] = np.cos(np.deg2rad(pdata.roll)) * lscan[n][0] - np.sin(np.deg2rad(pdata.roll)) * lscan[n][1]
        lscan[n][1] = -1 * (np.sin(np.deg2rad(pdata.roll)) * lscan[n][0] + np.cos(np.deg2rad(pdata.roll)) * lscan[n][1])

    return lscan


# ===============================================================
# ================== HELPER FUNCTION ============================
# ===============================================================
def cosd(a):
    a = np.cos(np.deg2rad(a))
    return a

def sind(a):
    a = np.sin(np.deg2rad(a))
    return a

def plot_percentage(y, n):

    percentage = int(n / float(len(y)) * 100)



    return percentage


