#include "../custom_header/UI.h"


char PID_tuning(char uInput, float* kp, float* ki, float* kd)
{


    static char tune = 'p';

    switch (uInput)
    {
    case 'p':
        tune = 'p';
        uInput = '~';
        break;
    case 'i':
        tune = 'i';
        uInput = '~';
        break;
    case 'd':
        tune = 'd';
        uInput = '~';
        break;
    case '+':
        if (tune =='p') *kp += 0.01;
        if (tune =='i') *ki += 0.005;
        if (tune =='d') *kd += 0.005;
        uInput = '~';
        break;
    case '-':
        if (tune =='p') *kp -= 0.01;
        if (tune =='i') *ki -= 0.005;
        if (tune =='d') *kd -= 0.005;
        uInput = '~';
        break;

    }

    return uInput;
}
void data_logging(std::ofstream& ldata_log, std::ofstream& scan_log, std::fstream& log_list, lidar_data ldata, pixhawk_data pixdata, long sys_time, float kp, float ki, float kd, int manual_ctrl, int RF, int rf2, int rf3)
{
	int nlog;
	static char prev_state = 'm';
	static long start_time = 0;

	char filename[20];

    // newly added
    static std::ofstream rf_log;


	if ((!manual_ctrl)&&(!ldata_log.is_open()))
	{
		log_list.open("log_list.txt");

		log_list >> nlog;
		log_list.close();
		log_list.open("log_list.txt", ios::out);
		log_list << nlog + 1;
		nlog++;
		log_list.close();

		snprintf(filename, sizeof filename, "ldata_%d.txt", nlog);
		ldata_log.open(filename);
		ldata_log << "PID values => kp: " << kp << " ki: " <<  ki << " kd: " << kd << endl;
        ldata_log << "Print out format:" << endl;
        ldata_log << "yc, zc, alt, CH1, CH3, is_manual?, roll, yaw, area, system time, x" << endl;

		snprintf(filename, sizeof filename, "scan_%d.txt", nlog);
		scan_log.open(filename);

        // newly added
        snprintf(filename, sizeof filename, "RF_data_%d.txt", nlog);
        rf_log.open(filename);
        rf_log << "Print out format:" << endl;
        rf_log << "y, z, rf1, rf2, rf3, time" << endl;

		start_time = sys_time + 100;

	}
	else if (!manual_ctrl)
	{
		// print out position and other info
		ldata_log << ldata.yc <<"," << ldata.zc <<"," << ldata.alt <<"," << pixdata.ch1 <<"," << pixdata.ch3 <<"," << manual_ctrl <<"," << pixdata.roll <<"," << pixdata.yaw <<"," << ldata.area <<"," << sys_time-start_time <<"," << (float) RF/1000.0 << endl;

        // print out lidar scan
		for (int i=0;i<ldata.nraw;i++)
		{
            scan_log << i+1 << ',' << ldata.angle[i] << ',' << ldata.dist[i] << ',' << sys_time-start_time <<endl;
		}

		// newly added
            rf_log << ldata.yc <<"," << ldata.zc <<"," << RF <<"," << rf2 <<"," << rf3 << endl;


    }

    if ((manual_ctrl)&&(prev_state=='a')&&(ldata_log.is_open()))
    {
    	ldata_log.close();
    	scan_log.close();


    	// newly added
    	rf_log.close();
    }

    if (!manual_ctrl)   prev_state = 'a';   // auto
    else                prev_state = 'm';   // manual
}



