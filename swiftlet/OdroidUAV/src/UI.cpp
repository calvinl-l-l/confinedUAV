//*****************************************************************************
// UI class
//*****************************************************************************
#include "UI.h"

UI::UI()
{
    input = "";

    init_log();
}

void UI::init_log()
{
    _log_list.open("../data/log_list.txt");
    _log_list >> _nlog;
    _log_list.close();

    _flag_file_is_opened = false;

}

void UI::start_log(queue<lidar_data_t> ldata_q, queue<PH2_data_t> ph2_data_q)
{
    if (!_flag_file_is_opened)
    {
        char filename[20];

        // file for info data
        snprintf(filename, sizeof filename, "../data/info_%d.txt", _nlog);
        info_log.open(filename);

        // file for lidar scan
        snprintf(filename, sizeof filename, "../data/lscan_%d.txt", _nlog);
        lscan_log.open(filename);
        lscan_log << "lidar scan data log\n";
        lscan_log << "************* LOG FORMAT *************\n";
        lscan_log << "0, ts_odroid, ts_lidar\n";
        lscan_log << "index, angle, range (for 1080 points)\n";
        lscan_log << "************* LOG FORMAT *************\n";

        _flag_file_is_opened = true;

        cout << "opening\n";
    }
    else
    {
        cout << "else\n";
    //=========================================================================
    // DATA INFO LOG
    //=========================================================================
        info_log << "Hello world";

    //=========================================================================
    // LIDAR SCAN DATA LOG
    //=========================================================================
        lidar_data_t ldata;
        while (!ldata_q.empty())
        {
            ldata = ldata_q.front();
            ldata_q.pop();
            cout << "hi\n";
            lscan_log << "0," << ldata.ts_odroid << ',' << ldata.ts_lidar << '\n';
            for (int i=0;i<540*2;i++)
            {
                lscan_log << i+1 << ',' << ldata.angle[i] << ',' << ldata.range[i] << '\n';
            }
        }
    }
}

void UI::end_log()
{
    info_log.close();
    lscan_log.close();

    // reset file open flag
    _flag_file_is_opened = false;

    // update log number
    _log_list.open("../data/log_list.txt", ios::out);
    _log_list << ++_nlog;
    _log_list.close();

    cout << "log files closed . . .\n";
}

void UI::set_log_num(int num)
{
    _log_list.open("../data/log_list.txt", ios::out);
    _log_list << num;
    _log_list.close();
}

void UI::set_startup_time(unsigned int sys_time)
{
    _ts_startup = sys_time;
}

void UI::DEBUG_PRINT()
{

}
