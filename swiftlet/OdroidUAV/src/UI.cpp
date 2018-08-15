//*****************************************************************************
// UI class
//*****************************************************************************
#include "UI.h"

UI::UI()
{
    _input = "";

    init_log();
}

void UI::init_log()
{
    _log_list.open("../data/log_list.txt");
    _log_list >> nlog;
    _log_list.close();

    flag.file_is_opened = false;
}

void UI::start_log(queue<lidar_data_t> ldata_q, queue<PH2_data_t> ph2_data_q)
{
    flag.file_is_closed = false;

    if (!flag.file_is_opened)
    {
        string filename;
        string dir = "../data/";
        string fextension = ".txt";

        // file for info data
        filename = dir + "info_" + to_string(nlog) + fextension;
        info_log.open(filename);
        filename = "";

        // file for lidar scan
        filename = dir + "lscan_" + to_string(nlog) + fextension;
        lscan_log.open(filename);
        lscan_log << "lidar scan data log\n";
        lscan_log << "************* LOG FORMAT *************\n";
        lscan_log << "0, ts_odroid, ts_lidar\n";
        lscan_log << "index, angle, range (for 1080 points)\n";
        lscan_log << "************* LOG FORMAT *************\n";

        flag.file_is_opened = true;

        cout << "starting to log ...\n";
    }
    else
    {

    //=========================================================================
    // DATA INFO LOG
    //=========================================================================


    //=========================================================================
    // LIDAR SCAN DATA LOG
    //=========================================================================
        lidar_data_t ldata;
        while (!ldata_q.empty())
        {
            ldata = ldata_q.front();
            ldata_q.pop();
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
    // reset file open flag
    flag.file_is_opened = false;

    if (!flag.file_is_closed)
    {
        info_log.close();
        lscan_log.close();

        // update log number
        _log_list.open("../data/log_list.txt", ios::out);
        _log_list << ++nlog;
        _log_list.close();

        cout << "Closed log file set: " << nlog - 1 << '\n';

        flag.file_is_closed = true;
    }
}

void UI::_set_log_num(int num)
{
    _log_list.open("../data/log_list.txt", ios::out);
    _log_list << num;
    _log_list.close();
    nlog = num;
}

void UI::set_startup_time(unsigned int sys_time)
{
    _ts_startup = sys_time;
}

void UI::DEBUG_PRINT()
{
    printf("hello world %d\n", millis());
}

void UI::run()
{
    if (!flag.startup)
    {
        _print_help();
        flag.startup = true;
    }

    cin >> _input;
// debug
    if (_input == "debug")  //TODO can choose what to print
    {
        flag.debug_print = true;

    }
// log
    else if (_input == "log")
        flag.log_data = true;
// sl: stop log
    else if (_input == "sl")
        flag.log_data = false;
// sd: stop debug
    else if (_input == "sd")
        flag.debug_print = false;
// s: stop all display
    else if (_input == "s")       // idle the display, reset most flags
    {
        flag.log_data = false;
        flag.debug_print = false;
    }
// set log number
    else if (_input == "set_log")
    {
        int n;
        cout << "Set log file number to: ";
        cin >> n;
        _set_log_num(n);
        cout << "done\n";
    }
// rename log
    else if (_input == "rename_log")
    {
        string newname;
        string newfile;
        string dir = "../data/";
        string fextension = ".txt";
        int n;
        string oldname;

        cout << "Which file set to rename: ";
        cin >> n;

        cout << "New file name is: ";
        cin >> newfile;


        // file for info data
        oldname = dir + "info_" + to_string(n) + fextension;
        newname = dir + "info_" + newfile + fextension;
        if (rename(oldname.c_str(), newname.c_str()))
            cout << "error renaming info file!";

        // file for lidar scan data
        oldname = dir + "lscan_" + to_string(n) + fextension;
        newname = dir + "lscan_" + newfile + fextension;
        if (rename(oldname.c_str(), newname.c_str()))
            cout << "error renaming lscan file!";

        cout << "done\n";
    }
// help: should always be last
    else if (_input == "help")
    {
        _print_help();
    }

}

void UI::_print_help()
{
    vector <string> cmd_list = {
            "log",
            "set_log",
            "rename_log",
            "sl: stop log",
            "sd, stop debug print",
            "s: stop all display",
            "debug" //debug should always be last
    };

    vector <string> debug_list; //TODO

    cout << "**** list of commands ****\n";
    for (int i=0; i<cmd_list.size(); i++)
    {
        cout << "   " << i+1 << ") " << cmd_list[i] << '\n';
    }
    cout << "**************************\n";
}
