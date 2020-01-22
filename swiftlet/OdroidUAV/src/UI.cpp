//*****************************************************************************
// UI class
//*****************************************************************************
#include "UI.h"

UI::UI()
{
    _input = "";
    data_log_density = 1;   // default

    init_log();
}

void UI::init_log()
{
    _log_list.open("../data/log_list.txt");
    _log_list >> nlog;
    _log_list.close();

    flag.file_is_opened = false;
}



void UI::end_log()
{
    // reset file open flag
    flag.file_is_opened = false;

    if (!flag.file_is_closed)
    {
        lscan_log.close();
        control_log.close();

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
    else if (_input == "logd")
    {
        cout << "log data density (highest) 1-4 (lowest):\n";
        cout << "1080 542 360 270 pts, select: ";
        cin >> data_log_density;
        flag.log_data = true;
    }
    else if (_input == "log")
    {
        flag.log_data = true;
    }
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
        string oldname;
        int n;


        cout << "Which file set to rename: ";
        cin >> n;

        cout << "New file name is: ";
        cin >> newfile;


        // file for info data
        oldname = dir + "info_" + to_string(n) + ".txt";
        newname = dir + "info_" + newfile + ".txt";
        if (rename(oldname.c_str(), newname.c_str()))
            cout << "error renaming info file!";

        // file for lidar scan data
        oldname = dir + "lscan_" + to_string(n) + ".dat";
        newname = dir + "lscan_" + newfile + ".dat";
        if (rename(oldname.c_str(), newname.c_str()))
            cout << "error renaming lscan file!";

        cout << "done\n";
    }
// set altitude type
    else if (_input == "alt")
    {
        int n = 0;
        cout << "Setting altitude type to:\n";
        cout << "0 - floor\n";
        cout << "1 - roof\n";
        cout << "2 - tunnel\n";
        cin >> n;

        switch(n)
        {
            case 0:
                lidar_CMD.alt_type = FLOOR;
                break;
            case 1:
                lidar_CMD.alt_type = ROOF;
                break;
            case 2:
                lidar_CMD.alt_type = TUNNEL;
                break;
        }

        lidar_CMD.set_type = true;
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
            "log or logd to choose log data density - default:1",
            "set_log",
            "rename_log",
            "sl: stop log",
            "sd, stop debug print",
            "s: stop all display",
            "alt",
            "debug" //debug should always be last
    };

    vector <string> debug_list; //TODO: choose what to print

    cout << "******** list of commands ****************\n\n";
    for (int i=0; i<cmd_list.size(); i++)
    {
        cout << "   " << i+1 << ") " << cmd_list[i] << '\n';
    }
    cout << "\n******************************************\n";
}
