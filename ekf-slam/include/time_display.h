#pragma once
#ifndef TIME_DISP_H
#define TIME_DISP_H

#include <ctime>
#include <iomanip>
#include <ctime>
#include <sstream>

#include "drawer.h"

using namespace std;

class TimeDisplay
{
public:
    typedef shared_ptr<TimeDisplay> Ptr;
    mutex data_mutex;

    TimeDisplay()
    {
        stop = false;
    }

    Drawer::Ptr drawer;
    bool stop;

    void DisplayTime()
    {
        while (!stop)
        {
            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);

            std::ostringstream oss;
            oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
            auto str = oss.str();

            drawer->SetNewTime(str, true);
        }
    }

    void SetStop(bool ss)
    {
        std::unique_lock<std::mutex> lck(data_mutex);
        stop = ss;
    }
};

#endif