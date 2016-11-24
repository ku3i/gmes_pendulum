#ifndef SETTING_H_INCLUDED
#define SETTING_H_INCLUDED

#include <stdio.h>
#include <string.h>
#include <string>

#include <common/log_messages.h>

class Setting
{
private:
    void read_options(int argc, char **argv);

public:

    Setting(int argc, char **argv);
};

#endif // SETTING_H_INCLUDED
