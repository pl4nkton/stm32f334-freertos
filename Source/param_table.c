#include "param_table.h"
#include "util.h"

static int board_address;

const struct param_info  param_table[] = {
    {    1, P_INT32(&board_address    ), READONLY, .name = "board_address" },


};


const int param_count = ARRAY_SIZE(param_table);
