#ifndef COMMON_HEADER_H
#define COMMON_HEADER_H

#include <iostream>

#define MAX_SHARED_DATA 128
#define MAX_CONFIG_DATA 157


typedef union{
    struct{
        unsigned    FET:1;	 	// FET ON   //
        unsigned    RUN:1;		// Control ON
        unsigned    INIT:1;     // Init Process Passed  //
        unsigned    MOD:1;		// Control Mode
        unsigned    NON_CTR:1;	// Nonius Error //
        unsigned    BAT:1;      // Low Battery //
        unsigned    CALIB:1;    // Calibration Mode //
        unsigned    MT_ERR:1;   // Multiturn Error //

        unsigned    JAM:1;		// JAM Error
        unsigned    CUR:1;		// Over Current Error
        unsigned    BIG:1;		// Big Position Error
        unsigned    INP:1;      // Big Input Error
        unsigned    FLT:1;		// FET Driver Fault Error //
        unsigned    TMP:1;      // Temperature Error //
        unsigned    PS1:1;		// Position Limit Error (Lower) ////
        unsigned    PS2:1;		// Position Limit Error (Upper)

        unsigned    rsvd:8;

        unsigned    CAN:1;
        unsigned    rsvd2:7;
    }b;
    unsigned char B[4];
}mSTAT;


typedef union{
    struct{
        char    header[4];
        // 0
        float   time;
        float   jnt_ref[6];
        float   jnt_ang[6];
        float   cur[6];
        // 19
        float   tcp_ref[6];
        float   tcp_pos[6];
        // 31
        float   analog_in[4];
        float   analog_out[4];
        int     digital_in[16];
        int     digital_out[16];
        // 71
        float   temperature_mc[6];
        // 77
        int     task_pc;        // int
        int     task_repeat;    // int
        int     task_run_id;    // int
        int     task_run_num;   // int
        int     task_run_time;  // int
        int     task_state;
        // 83
        float   default_speed;
        int     robot_state;
        int     power_state;
        // 86
        float   tcp_target[6];
        int     jnt_info[6];
        // 98
        int     collision_detect_onoff;
        int     is_freedrive_mode;
        int     program_mode;
        // 101
        int     init_state_info;
        int     init_error;
        // 103
        float   tfb_analog_in[2];
        int     tfb_digital_in[2];
        int     tfb_digital_out[2];
        float   tfb_voltage_out;
        // 110
        int     op_stat_collision_occur;
        int     op_stat_sos_flag;
        int     op_stat_self_collision;
        int     op_stat_soft_estop_occur;
        int     op_stat_ems_flag;
        // 115
        int     digital_in_config[2];
        // 117
        int     inbox_trap_flag[2];
        int     inbox_check_mode[2];
        // 121
        float   eft_fx;
        float   eft_fy;
        float   eft_fz;
        float   eft_mx;
        float   eft_my;
        float   eft_mz;
        // 127
    }sdata;
    float fdata[MAX_SHARED_DATA];
    int idata[MAX_SHARED_DATA];
}systemSTAT;


typedef union{
    struct{
        char    header[4];
        // 0
        float   sensitivity;
        float   work_x_min;
        float   work_x_max;
        float   work_y_min;
        float   work_y_max;
        float   work_z_min;
        float   work_z_max;
        int     work_onoff;
        float   mount_rotate[3];
        // 11
        float   toolbox_size[3];
        float   toolbox_center_pos[3];
        float   tool_mass;
        float   tool_mass_center_pos[3];
        float   tool_ee_pos[3];
        // 24
        int     usb_detected_flag;
        int     usb_copy_done_flag;
        // 26
        int     rs485_tool_baud;
        int     rs485_tool_stopbit;
        int     rs485_tool_paritybit;
        int     rs485_box_baud;
        int     rs485_box_stopbit;
        int     rs485_box_paritybit;
        // 32
        int     io_function_in[16];
        int     io_function_out[16];
        // 64
        int     ip_addr[4];
        int     netmask[4];
        int     gateway[4];
        // 76
        int     version;
        // 77
        char    default_script[64];    // 16*4
        // 93
        int     auto_init;
        float   inbox0_size[3];
        float   inbox0_pos[3];
        float   inbox1_size[3];
        float   inbox1_pos[3];
        // 106
        int     default_repeat_num;
        // 107
        float   direct_teaching_sensitivity[6];
        // 113
        float   tool_ee_ori[3];
        // 116
        float   user_coord_0[6];
        float   user_coord_1[6];
        float   user_coord_2[6];
        // 134
        char    dio_begin_box_dout[16]; // 4*4
        float   dio_begin_box_aout[4];
        char    dio_end_box_dout[16];   // 4*4
        float   dio_end_box_aout[4];
        int     dio_begin_tool_voltage; // 1
        int     dio_end_tool_voltage;   // 1
        char    dio_begin_tool_dout[2]; // 1
        char    dio_end_tool_dout[2];   // -
        // 153
        int     ext_ft_model_info;
        // 154
        int     robot_model_type;
        // 155
	int     collision_stop_mode;
        // 156
    }sdata;
    float fdata[MAX_CONFIG_DATA];
    int idata[MAX_CONFIG_DATA];
}systemCONFIG;



typedef struct{
    char    header[4];
    char    type;
    char    msg[1000];
    int     len;
}systemPOPUP;



typedef struct{
    char    header[4];
    char    msg[1000];
    int     len;
}systemGeneralCommand;



#endif // COMMON_HEADER_H
