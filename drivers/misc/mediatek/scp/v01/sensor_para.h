#ifndef __SENSOR_PARA_H__
#define __SENSOR_PARA_H__

struct acc_para_t {
	int8_t axis_map_x;
        int8_t axis_map_y;
        int8_t axis_map_z;
};

struct ps_para_t {
        uint8_t cali_order;
};

struct sensor_para_t {
	struct acc_para_t acc_para;
	struct ps_para_t ps_para;
};

extern struct sensor_para_t * sensor_para;

#endif /* __SENSOR_PARA_H__ */
