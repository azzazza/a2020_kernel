/*wangwenwen add H file */
#include <linux/sensors.h>
#include "mstar_drv_common.h"
#include <linux/wakelock.h>
struct msg21xx_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct input_dev *input_dev_ps;
	struct sensors_classdev msg21xx_ps_cdev;	
	struct msg21xx_ts_platform_data *pdata;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	bool suspended;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
	struct pinctrl_state *pinctrl_state_release;
	struct mutex ts_mutex;
	//struct touchInfo_t info;
	struct wake_lock wake_lock;
	unsigned char ps_open_state;	
};

extern struct msg21xx_ts_data *ts_data;



