////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2014 MStar Semiconductor, Inc.
// All rights reserved.
//
// Unless otherwise stipulated in writing, any and all information contained
// herein regardless in any format shall remain the sole proprietary of
// MStar Semiconductor Inc. and be kept in strict confidence
// (??MStar Confidential Information??) by the recipient.
// Any unauthorized act including without limitation unauthorized disclosure,
// copying, use, reproduction, sale, distribution, modification, disassembling,
// reverse engineering and compiling of the contents of MStar Confidential
// Information is unlawful and strictly prohibited. MStar hereby reserves the
// rights to any and all damages, losses, costs and expenses resulting therefrom.
//
////////////////////////////////////////////////////////////////////////////////

/**
 *
 * @file    mstar_drv_qcom.c
 *
 * @brief   This file defines the interface of touch screen
 *
 *
 */
 
/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/kobject.h>
#include <asm/irq.h>
#include <asm/io.h>


#include "mstar_drv_platform_interface.h"

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
#include <linux/regulator/consumer.h>
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON

/*=============================================================*/
// CONSTANT VALUE DEFINITION
/*=============================================================*/

#define MSG_TP_IC_NAME "msg22xx" //"msg21xxA" or "msg22xx" or "msg26xxM" /* Please define the mstar touch ic name based on the mutual-capacitive ic or self capacitive ic that you are using */

/*=============================================================*/
// VARIABLE DEFINITION
/*=============================================================*/

struct i2c_client *g_I2cClient = NULL;

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
struct regulator *g_ReguVdd = NULL;
struct regulator *g_ReguVcc_i2c = NULL;
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON

#include <linux/sensors.h>
#include "mstar_drv_qcom.h"
#include "mstar_drv_platform_porting_layer.h"

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
extern struct input_dev *ps_inputdevice;
extern void DrvPlatformLyrTpPsEnable(int nEnable);
u8 psensor_flag;
static struct sensors_classdev sensors_ts_proximity_cdev = {
	.name = "tp_ps_proximity",
	.vendor = "liteon",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.flags = 1,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static int msg21xx_ps_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
//	struct msg21xx_ts_data *data = container_of(sensors_cdev, struct msg21xx_ts_data, msg21xx_ps_cdev);
	int ret = 0;
	psensor_flag = enable;
	if ((enable != 0) && (enable != 1)) {
		pr_err("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}
	printk("msg21xx_ps_set_enable =%d\n",enable);
//	if (enable)
	//	wake_lock(&data->wake_lock);
//	else
	//	wake_unlock(&data->wake_lock);
	DrvPlatformLyrTpPsEnable(enable);

	return ret;
}
#endif


/*=============================================================*/
// FUNCTION DEFINITION
/*=============================================================*/

/* probe function is used for matching and initializing input device */
static int /*__devinit*/ touch_driver_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
    const char *vdd_name = "vdd";
    const char *vcc_i2c_name = "vcc_i2c";
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
	struct msg21xx_ts_data* ts_data;
	int ret = 0;
#else
	int ret = 0;
#endif
    DBG("*** %s ***\n", __FUNCTION__);
    
    if (client == NULL)
    {
        DBG("i2c client is NULL\n");
        return -1;
    }
    g_I2cClient = client;

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
    g_ReguVdd = regulator_get(&g_I2cClient->dev, vdd_name);
    regulator_set_voltage(g_ReguVdd, 2600000, 3300000); 
    g_ReguVcc_i2c = regulator_get(&g_I2cClient->dev, vcc_i2c_name);
    regulator_set_voltage(g_ReguVcc_i2c, 1800000, 1800000);  
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON

    ret =  MsDrvInterfaceTouchDeviceProbe(g_I2cClient, id);
	if (ret == 0){
        #ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
		 device_init_wakeup(&client->dev, 1);
                ts_data = devm_kzalloc(&client->dev,
                                sizeof(struct msg21xx_ts_data), GFP_KERNEL);
                if (!ts_data) {
                        dev_err(&client->dev, "Not enough memory\n");
                        return -ENOMEM;
                }
                ts_data->client = client;

                ts_data->input_dev_ps = input_allocate_device();
                if (!ts_data->input_dev_ps) {
                        ret = -ENOMEM;
                        dev_err(&client->dev,"Failed to allocate input device ps\n");
			  goto free_psensor_pdata;
                }
                set_bit(EV_ABS, ts_data->input_dev_ps->evbit);
                input_set_abs_params(ts_data->input_dev_ps, ABS_DISTANCE, 0, 1, 0, 0);

                ts_data->input_dev_ps->name = "proximity";
                ts_data->input_dev_ps->id.bustype = BUS_I2C;
                ts_data->input_dev_ps->dev.parent =&ts_data->client->dev;
                input_set_drvdata(ts_data->input_dev_ps, ts_data);
                ret = input_register_device(ts_data->input_dev_ps);
                if (ret) {
                        ret = -ENOMEM;
                        dev_err(&client->dev,"Unable to register input device ps: %s\n",ts_data->input_dev_ps->name);
			  goto free_psensor_input_dev;
                }
                ps_inputdevice = ts_data->input_dev_ps;
                printk("%s input device success.\n",__func__);

                ts_data->msg21xx_ps_cdev = sensors_ts_proximity_cdev;
                ts_data->msg21xx_ps_cdev.sensors_enable = msg21xx_ps_set_enable;
                ts_data->msg21xx_ps_cdev.sensors_poll_delay = NULL;
                ret = sensors_classdev_register(&client->dev, &ts_data->msg21xx_ps_cdev);
                if(ret) {
                        ret = -EROFS;
                        dev_err(&client->dev,"Unable to register to ps sensor class\n");
			  goto unregister_psensor_input_device;
                }
		wake_lock_init(&ts_data->wake_lock, WAKE_LOCK_SUSPEND, MSG_TP_IC_NAME);
		return ret;
	unregister_psensor_input_device:
		input_unregister_device(ts_data->input_dev_ps);
	free_psensor_input_dev:
		input_free_device(ts_data->input_dev_ps);
	free_psensor_pdata:
		devm_kfree(&client->dev, ts_data);
        #endif
}
    return ret;
}

/* remove function is triggered when the input device is removed from input sub-system */
static int touch_driver_remove(struct i2c_client *client)
{
    DBG("*** %s ***\n", __FUNCTION__);

    return MsDrvInterfaceTouchDeviceRemove(client);
}

/* The I2C device list is used for matching I2C device and I2C device driver. */
static const struct i2c_device_id touch_device_id[] =
{
    {MSG_TP_IC_NAME, 0},
    {}, /* should not omitted */ 
};

MODULE_DEVICE_TABLE(i2c, touch_device_id);

static struct of_device_id touch_match_table[] =
{
    { .compatible = "mstar,msg22xx", },
    {},
};

static struct i2c_driver touch_device_driver =
{
    .driver = {
        .name = MSG_TP_IC_NAME,
        .owner = THIS_MODULE,
        .of_match_table = touch_match_table,
    },
    .probe = touch_driver_probe,
    .remove = touch_driver_remove,
    .id_table = touch_device_id,
};

static int __init touch_driver_init(void)
{
    int ret;

    /* register driver */
    ret = i2c_add_driver(&touch_device_driver);
    if (ret < 0)
    {
        DBG("add touch device driver i2c driver failed.\n");
        return -ENODEV;
    }
    DBG("add touch device driver i2c driver.\n");

    return ret;
}

static void __exit touch_driver_exit(void)
{
    DBG("remove touch device driver i2c driver.\n");

    i2c_del_driver(&touch_device_driver);
}

module_init(touch_driver_init);
module_exit(touch_driver_exit);
MODULE_LICENSE("GPL");
