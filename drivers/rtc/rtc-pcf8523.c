/*
 * Copyright (C) 2012 Avionic Design GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/bcd.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/rtc.h>
#include <linux/of.h>

#define DRIVER_NAME "rtc-pcf8523"

#define REG_CONTROL1 0x00
#define REG_CONTROL1_CAP_SEL (1 << 7)
#define REG_CONTROL1_STOP    (1 << 5)

#define REG_CONTROL3 0x02
#define REG_CONTROL3_PM_BLD (1 << 7) /* battery low detection disabled */
#define REG_CONTROL3_PM_VDD (1 << 6) /* switch-over disabled */
#define REG_CONTROL3_PM_DSM (1 << 5) /* direct switching mode */
#define REG_CONTROL3_PM_MASK 0xe0
#define REG_CONTROL3_BLF (1 << 2) /* battery low bit, read-only */

#define REG_SECONDS  0x03
#define REG_SECONDS_OS (1 << 7)

#define REG_MINUTES  0x04
#define REG_HOURS    0x05
#define REG_DAYS     0x06
#define REG_WEEKDAYS 0x07
#define REG_MONTHS   0x08
#define REG_YEARS    0x09

#define PCF8523_REG_ALM1_MN        0x0a        /* Minutes */
#define PCF8523_REG_ALM1_HR        0x0b        /* Hours */
#define PCF8523_REG_ALM1_DT        0x0c        /* Day of month 1-31 */
#define PCF8523_REG_ALM1_WK        0x0d        /* weekday alarm */

#define REG_CONTROL1_AIE  (1 << 1) /* min,hr,day,mon alarm 1 */

#define REG_CONTROL2	0x01
#define REG_CONTROL2_AF_CLEAR	(0 << 3) //need to be cleared,flag set when alarm triggered
#define REG_CONTROL2_AF_SET	(1 << 3) //need to be cleared,flag set when alarm triggered

#define PCF8523_HR_PM                BIT(5) 

struct pcf8523 {
	struct i2c_client *client;
	struct rtc_device *rtc;
	struct work_struct work;
        struct regmap *regmap; 

	/* The mutex protects alarm operations, and prevents a race
	 * between the enable_irq() in the workqueue and the free_irq()
	 * in the remove function.
	 */
	struct mutex mutex;
	bool suspended;
	int irq;
        bool mode_12h; 
};

static int pcf8523_bcd12h_to_bin24h(int regval)
{
        int hr = bcd2bin(regval & 0x1f);
        bool pm = regval & PCF8523_HR_PM;

        if (hr == 12)
                return pm ? 12 : 0;

        return pm ? hr + 12 : hr;
}

static int pcf8523_bin24h_to_bcd12h(int hr24)
{
        bool pm = hr24 >= 12;
        int hr12 = hr24 % 12;

        if (!hr12)
                hr12++;

        return bin2bcd(hr12) | pm ? 0 : PCF8523_HR_PM;
}
 
static int pcf8523_read(struct i2c_client *client, u8 reg, u8 *valuep)
{
	struct i2c_msg msgs[2];
	u8 value = 0;
	int err;

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(reg);
	msgs[0].buf = &reg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = sizeof(value);
	msgs[1].buf = &value;

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err < 0)
		return err;

	*valuep = value;

	return 0;
}

static int pcf8523_write(struct i2c_client *client, u8 reg, u8 value)
{
	u8 buffer[2] = { reg, value };
	struct i2c_msg msg;
	int err;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = sizeof(buffer);
	msg.buf = buffer;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err < 0)
		return err;

	return 0;
}

static int pcf8523_select_capacitance(struct i2c_client *client, bool high)
{
	u8 value;
	int err;

	err = pcf8523_read(client, REG_CONTROL1, &value);
	if (err < 0)
		return err;

	if (!high)
		value &= ~REG_CONTROL1_CAP_SEL;
	else
		value |= REG_CONTROL1_CAP_SEL;

	err = pcf8523_write(client, REG_CONTROL1, value);
	if (err < 0)
		return err;

	return err;
}

static int pcf8523_set_pm(struct i2c_client *client, u8 pm)
{
	u8 value;
	int err;

	err = pcf8523_read(client, REG_CONTROL3, &value);
	if (err < 0)
		return err;

	value = (value & ~REG_CONTROL3_PM_MASK) | pm;

	err = pcf8523_write(client, REG_CONTROL3, value);
	if (err < 0)
		return err;

	return 0;
}

static int pcf8523_stop_rtc(struct i2c_client *client)
{
	u8 value;
	int err;

	err = pcf8523_read(client, REG_CONTROL1, &value);
	if (err < 0)
		return err;

	value |= REG_CONTROL1_STOP;

	err = pcf8523_write(client, REG_CONTROL1, value);
	if (err < 0)
		return err;

	return 0;
}

static int pcf8523_start_rtc(struct i2c_client *client)
{
	u8 value;
	int err;

	err = pcf8523_read(client, REG_CONTROL1, &value);
	if (err < 0)
		return err;

	value &= ~REG_CONTROL1_STOP;

	err = pcf8523_write(client, REG_CONTROL1, value);
	if (err < 0)
		return err;

	return 0;
}

static int pcf8523_enable_alarm(struct i2c_client *client, bool enable)
{
	u8 value;
	int ret;

	ret = pcf8523_read(client, REG_CONTROL1, &value);
	if(ret!=0)
        	return -1;
	
	if(enable)
		value |= REG_CONTROL1_AIE;
	else
		value &= ~(REG_CONTROL1_AIE);
	
	ret = pcf8523_write(client, REG_CONTROL1, value);
	if(ret!=0)
        	return -1;

	return 0;


}
 
static int pcf8523_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
        struct pcf8523 *pcf8523 = dev_get_drvdata(dev);
	struct i2c_client *client = to_i2c_client(dev);
        struct rtc_time *tm = &alarm->time;
        u8 first = PCF8523_REG_ALM1_MN;
        u8 last = PCF8523_REG_ALM1_WK;
        int len = last - first + 1;
        u8 regs[len];
        u8 hr_reg,value;
        int ret;
	struct i2c_msg msgs[2];

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &first;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = regs;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;

        tm->tm_min = bcd2bin(regs[PCF8523_REG_ALM1_MN - first] & 0x7f);

        hr_reg = regs[PCF8523_REG_ALM1_HR - first];

        if (pcf8523->mode_12h)
                tm->tm_hour = pcf8523_bcd12h_to_bin24h(hr_reg);
        else
                tm->tm_hour = bcd2bin(hr_reg & 0x3f);

        tm->tm_mday = bcd2bin(regs[PCF8523_REG_ALM1_DT - first]);
        tm->tm_wday  = bcd2bin(regs[PCF8523_REG_ALM1_WK - first]) - 1;
        tm->tm_year = -1;
        tm->tm_mon = -1;

        ret = pcf8523_read(client, REG_CONTROL1, &value);
        if (ret)
                return ret;
        alarm->enabled = !!(value & REG_CONTROL1_AIE);

        ret = pcf8523_read(client, REG_CONTROL2, &value);
        if (ret)
                return ret;
        alarm->pending = !!(value & REG_CONTROL2_AF_SET);

        return 0;
}

static int pcf8523_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
        struct pcf8523 *pcf8523 = dev_get_drvdata(dev);
	struct i2c_client *client = to_i2c_client(dev);
        struct rtc_time *tm = &alarm->time;
        u8 first = PCF8523_REG_ALM1_MN;
        u8 last = PCF8523_REG_ALM1_WK;
        int len = last - first + 1;
        u8 regs[len];
        u8 value;
        int ret;
	struct i2c_msg msg;

        /* Disable alarm comparison during update */
        ret = pcf8523_enable_alarm(client, false);
        if (ret)
                return ret;

        /* Clear any pending alarm (write 0=>clr, 1=>no change) */
        ret = pcf8523_read(client, REG_CONTROL2, &value);
        if (ret)
                return ret;

        /* Set the alarm time registers */
        regs[PCF8523_REG_ALM1_MN - first] = bin2bcd(tm->tm_min);
        regs[PCF8523_REG_ALM1_HR - first] = pcf8523->mode_12h ?
                        pcf8523_bin24h_to_bcd12h(tm->tm_hour) :
                        bin2bcd(tm->tm_hour);
        regs[PCF8523_REG_ALM1_DT - first] = bin2bcd(tm->tm_mday);
        regs[PCF8523_REG_ALM1_WK - first] = bin2bcd(tm->tm_mon + 1);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = sizeof(regs);
	msg.buf = regs;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		return ret;
	}

        if (alarm->enabled)
                ret = pcf8523_enable_alarm(client, true);

        return ret;
}

static int pcf8523_alarm_irq_enable(struct device *dev, unsigned int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	return pcf8523_enable_alarm(client, !!enable); 
}
 
static irqreturn_t pcf8523_irq(int irq, void *dev_id)
{
	struct i2c_client *client = dev_id;
	struct pcf8523 *pcf8523 = i2c_get_clientdata(client);
        int ret;
	u8 value;

        ret = pcf8523_read(client, REG_CONTROL2, &value);
        if (ret)
                return IRQ_NONE;

        if (value & REG_CONTROL2_AF_CLEAR) {
                pcf8523_write(client,REG_CONTROL2,
                             ~REG_CONTROL2_AF_CLEAR);

                rtc_update_irq(pcf8523->rtc, 1, RTC_IRQF | RTC_AF);

                return IRQ_HANDLED;
        }

        return IRQ_NONE;
}

static int pcf8523_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 start = REG_SECONDS, regs[7];
	struct i2c_msg msgs[2];
	int err;

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &start;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = sizeof(regs);
	msgs[1].buf = regs;

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err < 0)
		return err;

	if (regs[0] & REG_SECONDS_OS) {
		/*
		 * If the oscillator was stopped, try to clear the flag. Upon
		 * power-up the flag is always set, but if we cannot clear it
		 * the oscillator isn't running properly for some reason. The
		 * sensible thing therefore is to return an error, signalling
		 * that the clock cannot be assumed to be correct.
		 */

		regs[0] &= ~REG_SECONDS_OS;

		err = pcf8523_write(client, REG_SECONDS, regs[0]);
		if (err < 0)
			return err;

		err = pcf8523_read(client, REG_SECONDS, &regs[0]);
		if (err < 0)
			return err;

		if (regs[0] & REG_SECONDS_OS)
			return -EAGAIN;
	}

	tm->tm_sec = bcd2bin(regs[0] & 0x7f);
	tm->tm_min = bcd2bin(regs[1] & 0x7f);
	tm->tm_hour = bcd2bin(regs[2] & 0x3f);
	tm->tm_mday = bcd2bin(regs[3] & 0x3f);
	tm->tm_wday = regs[4] & 0x7;
	tm->tm_mon = bcd2bin(regs[5] & 0x1f) - 1;
	tm->tm_year = bcd2bin(regs[6]) + 100;

	return rtc_valid_tm(tm);
}

static int pcf8523_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_msg msg;
	u8 regs[8];
	int err;

	err = pcf8523_stop_rtc(client);
	if (err < 0)
		return err;

	regs[0] = REG_SECONDS;
	regs[1] = bin2bcd(tm->tm_sec);
	regs[2] = bin2bcd(tm->tm_min);
	regs[3] = bin2bcd(tm->tm_hour);
	regs[4] = bin2bcd(tm->tm_mday);
	regs[5] = tm->tm_wday;
	regs[6] = bin2bcd(tm->tm_mon + 1);
	regs[7] = bin2bcd(tm->tm_year - 100);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = sizeof(regs);
	msg.buf = regs;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err < 0) {
		/*
		 * If the time cannot be set, restart the RTC anyway. Note
		 * that errors are ignored if the RTC cannot be started so
		 * that we have a chance to propagate the original error.
		 */
		pcf8523_start_rtc(client);
		return err;
	}

	return pcf8523_start_rtc(client);
}

#ifdef CONFIG_RTC_INTF_DEV
static int pcf8523_rtc_ioctl(struct device *dev, unsigned int cmd,
			     unsigned long arg)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 value;
	int ret = 0, err;

	switch (cmd) {
	case RTC_VL_READ:
		err = pcf8523_read(client, REG_CONTROL3, &value);
		if (err < 0)
			return err;

		if (value & REG_CONTROL3_BLF)
			ret = 1;

		if (copy_to_user((void __user *)arg, &ret, sizeof(int)))
			return -EFAULT;

		return 0;
	default:
		return -ENOIOCTLCMD;
	}
}
#else
#define pcf8523_rtc_ioctl NULL
#endif

static const struct rtc_class_ops pcf8523_rtc_ops = {
	.read_time = pcf8523_rtc_read_time,
	.set_time = pcf8523_rtc_set_time,
	.read_alarm = pcf8523_read_alarm,
	.set_alarm = pcf8523_set_alarm,
	.alarm_irq_enable = pcf8523_alarm_irq_enable,
	.ioctl = pcf8523_rtc_ioctl,
};

static int pcf8523_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct pcf8523 *pcf;
	int err;
        struct device *dev = &client->dev;
 
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	pcf = devm_kzalloc(&client->dev, sizeof(*pcf), GFP_KERNEL);
	if (!pcf)
		return -ENOMEM;

	err = pcf8523_select_capacitance(client, true);
	if (err < 0)
		return err;

	err = pcf8523_set_pm(client, 0);
	if (err < 0)
		return err;

        pcf->irq = client->irq;
        dev_set_drvdata(dev, pcf);
      	pcf->client = client;  
        
        if (pcf->irq) {
		printk("int irq=%d\n",pcf->irq);
                err = devm_request_threaded_irq(dev, pcf->irq, NULL,
                                                pcf8523_irq,
                                                IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND,
                                                "pcf8523 alarm", pcf);
                if (err) {
                        dev_err(dev, "irq %d unavailable (%d)\n",
                                pcf->irq, err);
                        pcf->irq = 0;
                }
        }

        if (pcf->irq || of_property_read_bool(dev->of_node, "wakeup-source"))
                device_init_wakeup(dev, true); 	

	pcf->rtc = devm_rtc_device_register(&client->dev, DRIVER_NAME,
				       &pcf8523_rtc_ops, THIS_MODULE);
	if (IS_ERR(pcf->rtc))
		return PTR_ERR(pcf->rtc);

	i2c_set_clientdata(client, pcf);

	return 0;
}

static int pcf8523_remove(struct i2c_client *client)
{
        struct pcf8523 *pcf8523 = i2c_get_clientdata(client);
        struct device *dev = &client->dev;

        if (pcf8523->irq || of_property_read_bool(dev->of_node, "wakeup-source"))
                device_init_wakeup(dev, false);

        return 0;
}

#ifdef CONFIG_PM_SLEEP
static int pcf8523_suspend(struct device *dev)
{
        struct pcf8523 *pcf8523 = dev_get_drvdata(dev);
        int ret = 0;

        if (device_may_wakeup(dev))
                ret = enable_irq_wake(pcf8523->irq);

        return ret;
}

static int pcf8523_resume(struct device *dev)
{
        struct pcf8523 *pcf8523 = dev_get_drvdata(dev);
        int ret = 0;

        if (device_may_wakeup(dev))
                ret = disable_irq_wake(pcf8523->irq);

        return ret;
}

#endif 

static const struct i2c_device_id pcf8523_id[] = {
	{ "pcf8523", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pcf8523_id);

#ifdef CONFIG_OF
static const struct of_device_id pcf8523_of_match[] = {
	{ .compatible = "nxp,pcf8523" },
	{ }
};
MODULE_DEVICE_TABLE(of, pcf8523_of_match);
#endif

static SIMPLE_DEV_PM_OPS(pcf8523_pm_ops, pcf8523_suspend,  pcf8523_resume); 

static struct i2c_driver pcf8523_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(pcf8523_of_match),
		.pm = &pcf8523_pm_ops, 
	},
	.probe = pcf8523_probe,
        .remove = pcf8523_remove, 
	.id_table = pcf8523_id,
};
module_i2c_driver(pcf8523_driver);

MODULE_AUTHOR("Thierry Reding <thierry.reding@avionic-design.de>");
MODULE_DESCRIPTION("NXP PCF8523 RTC driver");
MODULE_LICENSE("GPL v2");
