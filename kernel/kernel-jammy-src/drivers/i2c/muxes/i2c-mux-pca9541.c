/*
 * I2C multiplexer driver for PCA9541 bus master selector
 *
 * Copyright (c) 2010 Ericsson AB.
 *
 * Author: Guenter Roeck <linux@roeck-us.net>
 *
 * Derived from:
 *  pca954x.c
 *
 *  Copyright (c) 2008-2009 Rodolfo Giometti <giometti@linux.it>
 *  Copyright (c) 2008-2009 Eurotech S.p.A. <info@eurotech.it>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/kernel.h>
#include <linux/string.h>

/*
 * The PCA9541/PCA9641 is a bus master selector. It supports two I2C masters
 * connected to a single slave bus.
 *
 * Before each bus transaction, a master has to acquire bus ownership. After the
 * transaction is complete, bus ownership has to be released. This fits well
 * into the I2C multiplexer framework, which provides select and release
 * functions for this purpose. For this reason, this driver is modeled as
 * single-channel I2C bus multiplexer.
 *
 * This driver assumes that the two bus masters are controlled by two different
 * hosts. If a single host controls both masters, platform code has to ensure
 * that only one of the masters is instantiated at any given time.
 */

#define PCA9541_CONTROL		0x01
#define PCA9541_ISTAT		0x02

#define PCA9541_CTL_MYBUS	BIT(0)
#define PCA9541_CTL_NMYBUS	BIT(1)
#define PCA9541_CTL_BUSON	BIT(2)
#define PCA9541_CTL_NBUSON	BIT(3)
#define PCA9541_CTL_BUSINIT	BIT(4)
#define PCA9541_CTL_TESTON	BIT(6)
#define PCA9541_CTL_NTESTON	BIT(7)

#define PCA9541_ISTAT_INTIN	BIT(0)
#define PCA9541_ISTAT_BUSINIT	BIT(1)
#define PCA9541_ISTAT_BUSOK	BIT(2)
#define PCA9541_ISTAT_BUSLOST	BIT(3)
#define PCA9541_ISTAT_MYTEST	BIT(6)
#define PCA9541_ISTAT_NMYTEST	BIT(7)

#define PCA9641_ID				0x00
#define PCA9641_ID_MAGIC		0x38

#define PCA9641_CONTROL			0x01
#define PCA9641_STATUS			0x02
#define PCA9641_TIME			0x03
#define PCA9641_INT_STATUS      0x04
#define PCA9641_INT_MASK        0x05
#define PCA9641_REG_DUMP_NUM    0x06
#define PCA9641_MBOX_LO         0x06
#define PCA9641_MBOX_HI         0x07

#define PCA9641_CTL_LOCK_REQ			BIT(0)
#define PCA9641_CTL_LOCK_GRANT			BIT(1)
#define PCA9641_CTL_BUS_CONNECT	 		BIT(2)
#define PCA9641_CTL_BUS_INIT			BIT(3)
#define PCA9641_CTL_SMBUS_SWRST	 		BIT(4)
#define PCA9641_CTL_IDLE_TIMER_DIS		BIT(5)
#define PCA9641_CTL_SMBUS_DIS	 		BIT(6)
#define PCA9641_CTL_PRIORITY			BIT(7)

#define PCA9641_STS_OTHER_LOCK	 		BIT(0)
#define PCA9641_STS_BUS_INIT_FAIL		BIT(1)
#define PCA9641_STS_BUS_HUNG	 		BIT(2)
#define PCA9641_STS_MBOX_EMPTY	 		BIT(3)
#define PCA9641_STS_MBOX_FULL			BIT(4)
#define PCA9641_STS_TEST_INT	 		BIT(5)
#define PCA9641_STS_SCL_IO		 		BIT(6)
#define PCA9641_STS_SDA_IO		 		BIT(7)

#define PCA9641_INT_BUS_HUNG_STS        BIT(6)
#define PCA9641_INT_MBOX_FULL_STS       BIT(5)
#define PCA9641_INT_MBOX_EMPTY_STS      BIT(4)
#define PCA9641_INT_TEST_INT_STS        BIT(3)
#define PCA9641_INT_LOCK_GRANT_STS      BIT(2)
#define PCA9641_INT_LOST_STS            BIT(1)
#define PCA9641_INT_IN_STS              BIT(0)

#define PCA9641_INT_BUS_HUNG_MSK        BIT(6)
#define PCA9641_INT_MBOX_FULL_MSK       BIT(5)
#define PCA9641_INT_MBOX_EMPTY_MSK      BIT(4)
#define PCA9641_TEST_INT_MASK           BIT(3)
#define PCA9641_INT_LOCK_GRANT_MASK     BIT(2)
#define PCA9641_INT_LOST_MASK           BIT(1)
#define PCA9641_INT_IN_MASK             BIT(0)

#define PCA9641_RES_TIME		0x03

#define BUSON		(PCA9541_CTL_BUSON | PCA9541_CTL_NBUSON)
#define MYBUS		(PCA9541_CTL_MYBUS | PCA9541_CTL_NMYBUS)
#define mybus(x)	(!((x) & MYBUS) || ((x) & MYBUS) == MYBUS)
#define busoff(x)	(!((x) & BUSON) || ((x) & BUSON) == BUSON)

#define BUSOFF(x, y)	(!((x) & PCA9641_CTL_LOCK_GRANT) && \
						!((y) & PCA9641_STS_OTHER_LOCK))
#define other_lock(x)	((x) & PCA9641_STS_OTHER_LOCK)
#define lock_grant(x)	((x) & PCA9641_CTL_LOCK_GRANT)
#define tst_bit(n, val) ((((val) >> (n)) & 0x01)^1)
#define toggle_bit(n, val) ((val) ^= (1UL << (n)))

#define BYTE_TO_BINARY_PATTERN "0x%.2x:0b%c%c%c%c%c%c%c%c\n"
#define BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0')

/* arbitration timeouts, in jiffies */
#define ARB_TIMEOUT	(HZ / 8)	/* 125 ms until forcing bus ownership */
#define ARB2_TIMEOUT	(HZ / 4)	/* 250 ms until acquisition failure */
#define ARB_TIMEOUT	(HZ / 2)		/* 500 ms until forcing bus ownership */
#define ARB2_TIMEOUT	(HZ)			/* 1000 ms until acquisition failure */

/* arbitration retry delays, in us */
#define SELECT_DELAY_SHORT	50
#define SELECT_DELAY_LONG	1000

struct pca9541 {
	struct i2c_client *client;
	unsigned long select_timeout;
	unsigned long arb_timeout;
	struct attribute_group *attr_group;
	struct attribute **attrs;
};

static const struct i2c_device_id pca9541_id[] = {
	{"pca9541", 0},
	{"pca9641", 1},
	{}
};

MODULE_DEVICE_TABLE(i2c, pca9541_id);

#ifdef CONFIG_OF
static const struct of_device_id pca9541_of_match[] = {
	{ .compatible = "nxp,pca9541" },
	{ .compatible = "nxp,pca9641" },
	{}
};
MODULE_DEVICE_TABLE(of, pca9541_of_match);
#endif

/*
 * Write to chip register. Don't use i2c_transfer()/i2c_smbus_xfer()
 * as they will try to lock the adapter a second time.
 */
static int pca9541_reg_write(struct i2c_client *client, u8 command, u8 val)
{
	struct i2c_adapter *adap = client->adapter;
	union i2c_smbus_data data = { .byte = val };

	return __i2c_smbus_xfer(adap, client->addr, client->flags,
				I2C_SMBUS_WRITE, command,
				I2C_SMBUS_BYTE_DATA, &data);
}

/*
 * Read from chip register. Don't use i2c_transfer()/i2c_smbus_xfer()
 * as they will try to lock adapter a second time.
 */
static int pca9541_reg_read(struct i2c_client *client, u8 command)
{
	struct i2c_adapter *adap = client->adapter;
	union i2c_smbus_data data;
	int ret;

	ret = __i2c_smbus_xfer(adap, client->addr, client->flags,
			       I2C_SMBUS_READ, command,
			       I2C_SMBUS_BYTE_DATA, &data);

	return ret ?: data.byte;
}

/*
 * Arbitration management functions
 */

/* Release bus. Also reset NTESTON and BUSINIT if it was set. */
static void pca9541_release_bus(struct i2c_client *client)
{
	int reg;

	reg = pca9541_reg_read(client, PCA9541_CONTROL);
	if (reg >= 0 && !busoff(reg) && mybus(reg))
		pca9541_reg_write(client, PCA9541_CONTROL,
				  (reg & PCA9541_CTL_NBUSON) >> 1);
}

/*
 * Arbitration is defined as a two-step process. A bus master can only activate
 * the slave bus if it owns it; otherwise it has to request ownership first.
 * This multi-step process ensures that access contention is resolved
 * gracefully.
 *
 * Bus	Ownership	Other master	Action
 * state		requested access
 * ----------------------------------------------------
 * off	-		yes		wait for arbitration timeout or
 *					for other master to drop request
 * off	no		no		take ownership
 * off	yes		no		turn on bus
 * on	yes		-		done
 * on	no		-		wait for arbitration timeout or
 *					for other master to release bus
 *
 * The main contention point occurs if the slave bus is off and both masters
 * request ownership at the same time. In this case, one master will turn on
 * the slave bus, believing that it owns it. The other master will request
 * bus ownership. Result is that the bus is turned on, and master which did
 * _not_ own the slave bus before ends up owning it.
 */

/* Control commands per PCA9541 datasheet */
static const u8 pca9541_control[16] = {
	4, 0, 1, 5, 4, 4, 5, 5, 0, 0, 1, 1, 0, 4, 5, 1
};

/*
 * Channel arbitration
 *
 * Return values:
 *  <0: error
 *  0 : bus not acquired
 *  1 : bus acquired
 */
static int pca9541_arbitrate(struct i2c_client *client)
{
	struct i2c_mux_core *muxc = i2c_get_clientdata(client);
	struct pca9541 *data = i2c_mux_priv(muxc);
	int reg;

	reg = pca9541_reg_read(client, PCA9541_CONTROL);
	if (reg < 0)
		return reg;

	if (busoff(reg)) {
		int istat;
		/*
		 * Bus is off. Request ownership or turn it on unless
		 * other master requested ownership.
		 */
		istat = pca9541_reg_read(client, PCA9541_ISTAT);
		if (!(istat & PCA9541_ISTAT_NMYTEST)
		    || time_is_before_eq_jiffies(data->arb_timeout)) {
			/*
			 * Other master did not request ownership,
			 * or arbitration timeout expired. Take the bus.
			 */
			pca9541_reg_write(client,
					  PCA9541_CONTROL,
					  pca9541_control[reg & 0x0f]
					  | PCA9541_CTL_NTESTON);
			data->select_timeout = SELECT_DELAY_SHORT;
		} else {
			/*
			 * Other master requested ownership.
			 * Set extra long timeout to give it time to acquire it.
			 */
			data->select_timeout = SELECT_DELAY_LONG * 2;
		}
	} else if (mybus(reg)) {
		/*
		 * Bus is on, and we own it. We are done with acquisition.
		 * Reset NTESTON and BUSINIT, then return success.
		 */
		if (reg & (PCA9541_CTL_NTESTON | PCA9541_CTL_BUSINIT))
			pca9541_reg_write(client,
					  PCA9541_CONTROL,
					  reg & ~(PCA9541_CTL_NTESTON
						  | PCA9541_CTL_BUSINIT));
		return 1;
	} else {
		/*
		 * Other master owns the bus.
		 * If arbitration timeout has expired, force ownership.
		 * Otherwise request it.
		 */
		data->select_timeout = SELECT_DELAY_LONG;
		if (time_is_before_eq_jiffies(data->arb_timeout)) {
			/* Time is up, take the bus and reset it. */
			pca9541_reg_write(client,
					  PCA9541_CONTROL,
					  pca9541_control[reg & 0x0f]
					  | PCA9541_CTL_BUSINIT
					  | PCA9541_CTL_NTESTON);
		} else {
			/* Request bus ownership if needed */
			if (!(reg & PCA9541_CTL_NTESTON))
				pca9541_reg_write(client,
						  PCA9541_CONTROL,
						  reg | PCA9541_CTL_NTESTON);
		}
	}
	return 0;
}

static int pca9541_select_chan(struct i2c_mux_core *muxc, u32 chan)
{
	struct pca9541 *data = i2c_mux_priv(muxc);
	struct i2c_client *client = data->client;
	int ret;
	unsigned long timeout = jiffies + ARB2_TIMEOUT;
		/* give up after this time */

	data->arb_timeout = jiffies + ARB_TIMEOUT;
		/* force bus ownership after this time */

	do {
		ret = pca9541_arbitrate(client);
		if (ret)
			return ret < 0 ? ret : 0;

		if (data->select_timeout == SELECT_DELAY_SHORT)
			udelay(data->select_timeout);
		else
			msleep(data->select_timeout / 1000);
	} while (time_is_after_eq_jiffies(timeout));

	return -ETIMEDOUT;
}

static int pca9541_release_chan(struct i2c_mux_core *muxc, u32 chan)
{
	struct pca9541 *data = i2c_mux_priv(muxc);
	struct i2c_client *client = data->client;

	pca9541_release_bus(client);
	return 0;
}

static int parse_mask_val(const char *buf, u32 *mask, u32 *val)
{
        char *input, *p, *token;
        int ret = 0;

        if (!buf || !mask || !val)
                return -EINVAL;

        input = kstrdup(buf, GFP_KERNEL);  // 内核空间需要自己复制字符串
        if (!input)
                return -ENOMEM;

        p = input;

        // 解析 mask
        token = strsep(&p, "/");
        if (!token) {
                ret = -EINVAL;
                goto out;
        }
        ret = kstrtou32(token, 0, mask);  // base=0: 自动识别0x、0前缀
        if (ret)
                goto out;

        // 解析 val
        token = strsep(&p, "/");
        if (!token) {
                ret = -EINVAL;
                goto out;
        }
        ret = kstrtou32(token, 0, val);
        if (ret)
                goto out;

out:
        kfree(input);
        return ret;
}
/*
 * Arbitration management functions
 */
static void pca9641_release_bus(struct i2c_client *client)
{
       pca9541_reg_write(client, PCA9641_CONTROL, 0);
}

/*
 * Channel arbitration
 *
 * Return values:
 *  <0: error
 *  0 : bus not acquired
 *  1 : bus acquired
 */
static int pca9641_arbitrate(struct i2c_client *client)
{
       struct i2c_mux_core *muxc = i2c_get_clientdata(client);
       struct pca9541 *data = i2c_mux_priv(muxc);
       int reg_ctl, reg_sts;

       reg_ctl = pca9541_reg_read(client, PCA9641_CONTROL);
       if (reg_ctl < 0) {
               printk("[Eli test] error %d\n",reg_ctl);
               return reg_ctl;
       }
       reg_sts = pca9541_reg_read(client, PCA9641_STATUS);

       if (BUSOFF(reg_ctl, reg_sts)) {
               /*
                * Bus is off. Request ownership or turn it on unless
                * other master requested ownership.
                */
               reg_ctl |= PCA9641_CTL_LOCK_REQ;
               pca9541_reg_write(client, PCA9641_CONTROL, reg_ctl);
               reg_ctl = pca9541_reg_read(client, PCA9641_CONTROL);

               if (lock_grant(reg_ctl)) {
                       /*
                        * Other master did not request ownership,
                        * or arbitration timeout expired. Take the bus.
                        */
                       reg_ctl |= PCA9641_CTL_BUS_CONNECT
                               | PCA9641_CTL_LOCK_REQ;
                       pca9541_reg_write(client, PCA9641_CONTROL, reg_ctl);
                       data->select_timeout = SELECT_DELAY_SHORT;

                       return 1;
               } else {
                       /*
                        * Other master requested ownership.
                        * Set extra long timeout to give it time to acquire it.
                        */
                       data->select_timeout = SELECT_DELAY_LONG * 2;
               }
       } else if (lock_grant(reg_ctl)) {
               /*
                * Bus is on, and we own it. We are done with acquisition.
                */
               reg_ctl |= PCA9641_CTL_BUS_CONNECT | PCA9641_CTL_LOCK_REQ;
               pca9541_reg_write(client, PCA9641_CONTROL, reg_ctl);

               return 1;
       } else if (other_lock(reg_sts)) {
               /*
                * Other master owns the bus.
                * If arbitration timeout has expired, force ownership.
                * Otherwise request it.
                */
               data->select_timeout = SELECT_DELAY_LONG;
               reg_ctl |= PCA9641_CTL_LOCK_REQ;
               pca9541_reg_write(client, PCA9641_CONTROL, reg_ctl);
       }
       return 0;
}

static int pca9641_select_chan(struct i2c_mux_core *muxc, u32 chan)
{
       struct pca9541 *data = i2c_mux_priv(muxc);
       struct i2c_client *client = data->client;
       int ret;
       unsigned long timeout = jiffies + ARB2_TIMEOUT;
       /* give up after this time */
       data->arb_timeout = jiffies + ARB_TIMEOUT;
       /* force bus ownership after this time */

       do {
               ret = pca9641_arbitrate(client);
               if (ret) {
                       return ret < 0 ? ret : 0;
               }

               if (data->select_timeout == SELECT_DELAY_SHORT) {
                       udelay(data->select_timeout);
               }
               else {
                       msleep(data->select_timeout / 1000);
               }

       } while (time_is_after_eq_jiffies(timeout));
       dev_dbg(&client->dev, "Timeout: %s\n", client->name);

       return -ETIMEDOUT;
}

static int pca9641_release_chan(struct i2c_mux_core *muxc, u32 chan)
{
       struct pca9541 *data = i2c_mux_priv(muxc);
       struct i2c_client *client = data->client;

       pca9641_release_bus(client);
       return 0;
}

static int pca9641_detect_id(struct i2c_client *client)
{
       int reg;

       reg = pca9541_reg_read(client, PCA9641_ID);
       if (reg == PCA9641_ID_MAGIC) {
               return 1;
       }
       else {
               return 0;
       }
}

static ssize_t int_desc_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	int ret = 0;
	ret = scnprintf(buf, PAGE_SIZE, "using interrupts guide\n");
	ret += scnprintf(buf + ret, PAGE_SIZE, "cat [int_enable/int_state]:display enable/state of interrupts and clear all of interrupts\n");	
	ret += scnprintf(buf + ret, PAGE_SIZE, "echo mask/val > [int_enable/int_state]:mask = 0xF0, val = 0xA0->updates only the upper 4 bits to A\n");				
	ret += scnprintf(buf + ret, PAGE_SIZE, "interrupt bitmap description\n");
	ret += scnprintf(buf + ret, PAGE_SIZE, "bit7:reserved\n");
	ret += scnprintf(buf + ret, PAGE_SIZE, "bit6:bus hung, 0 disable\n");
	ret += scnprintf(buf + ret, PAGE_SIZE, "bit5:mbox is full, 0 disable\n");
	ret += scnprintf(buf + ret, PAGE_SIZE, "bit4:other mbox is empty, 0 disable\n");
	ret += scnprintf(buf + ret, PAGE_SIZE, "bit3:sent an interrupt to itself, 0 disable\n");
	ret += scnprintf(buf + ret, PAGE_SIZE, "bit2:current master get bus, 0 disable\n");		
	ret += scnprintf(buf + ret, PAGE_SIZE, "bit1:this master has involuntarily lost bus, 0 disable\n");		
	ret += scnprintf(buf + ret, PAGE_SIZE, "bit0:io input from downstream, 0 disable\n");	
		
	return ret;
}
DEVICE_ATTR_RO(int_desc);
static ssize_t other_mbox_empty_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	int reg, ret = 0;

	reg = pca9541_reg_read(client, PCA9641_STATUS);

	if (reg < 0) {
		pr_err("cannot read pca9641 reg of status\n");
		ret = scnprintf(buf, PAGE_SIZE, "err:%ld\n", reg);
	} else {
		ret = scnprintf(buf, PAGE_SIZE, "%ld\n", (reg & PCA9641_STS_MBOX_EMPTY)?1:0);
	}
	return ret;
}
DEVICE_ATTR_RO(other_mbox_empty);

static ssize_t mbox_full_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	int reg, ret = 0;

	reg = pca9541_reg_read(client, PCA9641_STATUS);

	if (reg < 0) {
		pr_err("cannot read pca9641 reg of status\n");
		ret = scnprintf(buf, PAGE_SIZE, "err:%ld\n", reg);
	} else {
		ret = scnprintf(buf, PAGE_SIZE, "%ld\n", (reg & PCA9641_STS_MBOX_FULL)?1:0);
	}
	return ret;
}
DEVICE_ATTR_RO(mbox_full);

static ssize_t reg_dump_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	int i, reg, n = 0;

	for (i = 0; i < PCA9641_REG_DUMP_NUM; i++) {
		reg = pca9541_reg_read(client, i);
		if (reg < 0) {
			pr_err("cannot read pca9641 reg: %.2x\n", i);
			n += scnprintf(buf + n, PAGE_SIZE, "0x%.2x:err\n", i);
		} else {
			n += scnprintf(buf + n, PAGE_SIZE, "0x%.2x:%.2x\n", i, reg);
		}		
	}
	return n;
}
DEVICE_ATTR_RO(reg_dump);

static ssize_t other_lock_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	int reg, ret = 0;

	reg = pca9541_reg_read(client, PCA9641_STATUS);

	if (reg < 0) {
		pr_err("cannot read pca9641 reg of status\n");
		ret = scnprintf(buf, PAGE_SIZE, "err:%d\n", reg);
	} else {
		ret = scnprintf(buf, PAGE_SIZE, "%d\n", other_lock(reg));
	}
	return ret;
}
DEVICE_ATTR_RO(other_lock);

static ssize_t bus_release_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	int reg, ret = 0;

	reg = pca9541_reg_read(client, PCA9641_CONTROL);

	if (reg < 0) {
		pr_err("cannot read pca9641 reg of control\n");
		ret = scnprintf(buf, PAGE_SIZE, "err:%d\n", reg);
	} else {
		ret = scnprintf(buf, PAGE_SIZE, "%d\n", (lock_grant(reg))?0:1);
	}
	return ret;
}

static ssize_t bus_release_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	int unlock_bus = 0;

	if (!kstrtoint(buff, 0, &unlock_bus) && unlock_bus) 
		pca9641_release_bus(client);
	return size;
}
DEVICE_ATTR_RW(bus_release);

static ssize_t bus_idle_release_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	int reg, ret = 0;

	reg = pca9541_reg_read(client, PCA9641_CONTROL);

	if (reg < 0) {
		pr_err("cannot read pca9641 reg of control\n");
		ret = scnprintf(buf, PAGE_SIZE, "err:%d\n", reg);
	} else {
		ret = scnprintf(buf, PAGE_SIZE, "%d\n", (reg & PCA9641_CTL_IDLE_TIMER_DIS)?1:0);
	}
	return ret;
}

static ssize_t bus_idle_release_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	int release = 0;
	int reg = pca9541_reg_read(client, PCA9641_CONTROL);
	if (!kstrtoint(buff, 0, &release)) {
		if (release) 
			pca9541_reg_write(client, PCA9641_CONTROL, reg | PCA9641_CTL_IDLE_TIMER_DIS);
		else
			pca9541_reg_write(client, PCA9641_CONTROL, reg & (~PCA9641_CTL_IDLE_TIMER_DIS));
		
	}
		
	return size;
}
DEVICE_ATTR_RW(bus_idle_release);

// static ssize_t soft_reset_show(struct device *dev,
// 				struct device_attribute *attr,
// 				char *buf)
// {
// 	struct i2c_client *client = to_i2c_client(dev);

// 	int reg, ret = 0;

// 	reg = pca9541_reg_read(client, PCA9641_CONTROL);

// 	if (reg < 0) {
// 		pr_err("cannot read pca9641 reg of control\n");
// 		ret = scnprintf(buf, PAGE_SIZE, "err:%d\n", reg);
// 	} else {
// 		ret = scnprintf(buf, PAGE_SIZE, "%d\n", (reg & PCA9641_CTL_SMBUS_SWRST)?1:0);
// 	}
// 	return ret;
// }

static ssize_t soft_reset_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 reset_cmd = 0x06;
	int reset = 0;
	int ret;
	if (!kstrtoint(buff, 0, &reset) && reset) {
		
		struct i2c_msg msg = {
			.addr  = 0x00,       // General Call address
			.flags = 0,          // Write
			.len   = 1,
			.buf   = &reset_cmd,
		};
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret < 0)
			pr_err("pca 9641 I2C Software Reset failed: %d\n", ret);
	}
		
	return size;
}
DEVICE_ATTR_WO(soft_reset);

static ssize_t bus_lock_ms_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	int reg, ret = 0;

	reg = pca9541_reg_read(client, PCA9641_TIME);

	if (reg < 0) {
		pr_err("cannot read pca9641 reg of reserve time\n");
		ret = scnprintf(buf, PAGE_SIZE, "err:%d\n", reg);
	} else {
		ret = scnprintf(buf, PAGE_SIZE, "%d\n", reg);
	}
	return ret;
}

static ssize_t bus_lock_ms_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ms = 0;
	if (!kstrtoint(buff, 0, &ms)) {
		pca9541_reg_write(client, PCA9641_TIME, ms);
	}
	return size;
}
DEVICE_ATTR_RW(bus_lock_ms);

static ssize_t mbox_msg_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	int ret = -1;	

	struct i2c_adapter *adap = client->adapter;
	union i2c_smbus_data data = { .byte = 0 };
	u16 val;

	ret = __i2c_smbus_xfer(adap, client->addr, client->flags,
			       I2C_SMBUS_READ, PCA9641_MBOX_HI,
			       I2C_SMBUS_WORD_DATA, &data);

	if (ret < 0) {
		pr_err("cannot read pca9641 reg of mbox lo\n");
		ret = scnprintf(buf, PAGE_SIZE, "err:%d\n", ret);
	} else {
		val = data.byte;
		val <<= 8;
		ret = __i2c_smbus_xfer(adap, client->addr, client->flags,
					I2C_SMBUS_READ, PCA9641_MBOX_LO,
					I2C_SMBUS_WORD_DATA, &data);		
		if (ret < 0) {
			pr_err("cannot read pca9641 reg of mbox hi\n");
			ret = scnprintf(buf, PAGE_SIZE, "err:%d\n", ret);
		} else {         
			val |= data.byte; 
			ret = scnprintf(buf, PAGE_SIZE, "%d\n", val);
		}
	}
	return ret;
}

static ssize_t mbox_msg_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	u16 mbox = 0;
	struct i2c_adapter *adap = client->adapter;
	union i2c_smbus_data data = {.word = 0};

	if (!kstrtou16(buff, 0, &mbox)) {
		data.word = mbox;
		__i2c_smbus_xfer(adap, client->addr, client->flags,
						I2C_SMBUS_WRITE, PCA9641_MBOX_LO | 0x80,
						I2C_SMBUS_WORD_DATA, &data);						
	}
	return size;
}
DEVICE_ATTR_RW(mbox_msg);

static ssize_t int_enable_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	int reg, ret = 0;

	reg = pca9541_reg_read(client, PCA9641_INT_MASK);

	if (reg < 0) {
		pr_err("cannot read pca9641 reg of int enable\n");
		ret = scnprintf(buf, PAGE_SIZE, "err:%d\n", reg);
	} else {
		u8 mask = ~reg;
		ret = scnprintf(buf, PAGE_SIZE, BYTE_TO_BINARY_PATTERN, mask, BYTE_TO_BINARY(mask));				
	}
	return ret;
}

static ssize_t int_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret;
	u32 mask, val;
	char *kbuf;

	// 复制用户输入为以 \0 结尾的字符串
	kbuf = kmemdup_nul(buff, size, GFP_KERNEL);
	if (!kbuf) {
		pr_err("no mem in pca9641 int enable\n");
		return -ENOMEM;
	}

	strim(kbuf);  // 去除前后空白字符

	ret = parse_mask_val(kbuf, &mask, &val);  // 你前面定义的函数
	kfree(kbuf);
	val = ~val;
	if (ret) {
		pr_err("Invalid mask/val format in pca9641 int enable\n");
		return -EINVAL;
	}
	// pr_info("Got mask=0x%x, val=0x%x in pca9641 int enable\n", mask, val);
	ret = pca9541_reg_read(client, PCA9641_INT_MASK);
	if (ret < 0)
		pr_err("cannot read pca9641 reg of int mask %d\n", ret);
	else 
		pca9541_reg_write(client, PCA9641_INT_MASK, (mask & val)|(ret & ~mask));

	return size;
}
DEVICE_ATTR_RW(int_enable);

static ssize_t int_state_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	int reg, ret = 0;

	reg = pca9541_reg_read(client, PCA9641_INT_STATUS);

	if (reg < 0) {
		pr_err("cannot read pca9641 reg of int status\n");
		ret = scnprintf(buf, PAGE_SIZE, "err:%d\n", reg);
	} else {

		ret = scnprintf(buf, PAGE_SIZE, BYTE_TO_BINARY_PATTERN, reg, BYTE_TO_BINARY(reg));		
		pca9541_reg_write(client, PCA9641_INT_STATUS, 0x7f);	
	}
	return ret;
}

static ssize_t int_state_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	int reg, ret;
	u32 mask, val;
	char *kbuf;

	// 复制用户输入为以 \0 结尾的字符串
	kbuf = kmemdup_nul(buff, size, GFP_KERNEL);
	if (!kbuf) {
		pr_err("no mem in pca9641 int state\n");
		return -ENOMEM;
	}

	strim(kbuf);  // 去除前后空白字符

	ret = parse_mask_val(kbuf, &mask, &val);  // 你前面定义的函数
	val = ~val;
	kfree(kbuf);
	if (ret) {
		pr_err("Invalid mask/val format in pca9641 int state\n");
		return -EINVAL;
	}
	// pr_info("Got mask=0x%x, val=0x%x in pca9641 int state\n", mask, val);
	ret = pca9541_reg_read(client, PCA9641_INT_STATUS);
	if (ret < 0)
		pr_err("cannot read pca9641 reg of int state %d\n", ret);
	else 
		pca9541_reg_write(client, PCA9641_INT_STATUS, (mask & val)|(ret & ~mask));


	return size;
}
DEVICE_ATTR_RW(int_state);


static ssize_t ctrl_cmd_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	int reg, ret = 0;

	reg = pca9541_reg_read(client, PCA9641_CONTROL);

	if (reg < 0) {
		pr_err("cannot read pca9641 reg of ctrl\n");
		ret = scnprintf(buf, PAGE_SIZE, "err:%d\n", reg);
	} else {
		ret = scnprintf(buf, PAGE_SIZE, BYTE_TO_BINARY_PATTERN, reg, BYTE_TO_BINARY(reg));
	}
	return ret;
}

static ssize_t ctrl_cmd_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	int reg, cmd = 0;

	if (!kstrtoint(buff, 0, &cmd)) {
		reg = pca9541_reg_read(client, PCA9641_CONTROL);
		pca9541_reg_write(client, PCA9641_CONTROL, cmd);
	}
	return size;
}
DEVICE_ATTR_RW(ctrl_cmd);
/*
 * I2C init/probing/exit functions
 */
static int pca9541_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct i2c_adapter *adap = client->adapter;
	struct i2c_mux_core *muxc;
	struct pca9541 *data;
	int ret;
	int detect_id;
	struct attribute_group *attr_group;
	struct attribute **attrs;
	if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	detect_id = pca9641_detect_id(client);
	/*
	 * I2C accesses are unprotected here.
	 * We have to lock the I2C segment before releasing the bus.
	 */
	if (detect_id == 0) {
	        i2c_lock_bus(adap, I2C_LOCK_SEGMENT);
	        pca9541_release_bus(client);
	        i2c_unlock_bus(adap, I2C_LOCK_SEGMENT);
	} else {
	        i2c_lock_bus(adap, I2C_LOCK_SEGMENT);
	        pca9641_release_bus(client);
	        i2c_unlock_bus(adap, I2C_LOCK_SEGMENT);
			struct attribute *all_attrs[] = {
				&dev_attr_other_mbox_empty.attr,
				&dev_attr_mbox_full.attr,
				&dev_attr_other_lock.attr,
				&dev_attr_reg_dump.attr,
				&dev_attr_bus_release.attr,
				&dev_attr_bus_idle_release.attr,	
				&dev_attr_bus_lock_ms.attr,
				&dev_attr_soft_reset.attr,
				&dev_attr_mbox_msg.attr,	
				&dev_attr_int_desc.attr,
				// &dev_attr_clear_int_getbus.attr,				
				// &dev_attr_enable_int_new_msg.attr,
				// &dev_attr_clear_int_new_msg.attr,			
				// &dev_attr_enable_int_receipt.attr,	
				// &dev_attr_clear_int_receipt.attr,
				&dev_attr_int_state.attr,
				&dev_attr_int_enable.attr,
				&dev_attr_ctrl_cmd.attr,
				NULL
			};
			const int attr_count = ARRAY_SIZE(all_attrs);
			int i;

			attr_group = devm_kzalloc(&client->dev, sizeof(*attr_group), GFP_KERNEL);
			if (!attr_group)
				return -ENOMEM;

			attrs = devm_kzalloc(&client->dev, sizeof(struct attribute *) * (attr_count + 1), GFP_KERNEL);
			if (!attrs)
					return -ENOMEM;
			for (i = 0; i < attr_count; i++)
				attrs[i] = all_attrs[i];

			attr_group->attrs = attrs;
			dev_set_name(&client->dev, "pca9641-%d-%.2x", client->adapter->nr, client->addr);
			attr_group->name = client->dev.kobj.name;
			ret = sysfs_create_group(&client->dev.kobj, attr_group);
			if (ret) {
				dev_err(&client->dev, "could not create sysfs group\n");
				return ret;
			}			

	}

	/* Create mux adapter */

	if (detect_id == 0) {
	        muxc = i2c_mux_alloc(adap, &client->dev, 1, sizeof(*data),
	                             I2C_MUX_ARBITRATOR,
	                             pca9541_select_chan, pca9541_release_chan);
	} else {
	        muxc = i2c_mux_alloc(adap, &client->dev, 1, sizeof(*data),
	                             I2C_MUX_ARBITRATOR,
	                             pca9641_select_chan, pca9641_release_chan);
	}
	if (!muxc)
		return -ENOMEM;

	data = i2c_mux_priv(muxc);
	data->client = client;
	if (detect_id) {
		data->attr_group = attr_group;
		data->attrs = attrs;
	}

	i2c_set_clientdata(client, muxc);

	ret = i2c_mux_add_adapter(muxc, 0, 0, 0);
	if (ret)
		return ret;

	dev_info(&client->dev, "registered master selector for I2C %s\n",
		 client->name);

	return 0;
}

static int pca9541_remove(struct i2c_client *client)
{
	struct i2c_mux_core *muxc = i2c_get_clientdata(client);
	struct pca9541 *data = i2c_mux_priv(muxc);

	if (data && data->attr_group)
		sysfs_remove_group(&client->dev.kobj, data->attr_group);
	i2c_mux_del_adapters(muxc);
	return 0;
}

static struct i2c_driver pca9541_driver = {
	.driver = {
		   .name = "pca9541",
		   .of_match_table = of_match_ptr(pca9541_of_match),
		   },
	.probe = pca9541_probe,
	.remove = pca9541_remove,
	.id_table = pca9541_id,
};

module_i2c_driver(pca9541_driver);

MODULE_AUTHOR("bowen <zengbo@westwell-lab.com>");
MODULE_DESCRIPTION("PCA9541/PCA9641 I2C master selector driver");
MODULE_LICENSE("GPL v2");
