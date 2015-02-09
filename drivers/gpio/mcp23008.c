/*
 *  mcp23008.c - 8 bit I/O ports
 *
 *  Copyright (C) 2015 Advansee <support@advansee.com>
 *
 *  Derived from drivers/i2c/chips/pca953x.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/i2c/mcp23008.h>
#include <linux/slab.h>
#ifdef CONFIG_OF_GPIO
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#endif

#define MCP_IODIR	0x00		/* init/reset:  all ones */
#define MCP_IPOL	0x01
#define MCP_GPINTEN	0x02
#define MCP_DEFVAL	0x03
#define MCP_INTCON	0x04
#define MCP_IOCON	0x05
#	define IOCON_MIRROR	(1 << 6)
#	define IOCON_SEQOP	(1 << 5)
#	define IOCON_HAEN	(1 << 3)
#	define IOCON_ODR	(1 << 2)
#	define IOCON_INTPOL	(1 << 1)
#define MCP_GPPU	0x06
#define MCP_INTF	0x07
#define MCP_INTCAP	0x08
#define MCP_GPIO	0x09
#define MCP_OLAT	0x0a


#define MCP_TYPE_008	8

static const struct i2c_device_id mcp23008_id[] = {
	{ "mcp23008", MCP_TYPE_008, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mcp23008_id);

struct mcp23008_chip {
	unsigned gpio_start;
	uint16_t reg_output;
	uint16_t reg_direction;
	struct mutex i2c_lock;

#ifdef CONFIG_GPIO_MCP28003_IRQ
	struct mutex irq_lock;
	uint16_t irq_mask;
	uint16_t irq_stat;
	uint16_t irq_trig_raise;
	uint16_t irq_trig_fall;
	int	 irq_base;
#endif

	struct i2c_client *client;
	struct mcp23008_platform_data *dyn_pdata;
	struct gpio_chip gpio_chip;
	const char *const *names;
	int	chip_type;
};

static int mcp23008_write_reg(struct mcp23008_chip *chip, int reg, uint16_t val)
{
	int ret = 0;
	
	ret = i2c_smbus_write_byte_data(chip->client, reg, val);

	if (ret < 0) {
		printk(KERN_ERR "failed writing register\n");
		return ret;
	}

	return 0;
}

static int mcp23008_read_reg(struct mcp23008_chip *chip, int reg, uint16_t *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		printk(KERN_ERR "failed reading register\n");	
		return ret;
	}
	
	*val = (uint16_t)ret;

	return 0;
}

static int mcp23008_gpio_direction_input(struct gpio_chip *gc, unsigned off)
{
	struct mcp23008_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct mcp23008_chip, gpio_chip);

	mutex_lock(&chip->i2c_lock);
	reg_val = chip->reg_direction | (1u << off);

	ret = mcp23008_write_reg(chip, MCP_IODIR, reg_val);
	if (ret)
		goto exit;

	chip->reg_direction = reg_val;
	ret = 0;
exit:
	mutex_unlock(&chip->i2c_lock);
	return ret;
}

static int mcp23008_gpio_direction_output(struct gpio_chip *gc,
		unsigned off, int val)
{
	struct mcp23008_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct mcp23008_chip, gpio_chip);

	mutex_lock(&chip->i2c_lock);
	/* set output level */
	if (val)
		reg_val = chip->reg_output | (1u << off);
	else
		reg_val = chip->reg_output & ~(1u << off);

	ret = mcp23008_write_reg(chip, MCP_GPIO, reg_val);
	if (ret)
		goto exit;

	chip->reg_output = reg_val;

	/* then direction */
	reg_val = chip->reg_direction & ~(1u << off);
	ret = mcp23008_write_reg(chip, MCP_IODIR, reg_val);
	if (ret)
		goto exit;

	chip->reg_direction = reg_val;
	ret = 0;
exit:
	mutex_unlock(&chip->i2c_lock);
	return ret;
}

static int mcp23008_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
	struct mcp23008_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct mcp23008_chip, gpio_chip);

	mutex_lock(&chip->i2c_lock);
	ret = mcp23008_read_reg(chip, MCP_IPOL, &reg_val);
	mutex_unlock(&chip->i2c_lock);
	if (ret < 0) {
		/* NOTE:  diagnostic already emitted; that's all we should
		 * do unless gpio_*_value_cansleep() calls become different
		 * from their nonsleeping siblings (and report faults).
		 */
		return 0;
	}

	return (reg_val & (1u << off)) ? 1 : 0;
}

static void mcp23008_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
	struct mcp23008_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct mcp23008_chip, gpio_chip);

	mutex_lock(&chip->i2c_lock);
	if (val)
		reg_val = chip->reg_output | (1u << off);
	else
		reg_val = chip->reg_output & ~(1u << off);

	ret = mcp23008_write_reg(chip, MCP_GPIO, reg_val);
	if (ret)
		goto exit;

	chip->reg_output = reg_val;
exit:
	mutex_unlock(&chip->i2c_lock);
}

static void mcp23008_setup_gpio(struct mcp23008_chip *chip, int gpios)
{
	struct gpio_chip *gc;

	gc = &chip->gpio_chip;

	gc->direction_input  = mcp23008_gpio_direction_input;
	gc->direction_output = mcp23008_gpio_direction_output;
	gc->get = mcp23008_gpio_get_value;
	gc->set = mcp23008_gpio_set_value;
	gc->can_sleep = 1;

	gc->base = chip->gpio_start;
	gc->ngpio = gpios;
	gc->label = chip->client->name;
	gc->dev = &chip->client->dev;
	gc->owner = THIS_MODULE;
	gc->names = chip->names;
}

/*
 * Handlers for alternative sources of platform_data
 */
#ifdef CONFIG_OF_GPIO
/*
 * Translate OpenFirmware node properties into platform_data
 */
static struct mcp23008_platform_data *
mcp23008_get_alt_pdata(struct i2c_client *client)
{
	struct mcp23008_platform_data *pdata;
	struct device_node *node;
	const __be32 *val;
	int size;

	node = client->dev.of_node;
	if (node == NULL)
		return NULL;

	pdata = kzalloc(sizeof(struct mcp23008_platform_data), GFP_KERNEL);
	if (pdata == NULL) {
		dev_err(&client->dev, "Unable to allocate platform_data\n");
		return NULL;
	}

	pdata->gpio_base = -1;
	val = of_get_property(node, "linux,gpio-base", &size);
	if (val) {
		if (size != sizeof(*val))
			dev_warn(&client->dev, "%s: wrong linux,gpio-base\n",
				 node->full_name);
		else
			pdata->gpio_base = be32_to_cpup(val);
	}

	val = of_get_property(node, "polarity", NULL);
	if (val)
		pdata->invert = *val;

	return pdata;
}
#else
static struct mcp23008_platform_data *
mcp23008_get_alt_pdata(struct i2c_client *client)
{
	return NULL;
}
#endif

static int __devinit device_mcp23008_init(struct mcp23008_chip *chip, int invert)
{
	int ret;

	ret = mcp23008_read_reg(chip, MCP_IPOL, &chip->reg_output);
	if (ret)
		goto out;

	ret = mcp23008_read_reg(chip, MCP_IODIR, &chip->reg_direction);
	if (ret)
		goto out;

	/* set platform specific polarity inversion */
	ret = mcp23008_write_reg(chip, MCP_IOCON, invert);
	if (ret)
		goto out;
	return 0;
out:
	return ret;
}


static int __devinit mcp23008_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct mcp23008_platform_data *pdata;
	struct mcp23008_chip *chip;
	int ret = 0;
	
	chip = kzalloc(sizeof(struct mcp23008_chip), GFP_KERNEL);
	if (chip == NULL)
	{
	    printk(KERN_ERR "mcp23008 probe error kzalloc\n");
	    return -ENOMEM;
	}
	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		printk(KERN_ERR "mcp23008 probe if (pdata == NULL)\n");
		pdata = mcp23008_get_alt_pdata(client);
		/*
		 * Unlike normal platform_data, this is allocated
		 * dynamically and must be freed in the driver
		 */
		chip->dyn_pdata = pdata;
	}

	if (pdata == NULL) {
		printk(KERN_ERR "mcp23008 probe no platform data\n");
		ret = -EINVAL;
		goto out_failed;
	}

	chip->client = client;

	chip->gpio_start = pdata->gpio_base;
	printk(KERN_ERR "mcp23008 probe base %d\n",chip->gpio_start);

	chip->names = pdata->names;
	chip->chip_type = id->driver_data;
	
	printk(KERN_ERR "mcp23008 probe chip_type %d\n",chip->chip_type);
	

	mutex_init(&chip->i2c_lock);

	/* initialize cached registers from their original values.
	 * we can't share this chip with another i2c master.
	 */
	mcp23008_setup_gpio(chip, id->driver_data);

	if (chip->chip_type == MCP_TYPE_008)
		device_mcp23008_init(chip, pdata->invert);
	else
		goto out_failed;

	ret = gpiochip_add(&chip->gpio_chip);
	if (ret)
	  goto out_failed;
	
	if (pdata->setup) {
		ret = pdata->setup(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0)
		  printk(KERN_ERR "mcp23008 probe setup failed, %d\n", ret);
	}

	i2c_set_clientdata(client, chip);
	printk(KERN_ERR "mcp23008 probe base %d\n",chip->gpio_start);
	return 0;

out_failed:
	kfree(chip->dyn_pdata);
	kfree(chip);
	printk(KERN_ERR "mcp23008 probe out failed\n");
	return ret;
}

static int mcp23008_remove(struct i2c_client *client)
{
	struct mcp23008_platform_data *pdata = client->dev.platform_data;
	struct mcp23008_chip *chip = i2c_get_clientdata(client);
	int ret = 0;

	if (pdata->teardown) {
		ret = pdata->teardown(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0) {
			dev_err(&client->dev, "%s failed, %d\n",
					"teardown", ret);
			return ret;
		}
	}

	ret = gpiochip_remove(&chip->gpio_chip);
	if (ret) {
		dev_err(&client->dev, "%s failed, %d\n",
				"gpiochip_remove()", ret);
		return ret;
	}

	kfree(chip->dyn_pdata);
	kfree(chip);
	return 0;
}

static struct i2c_driver mcp23008_driver = {
	.driver = {
		.name	= "mcp23008",
	},
	.probe		= mcp23008_probe,
	.remove		= mcp23008_remove,
	.id_table	= mcp23008_id,
};

static int __init mcp23008_init(void)
{
	return i2c_add_driver(&mcp23008_driver);
}
/* register after i2c postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(mcp23008_init);

static void __exit mcp23008_exit(void)
{
	i2c_del_driver(&mcp23008_driver);
}
module_exit(mcp23008_exit);

MODULE_AUTHOR("Advansee <support@advansee.com>");
MODULE_DESCRIPTION("GPIO expander driver for MCP23008");
MODULE_LICENSE("GPL");
