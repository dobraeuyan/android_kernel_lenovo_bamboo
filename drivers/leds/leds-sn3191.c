/*
 * Driver for IS31FL3191 (SN3191) Breath LED
 * Optimized for Lenovo Yoga Tab 3 (Bamboo) / LineageOS 13
 * * Fixes: "One Shot" triggering sequence and Low Battery blink.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

/* Registers [cite: 241, 244, 251, 260, 275, 254, 280, 286, 299, 291, 294, 302] */
#define SN3191_REG_SHUTDOWN		0x00
#define SN3191_REG_BREATH_CTRL		0x01
#define SN3191_REG_LED_MODE		0x02
#define SN3191_REG_CURRENT		0x03
#define SN3191_REG_PWM			0x04
#define SN3191_REG_PWM_UPDATE		0x07
#define SN3191_REG_T0			0x0A
#define SN3191_REG_T1_T2		0x10
#define SN3191_REG_T3_T4		0x16
#define SN3191_REG_TIME_UPDATE		0x1C
#define SN3191_REG_LED_CTRL		0x1D
#define SN3191_REG_RESET		0x2F

/* Configuration Values [cite: 262, 278, 245] */
#define SN3191_MODE_PWM			0x00
#define SN3191_MODE_BREATH		0x01 /* One Shot Programming Mode */
#define SN3191_CURRENT_5MA		0x02 /* 010b = 5mA */
#define SN3191_CMD_ENABLE		0x20 /* EN=1, SSD=0 (Normal) */
#define SN3191_CMD_SHUTDOWN		0x01 /* SSD=1 (Software Shutdown) */

struct sn3191_led_data {
	struct i2c_client *client;
	struct led_classdev cdev;
	struct mutex lock;
	struct regulator *vdd;
	int sdb_gpio;
};

/* Time lookup tables (ms) based on Datasheet Tables 9 & 10 [cite: 290, 305] */
static const int t_rise_fall_ms[] = { 130, 260, 520, 1040, 2080, 4160, 8320, 16640 };
static const int t_hold_ms[]      = { 0, 130, 260, 520, 1040, 2080, 4160, 8320, 16640 };
static const int t_off_ms[]       = { 0, 130, 260, 520, 1040, 2080, 4160, 8320, 16640, 33280, 66560 };

/* Find index that provides a time >= requested time (Ceiling) */
static int find_nearest_index(const int *table, int size, int ms)
{
	int i;
	for (i = 0; i < size - 1; i++) {
		if (ms <= table[i])
			return i;
	}
	return size - 1;
}

static int sn3191_write_reg(struct sn3191_led_data *led, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(led->client, reg, val);
}

static void sn3191_set_brightness(struct led_classdev *cdev,
				  enum led_brightness brightness)
{
	struct sn3191_led_data *led = container_of(cdev, struct sn3191_led_data, cdev);

	mutex_lock(&led->lock);

	if (brightness == LED_OFF) {
		/* Software Shutdown [cite: 245] */
		sn3191_write_reg(led, SN3191_REG_SHUTDOWN, SN3191_CMD_SHUTDOWN);
	} else {
		/* Sequence for PWM Mode [cite: 314] */
		sn3191_write_reg(led, SN3191_REG_SHUTDOWN, SN3191_CMD_ENABLE);
		sn3191_write_reg(led, SN3191_REG_LED_CTRL, 0x01); /* Enable OUT */
		sn3191_write_reg(led, SN3191_REG_LED_MODE, SN3191_MODE_PWM); /* Set PWM Mode */
		sn3191_write_reg(led, SN3191_REG_PWM, brightness); /* Set Duty Cycle */
		sn3191_write_reg(led, SN3191_REG_PWM_UPDATE, 0x00); /* Load Value */
	}

	mutex_unlock(&led->lock);
}

static int sn3191_blink_set(struct led_classdev *cdev,
			    unsigned long *delay_on,
			    unsigned long *delay_off)
{
	struct sn3191_led_data *led = container_of(cdev, struct sn3191_led_data, cdev);
	int t1_idx, t2_idx, t3_idx, t4_idx;
	u8 t1_t2_val, t3_t4_val;

	/* Default behavior for Android [cite: 121] */
	if (!*delay_on && !*delay_off) {
		*delay_on = 1000;
		*delay_off = 1000;
	}

	mutex_lock(&led->lock);

	/* 1. Reset Mode to PWM first to STOP any running breath cycle */
	sn3191_write_reg(led, SN3191_REG_LED_MODE, SN3191_MODE_PWM);

	/* * Logic for Breathing[cite: 321]:
	 * T4 (Off) = delay_off
	 * T1 (Rise) + T2 (Hold) + T3 (Fall) = delay_on
	 * We split delay_on: 40% Rise, 20% Hold, 40% Fall for smooth look.
	 */
	t4_idx = find_nearest_index(t_off_ms, ARRAY_SIZE(t_off_ms), *delay_off);
	t1_idx = find_nearest_index(t_rise_fall_ms, ARRAY_SIZE(t_rise_fall_ms), (*delay_on * 4) / 10);
	t3_idx = t1_idx; /* Symmetric */
	t2_idx = find_nearest_index(t_hold_ms, ARRAY_SIZE(t_hold_ms), (*delay_on * 2) / 10);

	/* Construct Register Values [cite: 288, 300] */
	t1_t2_val = (t1_idx << 5) | (t2_idx << 1); /* T1=D7:D5, T2=D4:D1 */
	t3_t4_val = (t3_idx << 5) | (t4_idx << 1); /* T3=D7:D5, T4=D4:D1 */

	/* 2. Configure Hardware */
	sn3191_write_reg(led, SN3191_REG_SHUTDOWN, SN3191_CMD_ENABLE); /* Wake up */
	sn3191_write_reg(led, SN3191_REG_BREATH_CTRL, 0x00); /* Auto Breath (Default) */
	sn3191_write_reg(led, SN3191_REG_T0, 0x00); /* No start delay [cite: 282] */
	sn3191_write_reg(led, SN3191_REG_T1_T2, t1_t2_val);
	sn3191_write_reg(led, SN3191_REG_T3_T4, t3_t4_val);
	sn3191_write_reg(led, SN3191_REG_TIME_UPDATE, 0x00); /* Update Timers [cite: 293] */
	
	/* 3. Set Current and Enable */
	sn3191_write_reg(led, SN3191_REG_CURRENT, SN3191_CURRENT_5MA); /* 5mA [cite: 278] */
	sn3191_write_reg(led, SN3191_REG_LED_CTRL, 0x01); /* OUT Enable */

	/* 4. Trigger Start (Switch to Breath Mode) [cite: 320] */
	sn3191_write_reg(led, SN3191_REG_LED_MODE, SN3191_MODE_BREATH);

	mutex_unlock(&led->lock);

	return 0;
}

static int sn3191_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	struct sn3191_led_data *led;
	const char *led_name;
	int ret;

	led = devm_kzalloc(&client->dev, sizeof(*led), GFP_KERNEL);
	if (!led) return -ENOMEM;

	led->client = client;
	i2c_set_clientdata(client, led);
	mutex_init(&led->lock);

	/* GPIO Setup */
	led->sdb_gpio = of_get_named_gpio(np, "si-en,sdb-gpio", 0);
	if (gpio_is_valid(led->sdb_gpio)) {
		ret = devm_gpio_request_one(&client->dev, led->sdb_gpio, GPIOF_OUT_INIT_HIGH, "sn3191_sdb");
		if (ret < 0) dev_err(&client->dev, "SDB GPIO request failed\n");
	}

	/* Regulator Setup */
	led->vdd = devm_regulator_get(&client->dev, "vdd");
	if (!IS_ERR(led->vdd)) {
		ret = regulator_enable(led->vdd);
		if (ret) dev_err(&client->dev, "VDD enable failed\n");
	}

	/* Chip Reset & Init [cite: 303, 278] */
	sn3191_write_reg(led, SN3191_REG_RESET, 0x00); 
	sn3191_write_reg(led, SN3191_REG_CURRENT, SN3191_CURRENT_5MA);

	/* Register LED */
	if (of_property_read_string(np, "si-en,led-name", &led_name))
		led->cdev.name = "sn3191-led";
	else
		led->cdev.name = led_name;

	led->cdev.brightness_set = sn3191_set_brightness;
	led->cdev.blink_set = sn3191_blink_set;
	led->cdev.max_brightness = 255;

	ret = led_classdev_register(&client->dev, &led->cdev);
	if (ret < 0) return ret;

	dev_info(&client->dev, "IS31FL3191 (Bamboo) probed.\n");
	return 0;
}

static int sn3191_remove(struct i2c_client *client)
{
	struct sn3191_led_data *led = i2c_get_clientdata(client);
	led_classdev_unregister(&led->cdev);
	sn3191_write_reg(led, SN3191_REG_RESET, 0x00);
	if (gpio_is_valid(led->sdb_gpio)) gpio_set_value(led->sdb_gpio, 0);
	if (!IS_ERR(led->vdd)) regulator_disable(led->vdd);
	return 0;
}

static const struct of_device_id of_sn3191_match[] = {
	{ .compatible = "si-en,sn3191", }, { },
};
MODULE_DEVICE_TABLE(of, of_sn3191_match);

static const struct i2c_device_id sn3191_id[] = {
	{ "sn3191", 0 }, { },
};
MODULE_DEVICE_TABLE(i2c, sn3191_id);

static struct i2c_driver sn3191_driver = {
	.driver = {
		.name = "leds-sn3191",
		.owner = THIS_MODULE,
		.of_match_table = of_sn3191_match,
	},
	.probe = sn3191_probe,
	.remove = sn3191_remove,
	.id_table = sn3191_id,
};
module_i2c_driver(sn3191_driver);
MODULE_AUTHOR("Gemini & Fernando");
MODULE_DESCRIPTION("IS31FL3191 Driver for LineageOS");
MODULE_LICENSE("GPL v2");
