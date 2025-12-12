#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/switch.h>

/* Key codes for hall sensor events */
#define KEY_HALL_OPEN                KEY_WAKEUP
#define KEY_HALL_CLOSE               KEY_SLEEP
#define KEY_HALL_HOLDER_OPEN         0x254
#define KEY_HALL_HOLDER_CLOSE        0x255

#define HALL_COVER_IRQ_NAME  "hall-cover"
#define HALL_CAM_IRQ_NAME    "hall-cam"
#define HALL_HOLDER_IRQ_NAME "hall-holder"

struct hall_switch_irq {
	int gpio;
	int irq;
	int state;
	char irq_name[32];
	struct switch_dev sdev;
};

struct hall_switch_info {
	struct hall_switch_irq cover;
	struct hall_switch_irq cam;
	struct hall_switch_irq holder;
	struct input_dev *ipdev;
};

static irqreturn_t hall_interrupt(int irq, void *data)
{
	struct hall_switch_irq *hall = data;
	struct hall_switch_info *hall_info;
	int new_state;

	new_state = gpio_get_value(hall->gpio);

	pr_info("hall irq interrupt name = %s irq = %d state = %d\n", hall->irq_name, irq, new_state);

	/*
	 * The IRQ is triggered on both edges, so we only process
	 * if the state has actually changed.
	 */
	if (new_state == hall->state)
		return IRQ_HANDLED;

	hall->state = new_state;
	switch_set_state(&hall->sdev, hall->state);

	if (!strcmp(hall->irq_name, HALL_COVER_IRQ_NAME)) {
		hall_info = container_of(hall, struct hall_switch_info, cover);
		if (hall->state) {
			input_report_key(hall_info->ipdev, KEY_HALL_OPEN, 1);
			input_report_key(hall_info->ipdev, KEY_HALL_OPEN, 0);
		} else {
			input_report_key(hall_info->ipdev, KEY_HALL_CLOSE, 1);
			input_report_key(hall_info->ipdev, KEY_HALL_CLOSE, 0);
		}
		input_sync(hall_info->ipdev);
	} else if (!strcmp(hall->irq_name, HALL_HOLDER_IRQ_NAME)) {
		hall_info = container_of(hall, struct hall_switch_info, holder);
		if (hall->state) {
			input_report_key(hall_info->ipdev, KEY_HALL_HOLDER_CLOSE, 1);
			input_report_key(hall_info->ipdev, KEY_HALL_HOLDER_CLOSE, 0);
		} else {
			input_report_key(hall_info->ipdev, KEY_HALL_HOLDER_OPEN, 1);
			input_report_key(hall_info->ipdev, KEY_HALL_HOLDER_OPEN, 0);
		}
		input_sync(hall_info->ipdev);
	} else if (strcmp(hall->irq_name, HALL_CAM_IRQ_NAME)) {
		pr_err("hall irq name not found! name = %s\n", hall->irq_name);
	}

	return IRQ_HANDLED;
}

static int hall_init_irq(struct hall_switch_irq *hall_switch)
{
	int rc;
	struct hall_switch_irq *hall = hall_switch;

	rc = gpio_request(hall->gpio, hall->irq_name);
	if (rc < 0) {
		pr_err("gpio_request fail rc=%d\n", rc);
		return rc;
	}

	rc = gpio_direction_input(hall->gpio);
	if (rc < 0) {
		pr_err("gpio_direction_input fail rc=%d\n", rc);
		goto err_free_gpio;
	}
	hall->state = gpio_get_value(hall->gpio);

	hall->irq = gpio_to_irq(hall->gpio);
	if (hall->irq < 0) {
		rc = hall->irq;
		pr_err("gpio_to_irq fail rc=%d\n", rc);
		goto err_free_gpio;
	}

	irq_set_irq_wake(hall->irq, 1);
	rc = request_threaded_irq(hall->irq, NULL, hall_interrupt,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				hall->irq_name, hall);
	if (rc < 0) {
		pr_err("request_irq fail rc=%d\n", rc);
		goto err_free_gpio;
	}

	hall->sdev.name = hall->irq_name;
	rc = switch_dev_register(&hall->sdev);
	if (rc) {
		pr_err("Failed to setup switch dev for %s\n", hall->irq_name);
		goto err_free_irq;
	}
	switch_set_state(&hall->sdev, hall->state);

	return 0;

err_free_irq:
	free_irq(hall->irq, hall);
err_free_gpio:
	gpio_free(hall->gpio);
	return rc;
}

static int hall_parse_dt(struct device *dev, struct hall_switch_info *pdata)
{
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags = OF_GPIO_ACTIVE_LOW;

	pdata->cover.gpio = of_get_named_gpio_flags(np, "hall,irq-cover", 0, &flags);
	if (gpio_is_valid(pdata->cover.gpio))
		strlcpy(pdata->cover.irq_name, HALL_COVER_IRQ_NAME, sizeof(pdata->cover.irq_name));

	pdata->cam.gpio = of_get_named_gpio_flags(np, "hall,irq-cam", 0, &flags);
	if (gpio_is_valid(pdata->cam.gpio))
		strlcpy(pdata->cam.irq_name, HALL_CAM_IRQ_NAME, sizeof(pdata->cam.irq_name));

	pdata->holder.gpio = of_get_named_gpio_flags(np, "hall,irq-holder", 0, &flags);
	if (gpio_is_valid(pdata->holder.gpio))
		strlcpy(pdata->holder.irq_name, HALL_HOLDER_IRQ_NAME, sizeof(pdata->holder.irq_name));

	pr_info("hall_parse_dt success\n");
	return 0;
}

static int hall_probe(struct platform_device *pdev)
{
	int rc;
	struct hall_switch_info *hall_info;

	pr_info("hall_probe\n");

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "DT node not found\n");
		return -ENODEV;
	}

	hall_info = devm_kzalloc(&pdev->dev, sizeof(*hall_info), GFP_KERNEL);
	if (!hall_info) {
		dev_err(&pdev->dev, "failed to alloc memory for module data\n");
		return -ENOMEM;
	}

	rc = hall_parse_dt(&pdev->dev, hall_info);
	if (rc) {
		dev_err(&pdev->dev, "DT parsing failed\n");
		return rc;
	}

	platform_set_drvdata(pdev, hall_info);

	hall_info->ipdev = input_allocate_device();
	if (!hall_info->ipdev) {
		dev_err(&pdev->dev, "input_allocate_device fail\n");
		return -ENOMEM;
	}
	hall_info->ipdev->name = "hall-switch-input";
	input_set_capability(hall_info->ipdev, EV_KEY, KEY_HALL_OPEN);
	input_set_capability(hall_info->ipdev, EV_KEY, KEY_HALL_CLOSE);
	input_set_capability(hall_info->ipdev, EV_KEY, KEY_HALL_HOLDER_OPEN);
	input_set_capability(hall_info->ipdev, EV_KEY, KEY_HALL_HOLDER_CLOSE);

	rc = input_register_device(hall_info->ipdev);
	if (rc) {
		dev_err(&pdev->dev, "input_register_device fail rc=%d\n", rc);
		input_free_device(hall_info->ipdev);
		return rc;
	}

	if (gpio_is_valid(hall_info->cover.gpio)) {
		rc = hall_init_irq(&hall_info->cover);
		if (rc)
			goto err_unregister_input;
	}

	if (gpio_is_valid(hall_info->cam.gpio)) {
		rc = hall_init_irq(&hall_info->cam);
		if (rc)
			goto err_free_cover;
	}

	if (gpio_is_valid(hall_info->holder.gpio)) {
		rc = hall_init_irq(&hall_info->holder);
		if (rc)
			goto err_free_cam;
	}

	pr_info("hall_probe end\n");
	return 0;

err_free_cam:
	if (gpio_is_valid(hall_info->cam.gpio)) {
		free_irq(hall_info->cam.irq, &hall_info->cam);
		gpio_free(hall_info->cam.gpio);
		switch_dev_unregister(&hall_info->cam.sdev);
	}
err_free_cover:
	if (gpio_is_valid(hall_info->cover.gpio)) {
		free_irq(hall_info->cover.irq, &hall_info->cover);
		gpio_free(hall_info->cover.gpio);
		switch_dev_unregister(&hall_info->cover.sdev);
	}
err_unregister_input:
	input_unregister_device(hall_info->ipdev);
	return rc;
}

static int hall_remove(struct platform_device *pdev)
{
	struct hall_switch_info *hall = platform_get_drvdata(pdev);

	pr_info("hall_remove\n");

	if (gpio_is_valid(hall->holder.gpio)) {
		free_irq(hall->holder.irq, &hall->holder);
		gpio_free(hall->holder.gpio);
		switch_dev_unregister(&hall->holder.sdev);
	}
	if (gpio_is_valid(hall->cam.gpio)) {
		free_irq(hall->cam.irq, &hall->cam);
		gpio_free(hall->cam.gpio);
		switch_dev_unregister(&hall->cam.sdev);
	}
	if (gpio_is_valid(hall->cover.gpio)) {
		free_irq(hall->cover.irq, &hall->cover);
		gpio_free(hall->cover.gpio);
		switch_dev_unregister(&hall->cover.sdev);
	}

	input_unregister_device(hall->ipdev);

	return 0;
}

static const struct of_device_id sn_match_table[] = {
	{ .compatible = "hall-switch,och175", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sn_match_table);

static struct platform_driver hall_driver = {
	.probe      = hall_probe,
	.remove     = hall_remove,
	.driver     = {
		.name   = "hall-switch",
		.of_match_table = sn_match_table,
	},
};

module_platform_driver(hall_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("mawenke");
MODULE_DESCRIPTION("Hall switch sensor driver");
