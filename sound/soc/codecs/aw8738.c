// SPDX-License-Identifier: GPL-2.0-only

#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <sound/soc.h>

struct aw8738_priv {
	unsigned int gpio_mode;
	unsigned int mode;
};

static int aw8738_drv_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct aw8738_priv *aw = snd_soc_codec_get_drvdata(codec);
	int i;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		pr_info("%s: AW8738 Powering Up, mode=%d\n", __func__, aw->mode);
		
		for (i = 0; i < (aw->mode - 1); i++) {
			gpio_set_value(aw->gpio_mode, 1);
			udelay(2);
			gpio_set_value(aw->gpio_mode, 0);
			udelay(2);
		}
		gpio_set_value(aw->gpio_mode, 1);
		
		msleep(40);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		pr_info("%s: AW8738 Powering Down\n", __func__);
		gpio_set_value(aw->gpio_mode, 0);
		usleep_range(1000, 2000);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dapm_widget aw8738_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("IN"),
	SND_SOC_DAPM_OUT_DRV_E("Amp Driver", SND_SOC_NOPM, 0, 0, NULL, 0, aw8738_drv_event,
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("OUT"),
};

static const struct snd_soc_dapm_route aw8738_dapm_routes[] = {
	{ "Amp Driver", NULL, "IN" },
	{ "OUT", NULL, "Amp Driver" },
};

static const struct snd_soc_codec_driver soc_codec_dev_aw8738 = {
	.dapm_widgets = aw8738_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(aw8738_dapm_widgets),
	.dapm_routes = aw8738_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(aw8738_dapm_routes),
};

static int aw8738_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct aw8738_priv *aw;
	int ret;

	aw = devm_kzalloc(dev, sizeof(*aw), GFP_KERNEL);
	if (!aw)
		return -ENOMEM;
	dev_set_drvdata(dev, aw); /* Usamos dev_set_drvdata */

	aw->gpio_mode = of_get_named_gpio(dev->of_node, "mode-gpios", 0);
	if (!gpio_is_valid(aw->gpio_mode)) {
		dev_err(dev, "Failed to get 'mode-gpios'\n");
		return -EINVAL;
	}

	ret = devm_gpio_request_one(dev, aw->gpio_mode, GPIOF_OUT_INIT_LOW, "aw8738-mode");
	if (ret) {
		dev_err(dev, "Failed to request 'mode-gpios'\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "awinic,mode", &aw->mode);
	if (ret) {
		dev_err(dev, "Missing 'awinic,mode' property in device tree\n");
		return -EINVAL;
	}
	if (aw->mode == 0) {
		dev_err(dev, "'awinic,mode' cannot be zero\n");
		return -EINVAL;
	}

	return snd_soc_register_codec(&pdev->dev, &soc_codec_dev_aw8738, NULL, 0);
}

static int aw8738_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id aw8738_of_match[] = {
	{ .compatible = "awinic,aw8738" },
	{ .compatible = "awinic,aw8736" },
	{ }
};
MODULE_DEVICE_TABLE(of, aw8738_of_match);
#endif

static struct platform_driver aw8738_driver = {
	.probe	= aw8738_probe,
	.remove = aw8738_remove,
	.driver = {
		.name = "aw8738",
		.of_match_table = of_match_ptr(aw8738_of_match),
	},
};
module_platform_driver(aw8738_driver);

MODULE_DESCRIPTION("Awinic AW8738/AW8736 Amplifier Driver");
MODULE_LICENSE("GPL v2");
