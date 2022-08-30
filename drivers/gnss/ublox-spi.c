// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022 RnD Center "ELVEES", JSC
 */

#include <linux/gpio/consumer.h>
#include <linux/gnss.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/spi/spi.h>

#define BUF_SIZE 64

struct gnss_spi {
	struct delayed_work work;
	struct workqueue_struct *wq;
	struct gnss_device *gdev;
	struct spi_device *spi;
	u8 *rx_buf;
	u8 *tx_dummy_buf;
	bool is_opened;
};

static int gnss_spi_open(struct gnss_device *gdev)
{
	struct gnss_spi *gnss_spi = gnss_get_drvdata(gdev);

	gnss_spi->is_opened = true;
	queue_delayed_work(gnss_spi->wq, &gnss_spi->work, 0);

	return 0;
}

static void gnss_spi_close(struct gnss_device *gdev)
{
	struct gnss_spi *gnss_spi = gnss_get_drvdata(gdev);

	gnss_spi->is_opened = false;
	cancel_delayed_work_sync(&gnss_spi->work);
}

static int gnss_spi_read_locked(struct gnss_spi *gnss_spi,
				const unsigned char *tx_buf,
				size_t tx_count)
{
	struct spi_message message;
	struct spi_transfer tran = {
		.rx_buf = gnss_spi->rx_buf,
	};
	int sended = 0;
	char *ptr_end;
	size_t len;
	int ret;

	if (!tx_buf) {
		tran.tx_buf = gnss_spi->tx_dummy_buf;
		tran.len = 1;
		spi_message_init_with_transfers(&message, &tran, 1);
		ret = spi_sync_locked(gnss_spi->spi, &message);
		if (ret)
			return ret;

		if (gnss_spi->rx_buf[0] == 0xff)
			return 0;

		gnss_insert_raw(gnss_spi->gdev, gnss_spi->rx_buf, 1);
	}

	tran.cs_change = 1;
	do {
		tran.rx_buf = gnss_spi->rx_buf;
		if (tx_buf) {
			tran.len = tx_count > BUF_SIZE ? BUF_SIZE : tx_count;
			tran.tx_buf = tx_buf + sended;
			tx_count -= tran.len;
			sended += tran.len;
		} else
			tran.len = BUF_SIZE;

		spi_message_init_with_transfers(&message, &tran, 1);
		ret = spi_sync_locked(gnss_spi->spi, &message);
		if (ret)
			return ret;

		ptr_end = memchr(gnss_spi->rx_buf, 0xff, BUF_SIZE);
		len = ptr_end ? (size_t)(ptr_end - (char *)gnss_spi->rx_buf) :
		      BUF_SIZE;
		gnss_insert_raw(gnss_spi->gdev, gnss_spi->rx_buf, len);
	} while ((tx_buf && tx_count) || (!tx_buf && len == BUF_SIZE));

	return 1;
}

static int gnss_spi_write_raw(struct gnss_device *gdev,
			      const unsigned char *buf,
			      size_t count)
{
	struct gnss_spi *gnss_spi = gnss_get_drvdata(gdev);
	int ret;

	cancel_delayed_work_sync(&gnss_spi->work);

	spi_bus_lock(gnss_spi->spi->controller);
	ret = gnss_spi_read_locked(gnss_spi, buf, count);
	while (ret > 0)
		ret = gnss_spi_read_locked(gnss_spi, NULL, 0);

	spi_bus_unlock(gnss_spi->spi->controller);

	if (gnss_spi->is_opened)
		queue_delayed_work(gnss_spi->wq, &gnss_spi->work,
				   msecs_to_jiffies(200));

	return (!ret) ? count : ret;
}

static const struct gnss_operations gnss_spi_ops = {
	.open = gnss_spi_open,
	.close = gnss_spi_close,
	.write_raw = gnss_spi_write_raw,
};

static void gnss_spi_read(struct work_struct *work)
{
	struct gnss_spi *gnss_spi = container_of(work, struct gnss_spi,
						 work.work);
	int ret;

	spi_bus_lock(gnss_spi->spi->controller);
	do {
		ret = gnss_spi_read_locked(gnss_spi, NULL, 0);
	} while (ret > 0);
	spi_bus_unlock(gnss_spi->spi->controller);

	if (ret < 0)
		dev_err(&gnss_spi->spi->dev, "SPI transfer error (%d)\n", ret);

	if (gnss_spi->is_opened)
		queue_delayed_work(gnss_spi->wq, &gnss_spi->work,
				   msecs_to_jiffies(200));
}

static int gnss_spi_probe(struct spi_device *spi)
{
	struct gnss_spi *gnss_spi;
	struct gpio_desc *reset_gpio;
	int ret;

	gnss_spi = devm_kzalloc(&spi->dev, sizeof(*gnss_spi), GFP_KERNEL);
	if (!gnss_spi)
		return -ENOMEM;
	spi_set_drvdata(spi, gnss_spi);
	reset_gpio = devm_gpiod_get_optional(&spi->dev, "reset",
					     GPIOD_OUT_HIGH);
	if (IS_ERR(reset_gpio)) {
		ret = PTR_ERR(reset_gpio);
		dev_err(&spi->dev, "failed to get reset GPIO (%d)\n", ret);
		return ret;
	}

	gnss_spi->spi = spi;
	gnss_spi->rx_buf = devm_kzalloc(&spi->dev, BUF_SIZE, GFP_KERNEL);
	gnss_spi->tx_dummy_buf = devm_kzalloc(&spi->dev, BUF_SIZE, GFP_KERNEL);
	if (!gnss_spi->rx_buf || !gnss_spi->tx_dummy_buf)
		return -ENOMEM;

	memset(gnss_spi->tx_dummy_buf, 0xff, BUF_SIZE);
	gnss_spi->gdev = gnss_allocate_device(&spi->dev);
	if (!gnss_spi->gdev)
		return -ENOMEM;

	gnss_spi->gdev->ops = &gnss_spi_ops;
	gnss_spi->gdev->type = GNSS_TYPE_NMEA;
	gnss_set_drvdata(gnss_spi->gdev, gnss_spi);
	gnss_spi->wq = create_workqueue("qnss_spi");
	INIT_DELAYED_WORK(&gnss_spi->work, gnss_spi_read);

	ret = gnss_register_device(gnss_spi->gdev);
	if (ret)
		return ret;

	return 0;
}

static int gnss_spi_remove(struct spi_device *spi)
{
	struct gnss_spi *gnss_spi = spi_get_drvdata(spi);

	gnss_deregister_device(gnss_spi->gdev);

	return 0;
}

static int __init gnss_spi_register_driver(struct spi_driver *sdrv)
{
	return spi_register_driver(sdrv);
}

#ifdef CONFIG_OF
static const struct of_device_id gnss_spi_of_id[] = {
	{ .compatible = "u-blox,zoe-m8b-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, gnss_spi_of_id);
#endif

static const struct spi_device_id gnss_spi_id[] = {
	{ "zoe-m8b-spi", 0, },
	{ }
};
MODULE_DEVICE_TABLE(spi, gnss_spi_id);

static struct spi_driver gnss_spi_driver = {
	.driver = {
		.name		= "gnss_spi",
		.of_match_table	= of_match_ptr(gnss_spi_of_id),
	},
	.probe	  = gnss_spi_probe,
	.remove	  = gnss_spi_remove,
	.id_table = gnss_spi_id,
};
module_driver(gnss_spi_driver, gnss_spi_register_driver, spi_unregister_driver);

MODULE_DESCRIPTION("U-blox SPI GNSS receiver driver");
MODULE_LICENSE("GPL v2");
