// SPDX-License-Identifier: GPL-2.0
/*
 * ELVEES serial driver for printing messages into memory
 *
 * Copyright 2019 RnD Center "ELVEES", JSC
 */

#include <linux/module.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/io.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

#define P2M_SERIAL_DEV_NAME	"ttyP2M"

static struct uart_port p2m_port;

static struct console p2m_console;

#define DRIVER_NAME	"p2m"

static struct uart_driver p2m_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= DRIVER_NAME,
	.dev_name	= P2M_SERIAL_DEV_NAME,
	.major		= 0,
	.minor		= 0,
	.nr		= 1,
	.cons		= &p2m_console,
};

static void p2m_stop_rx(struct uart_port *port)
{
}

static void p2m_stop_tx(struct uart_port *port)
{
}

static unsigned int p2m_tx_empty(struct uart_port *port)
{
	return TIOCSER_TEMT;
}

static void p2m_start_tx(struct uart_port *port)
{
}

static unsigned int p2m_get_mctrl(struct uart_port *port)
{
	/*
	 * Pretend we have a Modem status reg and following bits are
	 * always set, to satify the serial core state machine
	 * (DSR) Data Set Ready
	 * (CTS) Clear To Send
	 * (CAR) Carrier Detect
	 */
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void p2m_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static void p2m_break_ctl(struct uart_port *port, int break_state)
{
}

static int p2m_startup(struct uart_port *port)
{
	return 0;
}

/* This is not really needed */
static void p2m_shutdown(struct uart_port *port)
{
}

static void p2m_set_termios(struct uart_port *port, struct ktermios *new,
			    struct ktermios *old)
{
}

static const char *p2m_type(struct uart_port *port)
{
	return NULL;
}

static void p2m_release_port(struct uart_port *port)
{
}

static int p2m_request_port(struct uart_port *port)
{
	return 0;
}

static int p2m_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	return 0;
}

static void p2m_config_port(struct uart_port *port, int flags)
{
}

static const struct uart_ops p2m_serial_pops = {
	.tx_empty	= p2m_tx_empty,
	.set_mctrl	= p2m_set_mctrl,
	.get_mctrl	= p2m_get_mctrl,
	.stop_tx	= p2m_stop_tx,
	.start_tx	= p2m_start_tx,
	.stop_rx	= p2m_stop_rx,
	.break_ctl	= p2m_break_ctl,
	.startup	= p2m_startup,
	.shutdown	= p2m_shutdown,
	.set_termios	= p2m_set_termios,
	.type		= p2m_type,
	.release_port	= p2m_release_port,
	.request_port	= p2m_request_port,
	.config_port	= p2m_config_port,
	.verify_port	= p2m_verify_port,
};

static int p2m_console_setup(struct console *co, char *options)
{
	return 0;
}

/* The following аlgorithm to print messages during RTL simulation is assumed:
 *  1. The p2m driver puts message into shared buffer.
 *  2. The p2m driver writes shared buffer address to print agent register.
 *  3. Print agent prints message in RTL simulation environment.
 */

#define P2M_SHARED_BUFFER_OFFSET	0x800
#define P2M_AGENT_BUFFER_ADDR		0xC
#define P2M_AGENT_MAX_STRING_SIZE	4096

static void p2m_putstr(struct uart_port *port, const char *s,
		       unsigned int count)
{
	unsigned int i;

	if (count >= P2M_AGENT_MAX_STRING_SIZE)
		count = P2M_AGENT_MAX_STRING_SIZE - 1;

	for (i = 0; i < count; i++, s++)
		writeb(*s, port->membase + P2M_SHARED_BUFFER_OFFSET + i);

	/* Print agent requires that every message ends with null character */
	writeb(0, port->membase + P2M_SHARED_BUFFER_OFFSET + count);

	writel(port->mapbase + P2M_SHARED_BUFFER_OFFSET,
	       port->membase + P2M_AGENT_BUFFER_ADDR);
}

static void p2m_console_write(struct console *co, const char *s,
			      unsigned int count)
{
	unsigned long flags;

	spin_lock_irqsave(&p2m_port.lock, flags);
	p2m_putstr(&p2m_port, s, count);
	spin_unlock_irqrestore(&p2m_port.lock, flags);
}

static struct console p2m_console = {
	.name	= P2M_SERIAL_DEV_NAME,
	.write	= p2m_console_write,
	.device	= uart_console_device,
	.setup	= p2m_console_setup,
	.flags	= CON_PRINTBUFFER,
	.index	= -1,
	.data	= &p2m_uart_driver
};

static void p2m_early_write(struct console *con, const char *s,
			    unsigned int n)
{
	struct earlycon_device *dev = con->data;

	p2m_putstr(&dev->port, s, n);
}

static int __init p2m_early_console_setup(struct earlycon_device *dev,
					  const char *opt)
{
	dev->con->write = p2m_early_write;

	return 0;
}
OF_EARLYCON_DECLARE(p2m_uart, "elvees,p2m", p2m_early_console_setup);

static int p2m_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;

	if (!np)
		return -ENODEV;

	p2m_port.membase = of_iomap(np, 0);
	if (!p2m_port.membase)
		return -ENXIO;

	p2m_port.dev = &pdev->dev;
	p2m_port.iotype = UPIO_MEM;
	p2m_port.flags = UPF_BOOT_AUTOCONF;
	p2m_port.line = 0;
	p2m_port.ops = &p2m_serial_pops;

	return uart_add_one_port(&p2m_uart_driver, &p2m_port);
}

static int p2m_remove(struct platform_device *pdev)
{
	/* This will never be called */
	return 0;
}

static const struct of_device_id p2m_dt_ids[] = {
	{ .compatible = "elvees,p2m" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, p2m_dt_ids);

static struct platform_driver p2m_platform_driver = {
	.probe = p2m_probe,
	.remove = p2m_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table  = p2m_dt_ids,
	},
};

static int __init p2m_init(void)
{
	int ret;

	ret = uart_register_driver(&p2m_uart_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&p2m_platform_driver);
	if (ret)
		uart_unregister_driver(&p2m_uart_driver);

	return ret;
}

static void __exit p2m_exit(void)
{
	platform_driver_unregister(&p2m_platform_driver);
	uart_unregister_driver(&p2m_uart_driver);
}

module_init(p2m_init);
module_exit(p2m_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_DESCRIPTION("ELVEES serial driver for printing messages into memory");
