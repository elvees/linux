// SPDX-License-Identifier: GPL-2.0
// Copyright 2025 RnD Center "ELVEES", JSC

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include <soc/elvees/mcom03/mcom03-helper.h>

int mcom03_sprintf(char *buf, size_t *size, int *pos, const char *fmt, ...)
{
	va_list args;
	int n;

	if (*pos >= *size)
		return -ENOMEM;

	va_start(args, fmt);
	n = vsnprintf(NULL, 0, fmt, args);
	va_end(args);

	if (n < 0)
		return -EINVAL;
	if (n >= *size)
		return -ENOMEM;

	va_start(args, fmt);
	n = vsnprintf(&buf[*pos], *size, fmt, args);
	va_end(args);

	*pos += n;
	*size -= n;

	return 0;
}
