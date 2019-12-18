// SPDX-License-Identifier: GPL-2.0+
// Copyright 2018-2019 RnD Center "ELVEES", JSC

#include <linux/kernel.h>
#include <linux/anon_inodes.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/dmaengine.h>
#include <linux/fs.h>
#include <linux/genalloc.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/poll.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <uapi/linux/delcore30m.h>

#include "regs.h"

/**
 * struct delcore30m_private_data - Driver private data
 * @dev:      Device info.
 * @cmn_regs: Virtual (kernel address space) addresses of common registers.
 * @dsp_regs: Virtual (kernel address space) addresses of DSP registers.
 * @spinlock: Virtual (kernel address space) addresses of spinlock registers.
 * @sdma:     Register map for SDMA registers.
 * @xyram_pool: General pool for XYRAM.
 * @pram:     PRAM resource and firmware size.
 * @count:    Driver's users count.
 * @fwready:  DSP's firmware ready flag.
 * @enqueued: List of enqueued jobs.
 * @running:  List of running jobs.
 * @lock:     Lock for jobs.
 * @sdmas:    Mask of SDMA channels.
 * @cores:    Mask of cores.
 * @reslock:  Lock for resources.
 * @stack:    Physical and virtual (kernel address space) addresses
 *            of core stacks.
 * @clock_count: Clock count.
 * @clocks:   Clocks info.
 * @dev_num:  Device numbers.
 * @cdev:     Char device info.
 * @hw:       Hardware info struct.
 */
struct delcore30m_private_data {
	struct device *dev;

	void __iomem *cmn_regs;
	void __iomem *dsp_regs[MAX_CORES];
	void __iomem *spinlock;

	struct regmap *sdma;

	struct gen_pool *xyram_pool[MAX_CORES];

	struct delcore30m_pram {
		struct resource *pram_res;
		size_t firmware_size;
	} pram[MAX_CORES];

	atomic_t count;
	bool fwready[MAX_CORES];

	struct list_head enqueued, running;
	spinlock_t lock;

	unsigned long sdmas;
	unsigned long cores;
	spinlock_t reslock;

	struct delcore30m_stack {
		dma_addr_t paddr;
		void *vaddr;
	} stack[MAX_CORES];

	int clock_count;
	struct clk **clocks;

	dev_t dev_num;
	struct cdev cdev;
	struct delcore30m_hardware hw;
};

static struct class *class;

/**
 * struct delcore30m_buffer_desc - Internal data about allocated buffer
 * @pdata: Pointer to driver private data.
 * @buf:   Embedded user-visible buffer data.
 * @dmabuf: Pointer to DMA buffer description associated with buffer.
 * @paddr: Physical address of allocated memory.
 * @vaddr: Virtual address of allocated memory in kernel address space.
 */
struct delcore30m_buffer_desc {
	struct delcore30m_private_data *pdata;

	struct delcore30m_buffer buf;
	struct dma_buf *dmabuf;

	dma_addr_t paddr;
	void *vaddr;
};

/**
 * struct buf_info - Information of imported buffer
 * @attach: DMA buffer attachment.
 * @sgt:    SG table returned after attachment mapping.
 * @dir:    Data direction.
 */
struct buf_info {
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	enum dma_data_direction dir;
};

/**
 * struct delcore30m_job_desc - Internal data about created job
 * @pdata: Pointer to driver private data.
 * @job:   Embedded user-visible job data.
 * @profilebuf: Buffers for performance data.
 * @bufs:  Array of &struct buf_info for buffers.
 * @num_bufs: Buffer number.
 * @cores: Mask of cores.
 * @sdmas: Mask of SDMA channels.
 * @wait:  Wait queue.
 * @list:  Job list node.
 */
struct delcore30m_job_desc {
	struct delcore30m_private_data *pdata;

	struct delcore30m_job job;
	struct delcore30m_buffer_desc profilebuf[MAX_CORES];

	struct buf_info bufs[MAX_INPUTS + MAX_OUTPUTS];
	int num_bufs;

	unsigned long cores;
	unsigned long sdmas;

	wait_queue_head_t wait;

	struct list_head list;
};

/**
 * struct delcore30m_resource_desc - Internal data about created resource
 * @pdata: Pointer to driver private data.
 * @resource:   Embedded user-visible resource data.
 */
struct delcore30m_resource_desc {
	struct delcore30m_private_data *pdata;
	struct delcore30m_resource resource;
};

/**
 * struct sdma_program_buf - Internal data about SDMA program
 * @start: Pointer to start of SDMA program buffer.
 * @pos:   Pointer to current position of SDMA program buffer.
 * @end:   Pointer to end of SDMA program buffer.
 */
struct sdma_program_buf {
	char *start, *pos, *end;
};

/**
 * struct timestamp - Internal data about profile buffer entry
 * @tag:  Identifier of entry.
 * @time: Value of TOTAL_CLK_CNTR for this entry.
 */

struct timestamp {
	u32 tag;
	u32 time;
};

#define phys_to_xyram(x) ((x) & 0xFFFFF)

static const struct file_operations delcore30m_resource_fops;

static inline u16 delcore30m_readw(struct delcore30m_private_data
				   const *const pdata, u8 core, off_t reg)
{
	return ioread16(pdata->dsp_regs[core] + reg);
}

static inline u32 delcore30m_readl(struct delcore30m_private_data
				   const *const pdata, u8 core, off_t reg)
{
	return ioread32(pdata->dsp_regs[core] + reg);
}

static inline u32 delcore30m_readl_cmn(struct delcore30m_private_data
				       const *const pdata, off_t reg)
{
	return ioread32(pdata->cmn_regs + reg);
}

static inline void delcore30m_writew(struct delcore30m_private_data
				    const *const pdata, u8 core, off_t reg,
				    u16 value)
{
	return iowrite16(value, pdata->dsp_regs[core] + reg);
}

static inline void delcore30m_writel(struct delcore30m_private_data
				     const *const pdata, u8 core, off_t reg,
				     u32 value)
{
	return iowrite32(value, pdata->dsp_regs[core] + reg);
}

static inline void delcore30m_writel_cmn(struct delcore30m_private_data
					 const *const pdata, off_t reg,
					 u32 value)
{
	return iowrite32(value, pdata->cmn_regs + reg);
}

static int delcore30m_mem_alloc(struct delcore30m_buffer_desc *buf_desc)
{
	buf_desc->buf.size = ALIGN(buf_desc->buf.size, PAGE_SIZE);

	switch (buf_desc->buf.type) {
	case DELCORE30M_MEMORY_XYRAM:
		buf_desc->vaddr = gen_pool_dma_alloc(
			buf_desc->pdata->xyram_pool[buf_desc->buf.core_num],
			buf_desc->buf.size,
			&buf_desc->paddr);
		if (!buf_desc->vaddr) {
			dev_err(buf_desc->pdata->dev,
				"Failed to allocate XYRAM\n");
			return -ENOMEM;
		}

		break;
	case DELCORE30M_MEMORY_SYSTEM:
		buf_desc->vaddr = dma_alloc_coherent(buf_desc->pdata->dev,
						     buf_desc->buf.size,
						     &buf_desc->paddr,
						     GFP_KERNEL);
		if (!buf_desc->vaddr) {
			dev_err(buf_desc->pdata->dev,
				"Failed to allocate system memory\n");
			return -ENOMEM;
		}
		break;
	default:
		dev_err(buf_desc->pdata->dev, "Unknown memory type\n");
		return -EINVAL;
	}

	return 0;
}

static void delcore30m_mem_free(struct delcore30m_buffer_desc *buf_desc)
{
	switch (buf_desc->buf.type) {
	case DELCORE30M_MEMORY_XYRAM:
		gen_pool_free(
			buf_desc->pdata->xyram_pool[buf_desc->buf.core_num],
			(unsigned long) buf_desc->vaddr, buf_desc->buf.size);
		break;
	case DELCORE30M_MEMORY_SYSTEM:
		dma_free_coherent(buf_desc->pdata->dev, buf_desc->buf.size,
				  buf_desc->vaddr, buf_desc->paddr);
		break;
	default:
		dev_err(buf_desc->pdata->dev, "Unknown memory type\n");
	}
}

static u32 cpu_to_delcore30m(dma_addr_t address)
{
	return address >> 2;
}

static dma_addr_t delcore30m_to_cpu(u32 address)
{
	return address << 2;
}

static void write_arg_regs(struct delcore30m_job_desc *desc,
			   u8 core, int n, u32 data)
{
	const off_t reg = n == 0 ? DELCORE30M_R2 : DELCORE30M_R4;

	delcore30m_writel(desc->pdata, core, reg, data);
}

static ptrdiff_t set_args(struct delcore30m_job_desc *desc, u8 core)
{
	int i;
	dma_addr_t dma_address;
	int inregs = min(desc->num_bufs, 2);
	int instack = desc->num_bufs - inregs;
	u64 *stack = (u64 *)desc->pdata->stack[core].vaddr - instack +
		     STACK_SIZE / sizeof(u64);

	/* Set registers R2 and R4 */
	for (i = 0; i < inregs; ++i) {
		dma_address = sg_dma_address(desc->bufs[i].sgt->sgl);
		write_arg_regs(desc, core, i, dma_address);
	}

	/* Args 0 and 1 in R2 and R4. Each arg - 8 bytes */
	for (i = 2; i < desc->num_bufs; ++i) {
		dma_address = sg_dma_address(desc->bufs[i].sgt->sgl);
		stack[i - 2] = dma_address;
	}
	return (u8 *)stack - (u8 *)desc->pdata->stack[core].vaddr;
}

static void buf_info_detach(struct device *dev, struct buf_info *info)
{
	struct dma_buf *dmabuf;

	if (!IS_ERR_OR_NULL(info->attach)) {
		dmabuf = info->attach->dmabuf;

		dma_buf_detach(dmabuf, info->attach);
		info->attach = NULL;

		dma_buf_put(dmabuf);
	}
}

static int buf_info_attach(struct device *dev, struct buf_info *info, int fd)
{
	struct dma_buf *dmabuf = dma_buf_get(fd);

	if (IS_ERR(dmabuf)) {
		dev_err(dev, "Failed to get DMA buffer for fd %d\n", fd);
		return PTR_ERR(dmabuf);
	}

	info->attach = dma_buf_attach(dmabuf, dev);

	if (IS_ERR(info->attach)) {
		dev_err(dev, "Failed to attach DMA buffer for fd %d\n", fd);
		dma_buf_put(dmabuf);
		return PTR_ERR(info->attach);
	}

	return 0;
}

static void detach_buffers(struct delcore30m_job_desc *desc)
{
	int i;
	struct device *dev = desc->pdata->dev;

	for (i = desc->num_bufs - 1; i >= 0; --i)
		buf_info_detach(dev, &desc->bufs[i]);
}

static int attach_buffers(struct delcore30m_job_desc *desc)
{
	int rc, i, j;
	struct device *dev = desc->pdata->dev;

	for (i = 0, j = 0; i < desc->job.inum; ++i, ++j) {
		desc->bufs[j].dir = DMA_TO_DEVICE;
		rc = buf_info_attach(dev, &desc->bufs[j], desc->job.input[i]);
		if (rc)
			goto err;
	}

	for (i = 0; i < desc->job.onum; ++i, ++j) {
		desc->bufs[j].dir = DMA_BIDIRECTIONAL;
		rc = buf_info_attach(dev, &desc->bufs[j], desc->job.output[i]);
		if (rc)
			goto err;
	}

	return 0;

err:
	detach_buffers(desc);
	return rc;
}

static void buf_info_unmap(struct buf_info *info)
{
	if (!IS_ERR_OR_NULL(info->sgt)) {
		dma_buf_unmap_attachment(info->attach, info->sgt, info->dir);
		info->sgt = NULL;
	}
}

static int buf_info_map(struct buf_info *info)
{
	/* TODO: Sanity check. To be removed. */
	if (WARN_ON(IS_ERR_OR_NULL(info->attach)))
		return -EINVAL;

	info->sgt = dma_buf_map_attachment(info->attach, info->dir);
	if (IS_ERR(info->sgt))
		return PTR_ERR(info->sgt);

	return 0;
}

static int unmap_buffers(struct delcore30m_job_desc *desc)
{
	int i;

	for (i = desc->num_bufs - 1; i >= 0; --i)
		buf_info_unmap(&desc->bufs[i]);

	return 0;
}

static int map_buffers(struct delcore30m_job_desc *desc)
{
	int rc, i;

	for (i = 0; i < desc->num_bufs; ++i) {
		rc = buf_info_map(&desc->bufs[i]);
		if (rc)
			goto err;
	}

	return 0;

err:
	unmap_buffers(desc);
	return 1;
}

static void delcore30m_run_job(struct delcore30m_job_desc *desc)
{
	struct delcore30m_private_data *pdata = desc->pdata;
	u8 thread_num = 0;
	ptrdiff_t stack_offset;
	u32 reg_value;
	int i;

	for_each_set_bit(i, &desc->cores, MAX_CORES) {
		delcore30m_writel(pdata, i, DELCORE30M_R0, thread_num++);

		stack_offset = set_args(desc, i);

		reg_value = pdata->stack[i].paddr + stack_offset;
		reg_value = cpu_to_delcore30m(phys_to_xyram(reg_value));
		delcore30m_writel(pdata, i, DELCORE30M_A6, reg_value);
		delcore30m_writel(pdata, i, DELCORE30M_A7, reg_value);
		desc->job.status = DELCORE30M_JOB_RUNNING;

		reg_value = pdata->pram[i].pram_res->start;
		delcore30m_writew(pdata, i, DELCORE30M_PC,
				  cpu_to_delcore30m(phys_to_xyram(reg_value)));

		if (desc->job.flags & DELCORE30M_PROFILE) {
			reg_value = desc->profilebuf[i].paddr;
			delcore30m_writel(pdata, i, DELCORE30M_A5,
					  cpu_to_delcore30m(reg_value));
			delcore30m_writel_cmn(pdata, DELCORE30M_TOTAL_CLK_CNTR,
					      0);
		}

		reg_value = delcore30m_readl_cmn(pdata, DELCORE30M_MASKR_DSP);
		reg_value |= DELCORE30M_QSTR_CORE_MASK(i);
		delcore30m_writel_cmn(pdata, DELCORE30M_MASKR_DSP, reg_value);

		/* Sequential cores start */
		if (desc->cores != ELCORE30M_CORE_ALL)
			delcore30m_writel(pdata, i, DELCORE30M_DCSR,
					  DELCORE30M_DCSR_RUN);
	}

	/* Simultaneous cores start */
	if (desc->cores == ELCORE30M_CORE_ALL)
		delcore30m_writel_cmn(pdata, DELCORE30M_CSR_DSP,
				      DELCORE30M_CSR_SYNSTART);
}

static void delcore30m_try_run(struct delcore30m_private_data *pdata)
{
	struct delcore30m_job_desc *desc, *next;
	u8 core;
	u32 stopped_cores = 0;

	for (core = 0; core < MAX_CORES; ++core)
		if (!(delcore30m_readw(pdata, core, DELCORE30M_DCSR) &
		      DELCORE30M_DCSR_RUN))
			stopped_cores |= BIT(core);

	list_for_each_entry_safe(desc, next, &pdata->enqueued, list) {
		/* Search job, that can be placed on stopped cores */
		if ((stopped_cores & desc->cores) != desc->cores)
			continue;

		delcore30m_run_job(desc);

		list_del(&desc->list);
		list_add_tail(&desc->list, &pdata->running);
	}
}

void reset_cores(struct delcore30m_job_desc *job_desc)
{
	struct delcore30m_private_data *pdata = job_desc->pdata;
	u32 i;
	off_t offset;

	for_each_set_bit(i, &job_desc->cores, MAX_CORES) {
		delcore30m_writew(pdata, i, DELCORE30M_DCSR, 0x0);
		delcore30m_writew(pdata, i, DELCORE30M_SR, 0x0);
		delcore30m_writew(pdata, i, DELCORE30M_LA, 0xFFFF);
		delcore30m_writew(pdata, i, DELCORE30M_CSL, 0x0);
		delcore30m_writew(pdata, i, DELCORE30M_LC, 0x0);
		delcore30m_writew(pdata, i, DELCORE30M_CSH, 0x0);
		delcore30m_writew(pdata, i, DELCORE30M_SP, 0x0);
		delcore30m_writel(pdata, i, DELCORE30M_IMASKR, 0x0);

		offset = DELCORE30M_A0;

		for (; offset <= DELCORE30M_A5; offset += 4)
			delcore30m_writel(pdata, i, offset, 0x0);
	}
}

static void job_cancel(struct delcore30m_job_desc *job_desc)
{
	struct delcore30m_private_data *pdata;
	unsigned long flags;

	pdata = job_desc->pdata;

	spin_lock_irqsave(&pdata->lock, flags);
	if (job_desc->job.status == DELCORE30M_JOB_IDLE) {
		spin_unlock_irqrestore(&pdata->lock, flags);
		return;
	}

	if (job_desc->job.status == DELCORE30M_JOB_RUNNING)
		reset_cores(job_desc);

	job_desc->job.rc = DELCORE30M_JOB_CANCELLED;
	job_desc->job.status = DELCORE30M_JOB_IDLE;

	list_del(&job_desc->list);

	if (!list_empty(&pdata->enqueued))
		delcore30m_try_run(pdata);

	spin_unlock_irqrestore(&pdata->lock, flags);
	wake_up_interruptible(&job_desc->wait);
}

static int delcore30m_job_release(struct inode *inode, struct file *file)
{
	struct delcore30m_job_desc *desc = file->private_data;
	int i;

	job_cancel(desc);

	if (desc->job.flags & DELCORE30M_PROFILE)
		for_each_set_bit(i, &desc->cores, MAX_CORES)
			dma_free_coherent(desc->pdata->dev,
					desc->profilebuf[i].buf.size,
					desc->profilebuf[i].vaddr,
					desc->profilebuf[i].paddr);

	unmap_buffers(desc);
	detach_buffers(desc);

	kfree(desc);

	return 0;
}

static unsigned int delcore30m_job_poll(struct file *file, poll_table *wait)
{
	struct delcore30m_job_desc *desc = file->private_data;

	poll_wait(file, &desc->wait, wait);
	if (desc->job.status != DELCORE30M_JOB_IDLE)
		return 0;

	switch (desc->job.rc) {
	case DELCORE30M_JOB_SUCCESS:
		return POLLIN;
	case DELCORE30M_JOB_ERROR:
		return POLLERR;
	case DELCORE30M_JOB_CANCELLED:
		return POLLHUP;
	default:
		return -EINVAL;
	}
}

static const struct file_operations delcore30m_job_fops = {
	.release = delcore30m_job_release,
	.poll = delcore30m_job_poll
};

static int delcore30m_job_create(struct delcore30m_private_data *pdata,
				 void __user *arg)
{
	struct delcore30m_job_desc *job_desc;
	struct device *dev = pdata->dev;
	struct fd fd;
	struct delcore30m_resource_desc *res_desc;
	int ret, i;

	job_desc = kzalloc(sizeof(struct delcore30m_job_desc), GFP_KERNEL);
	if (!job_desc)
		return -ENOMEM;

	job_desc->pdata = pdata;

	ret = copy_from_user(&job_desc->job, arg,
			     sizeof(struct delcore30m_job));
	if (ret) {
		ret = -EACCES;
		goto err_desc;
	}

	fd = fdget(job_desc->job.cores_fd);
	if (!fd.file || fd.file->f_op != &delcore30m_resource_fops) {
		ret = -EBADFD;
		fdput(fd);
		goto err_desc;
	}

	res_desc = fd.file->private_data;
	job_desc->cores = res_desc->resource.mask;
	fdput(fd);

	fd = fdget(job_desc->job.sdmas_fd);
	if (fd.file && (fd.file->f_op == &delcore30m_resource_fops)) {
		res_desc = fd.file->private_data;
		job_desc->sdmas = res_desc->resource.mask;
	}
	fdput(fd);

	for_each_set_bit(i, &job_desc->cores, MAX_CORES)
		if (!pdata->fwready[i]) {
			ret = -EPERM;
			goto err_desc;
		}

	ret = anon_inode_getfd("delcorejob", &delcore30m_job_fops, job_desc, 0);
	if (ret < 0)
		goto err_desc;

	job_desc->job.fd = ret;
	job_desc->job.status = DELCORE30M_JOB_IDLE;
	init_waitqueue_head(&job_desc->wait);

	ret = copy_to_user(arg, &job_desc->job, sizeof(struct delcore30m_job));
	if (ret) {
		ret = -EFAULT;
		goto err_fd;
	}

	if (job_desc->job.inum > MAX_INPUTS) {
		dev_err(dev, "Number of input buffers is too big (%d)\n",
			job_desc->job.inum);
		ret = -EINVAL;
		goto err_fd;
	}

	if (job_desc->job.onum > MAX_OUTPUTS) {
		dev_err(dev, "Number of output buffers is too big (%d)\n",
			job_desc->job.onum);
		ret = -EINVAL;
		goto err_fd;
	}
	job_desc->num_bufs = job_desc->job.inum + job_desc->job.onum;

	ret = attach_buffers(job_desc);
	if (ret) {
		dev_err(dev, "Failed to attach buffers\n");
		goto err_fd;
	}

	ret = map_buffers(job_desc);
	if (ret) {
		dev_err(dev, "Failed to map buffers\n");
		goto detach_buffers;
	}

	if (job_desc->job.flags & DELCORE30M_PROFILE) {
		for_each_set_bit(i, &job_desc->cores, MAX_CORES) {
			job_desc->profilebuf[i].buf.size = SZ_32K;
			job_desc->profilebuf[i].vaddr = dma_alloc_coherent(
					job_desc->pdata->dev,
					job_desc->profilebuf[i].buf.size,
					&job_desc->profilebuf[i].paddr,
					GFP_KERNEL);
			if (!job_desc->profilebuf[i].vaddr) {
				dev_err(job_desc->pdata->dev,
					"Failed to allocate buffer for profile\n");
				ret = -ENOMEM;
				goto unmap_buffers;
			}
		}
	}

	return 0;
unmap_buffers:
	unmap_buffers(job_desc);
detach_buffers:
	detach_buffers(job_desc);
err_fd:
	put_unused_fd(job_desc->job.fd);
err_desc:
	kfree(job_desc);
	return ret;
}

static int delcore30m_job_enqueue(struct delcore30m_private_data *pdata,
				  void __user *arg)
{
	struct delcore30m_job job;
	struct delcore30m_job_desc *job_desc;
	unsigned long flags;
	struct fd fd;
	int ret;

	ret = copy_from_user(&job, arg, sizeof(struct delcore30m_job));
	if (ret)
		return -EACCES;

	fd = fdget(job.fd);
	if (!fd.file || fd.file->f_op != &delcore30m_job_fops) {
		ret = -EBADFD;
		goto done;
	}

	job_desc = fd.file->private_data;

	/* TODO: check that all buffers were unmapped */

	spin_lock_irqsave(&pdata->lock, flags);
	if (job_desc->job.status != DELCORE30M_JOB_IDLE) {
		spin_unlock_irqrestore(&pdata->lock, flags);
		ret = -EBUSY;
		goto done;
	}

	job_desc->job.status = DELCORE30M_JOB_ENQUEUED;
	list_add(&job_desc->list, &pdata->enqueued);
	if (list_is_singular(&pdata->enqueued))
		delcore30m_try_run(pdata);
	spin_unlock_irqrestore(&pdata->lock, flags);

done:
	fdput(fd);
	return ret;
}

static int delcore30m_job_status(void __user *arg)
{
	struct delcore30m_job job;
	struct delcore30m_job_desc *job_desc;
	struct fd fd;
	int ret;

	ret = copy_from_user(&job, arg, sizeof(struct delcore30m_job));
	if (ret)
		return -EACCES;

	fd = fdget(job.fd);
	if (!fd.file || fd.file->f_op != &delcore30m_job_fops) {
		ret = -EBADFD;
		goto done;
	}

	job_desc = fd.file->private_data;

	ret = copy_to_user(arg, &job_desc->job, sizeof(struct delcore30m_job));
	if (ret)
		ret = -EFAULT;

done:
	fdput(fd);
	return ret;
}

static int delcore30m_job_cancel(struct delcore30m_private_data *pdata,
				 void __user *arg)
{
	struct delcore30m_job job;
	struct delcore30m_job_desc *job_desc;
	struct fd fd;
	int ret;

	ret = copy_from_user(&job, arg, sizeof(struct delcore30m_job));
	if (ret)
		return -EACCES;

	fd = fdget(job.fd);
	if (!fd.file || fd.file->f_op != &delcore30m_job_fops) {
		ret = -EBADFD;
		goto done;
	}

	job_desc = fd.file->private_data;

	job_cancel(job_desc);
done:
	fdput(fd);
	return ret;
}

static struct sg_table *delcore30m_dmabuf_map(struct dma_buf_attachment *attach,
					      enum dma_data_direction dir)
{
	int rc;
	struct sg_table *sgt;

	/* I'm sure attach is not NULL and corresponds to my dmabuf. */
	struct delcore30m_buffer_desc *desc = attach->dmabuf->priv;

	/* TODO: dmabuf-specific lock */

	sgt = kmalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return ERR_PTR(-ENOMEM);

	switch (desc->buf.type) {
	case DELCORE30M_MEMORY_SYSTEM:
		dma_get_sgtable(attach->dev, sgt, desc->vaddr, desc->paddr,
				desc->buf.size);
		rc = dma_map_sg(attach->dev, sgt->sgl, sgt->orig_nents, dir);
		if (rc <= 0) {
			pr_err("Failed to map DMA buffer\n");
			goto free_table;
		}
		break;
	case DELCORE30M_MEMORY_XYRAM:
		rc = sg_alloc_table(sgt, 1, GFP_KERNEL);
		if (rc)
			goto free_sgt;
		sgt->sgl[0].length = desc->buf.size;
		sg_dma_len(sgt->sgl) = desc->buf.size;
		sgt->sgl[0].dma_address = desc->paddr;
		break;
	default:
		WARN_ON(1);
		rc = -EINVAL;
		goto free_sgt;
	}

	return sgt;

free_table:
	sg_free_table(sgt);
free_sgt:
	kfree(sgt);
	return ERR_PTR(rc);
}

static void delcore30m_dmabuf_unmap(struct dma_buf_attachment *attach,
				    struct sg_table *sgt,
				    enum dma_data_direction dir)
{
	struct delcore30m_buffer_desc *desc = attach->dmabuf->priv;

	if (desc->buf.type == DELCORE30M_MEMORY_SYSTEM)
		dma_unmap_sg(attach->dev, sgt->sgl, sgt->orig_nents, dir);
	sg_free_table(sgt);
	kfree(sgt);
}

static void *delcore30m_dmabuf_kmap_atomic(struct dma_buf *dmabuf,
					   unsigned long page_num)
{
	struct delcore30m_buffer_desc *buf_desc = dmabuf->priv;

	if (WARN_ON(page_num * PAGE_SIZE >= buf_desc->buf.size))
		return NULL;

	return buf_desc->vaddr ? buf_desc->vaddr + PAGE_SIZE * page_num : NULL;
}

static void *delcore30m_dmabuf_vmap(struct dma_buf *dmabuf)
{
	/* TODO: virtual mapping */
	return delcore30m_dmabuf_kmap_atomic(dmabuf, 0);
}

static int delcore30m_dmabuf_mmap(struct dma_buf *dmabuf,
				  struct vm_area_struct *vma)
{
	struct delcore30m_buffer_desc *buf_desc = dmabuf->priv;
	struct delcore30m_private_data *pdata = buf_desc->pdata;
	int ret;

	if (vma->vm_pgoff) {
		dev_err(pdata->dev, "Nonzero vm_pgoff is not supported\n");
		return -EINVAL;
	}

	switch (buf_desc->buf.type) {
	case DELCORE30M_MEMORY_XYRAM:
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		ret = io_remap_pfn_range(vma, vma->vm_start,
					 buf_desc->paddr >> PAGE_SHIFT,
					 vma->vm_end - vma->vm_start,
					 vma->vm_page_prot);
		break;
	case DELCORE30M_MEMORY_SYSTEM:
		ret = dma_mmap_coherent(pdata->dev, vma,
					buf_desc->vaddr, buf_desc->paddr,
					buf_desc->buf.size);
		break;
	default:
		ret = -EINVAL;
	}

	if (ret)
		dev_err(pdata->dev, "Failed to map memory\n");

	return ret;
}

static void delcore30m_dmabuf_release(struct dma_buf *dmabuf)
{
	struct delcore30m_buffer_desc *buf_desc = dmabuf->priv;

	delcore30m_mem_free(buf_desc);
	devm_kfree(buf_desc->pdata->dev, buf_desc);
}

static const struct dma_buf_ops delcore30m_dmabuf_ops = {
	.map_dma_buf = delcore30m_dmabuf_map,
	.unmap_dma_buf = delcore30m_dmabuf_unmap,
	.kmap = delcore30m_dmabuf_kmap_atomic,
	.kmap_atomic = delcore30m_dmabuf_kmap_atomic,
	.vmap = delcore30m_dmabuf_vmap,
	.mmap = delcore30m_dmabuf_mmap,
	.release = delcore30m_dmabuf_release
};

static int delcore30m_buf_alloc(struct delcore30m_private_data *pdata,
				void __user *arg)
{
	struct dma_buf *dmabuf;
	struct delcore30m_buffer_desc *buf_desc;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	int ret;

	buf_desc = devm_kzalloc(pdata->dev,
				sizeof(struct delcore30m_buffer_desc),
				GFP_KERNEL);
	if (!buf_desc)
		return -ENOMEM;

	ret = copy_from_user(&buf_desc->buf, arg,
			     sizeof(struct delcore30m_buffer));
	if (ret) {
		ret = -EACCES;
		goto err_free_buf_desc;
	}

	buf_desc->pdata = pdata;

	ret = delcore30m_mem_alloc(buf_desc);
	if (ret)
		goto err_free_buf_desc;

	exp_info.ops = &delcore30m_dmabuf_ops;
	exp_info.size = buf_desc->buf.size;
	exp_info.flags = O_RDWR;
	exp_info.priv = buf_desc;

	dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(dmabuf)) {
		ret = PTR_ERR(dmabuf);
		goto err_free_mem;
	}

	buf_desc->dmabuf = dmabuf;
	buf_desc->buf.fd = dma_buf_fd(dmabuf, 0);

	if (!buf_desc->buf.fd) {
		/* This will call delcore30m_dmabuf_release() */
		dma_buf_put(dmabuf);
		return -EINVAL;
	}

	ret = copy_to_user(arg, &buf_desc->buf,
			   sizeof(struct delcore30m_buffer));
	if (ret) {
		dma_buf_put(dmabuf);
		return -EFAULT;
	}

	return 0;
err_free_mem:
	delcore30m_mem_free(buf_desc);
err_free_buf_desc:
	devm_kfree(pdata->dev, buf_desc);
	return ret;
}

static int delcore30m_spinlock_try(struct delcore30m_private_data *pdata,
				   u32 timeout)
{
	u8 spinlock_value;

	if (readl_poll_timeout_atomic(pdata->spinlock + SPINLOCK_REG_OFFSET,
				      spinlock_value, (spinlock_value == 0), 5,
				      1000))
		return -EBUSY;
	return 0;
}

static void delcore30m_spinlock_unlock(struct delcore30m_private_data *pdata)
{
	writeb(0, pdata->spinlock + SPINLOCK_REG_OFFSET);
}

static int resource_release(struct delcore30m_resource_desc *res)
{
	struct delcore30m_private_data *pdata = res->pdata;
	int i, j, rc;
	u32 qmaskr0_val, dbg_status, chn_status;

	spin_lock(&pdata->reslock);

	switch (res->resource.type) {
	case DELCORE30M_CORE:
		pdata->cores &= ~res->resource.mask;
		for_each_set_bit(i, &res->resource.mask, MAX_CORES)
			pdata->fwready[i] = false;
		break;
	case DELCORE30M_SDMA:
		pdata->sdmas &= ~res->resource.mask;
		for (j = 0; j < MAX_CORES; ++j) {
			qmaskr0_val = delcore30m_readl(pdata, j,
						       DELCORE30M_QMASKR0);
			qmaskr0_val &= ~(res->resource.mask << 8);
			delcore30m_writel(pdata, j, DELCORE30M_QMASKR0,
					  qmaskr0_val);
		}

		for_each_set_bit(i, &res->resource.mask, MAX_SDMA_CHANNELS) {
			regmap_read(pdata->sdma, CHANNEL_STATUS(i),
				    &chn_status);
			if ((chn_status & 0xF) == 0)
				continue;

			rc = delcore30m_spinlock_try(pdata, 1000);
			if (rc)
				return rc;

			do {
				regmap_read(pdata->sdma, DBGSTATUS,
					    &dbg_status);
			} while (dbg_status & 1);

			regmap_write(pdata->sdma, DBGINST0,
				     (SDMA_DMAKILL << 16) | (i << 8) | 1);
			regmap_write(pdata->sdma, DBGINST1, 0);
			regmap_write(pdata->sdma, DBGCMD, 0);

			delcore30m_spinlock_unlock(pdata);
		}
		break;
	default:
		spin_unlock(&pdata->reslock);
		return -EINVAL;
	}

	spin_unlock(&pdata->reslock);
	return 0;
}

static int delcore30m_resource_release(struct inode *inode, struct file *file)
{
	struct delcore30m_resource_desc *res = file->private_data;
	int rc;

	rc = resource_release(res);
	kfree(res);

	return rc;
}

static int delcore30m_resource_mmap(struct file *file,
				    struct vm_area_struct *vma)
{
	struct delcore30m_resource_desc *res_desc = file->private_data;
	struct delcore30m_private_data *pdata = res_desc->pdata;
	unsigned long size;
	int ret, core;

	if (res_desc->resource.type == DELCORE30M_SDMA)
		return -EINVAL;
	core = vma->vm_pgoff;
	if (core >= MAX_CORES || ((res_desc->resource.mask >> core) & 1) == 0)
		return -EINVAL;
	size = vma->vm_end - vma->vm_start;
	if (size > pdata->hw.core_pram_size) {
		dev_err(pdata->dev, "Failed to map PRAM\n");
		return -EINVAL;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	ret = io_remap_pfn_range(vma,
				 vma->vm_start,
				 PFN_DOWN(pdata->pram[core].pram_res->start),
				 size,
				 vma->vm_page_prot);
	if (ret) {
		dev_err(pdata->dev, "Failed to map PRAM\n");
		return -EIO;
	}
	pdata->pram[core].firmware_size = size;
	/* TODO: Set flag in overridden close callback in vm_operations */
	pdata->fwready[core] = true;
	return 0;
}

static const struct file_operations delcore30m_resource_fops = {
	.release = delcore30m_resource_release,
	.mmap = delcore30m_resource_mmap
};

static int delcore30m_resource_request(struct delcore30m_private_data *pdata,
				       void __user *arg)
{
	struct delcore30m_resource_desc *res;
	unsigned long *array;
	int rc, i, idx, max;

	res = kzalloc(sizeof(struct delcore30m_resource_desc), GFP_KERNEL);
	if (!res)
		return -ENOMEM;
	res->pdata = pdata;

	rc = copy_from_user(&res->resource, arg,
			    sizeof(struct delcore30m_resource));
	if (rc) {
		rc = -EACCES;
		goto err_resource_request;
	}

	switch (res->resource.type) {
	case DELCORE30M_CORE:
		array = &pdata->cores;
		max = MAX_CORES;
		break;
	case DELCORE30M_SDMA:
		array = &pdata->sdmas;

		max = MAX_SDMA_CHANNELS;
		break;
	default:
		rc = -EINVAL;
		goto err_resource_request;
	}

	if (res->resource.num <= 0 || res->resource.num > max) {
		rc = -EINVAL;
		goto err_resource_request;
	}

	rc = anon_inode_getfd("delcoreresource",
			       &delcore30m_resource_fops,
			       res,
			       O_RDWR);
	if (rc < 0)
		goto err_resource_request;

	res->resource.fd = rc;

	spin_lock(&pdata->reslock);

	res->resource.mask = 0;
	for (i = 0; i < res->resource.num; ++i) {
		idx = find_first_zero_bit(array, max);
		if (idx == max) {
			spin_unlock(&pdata->reslock);
			rc = -EBUSY;
			goto resource_release;
		}

		set_bit(idx, array);
		set_bit(idx, &res->resource.mask);
	}

	spin_unlock(&pdata->reslock);

	rc = copy_to_user(arg, &res->resource,
			  sizeof(struct delcore30m_resource));
	if (rc < 0)
		goto resource_release;

	return 0;

resource_release:
	resource_release(res);
err_resource_request:
	kfree(res);
	return rc;
}

static int delcore30m_sys_info(struct delcore30m_private_data *pdata,
			       void __user *arg)
{
	int rc;

	rc = copy_to_user(arg, &pdata->hw, sizeof(struct delcore30m_hardware));
	if (rc)
		return -EACCES;

	return 0;
}

static void sdma_command_add(struct sdma_program_buf *buf, u64 command,
			     size_t commandlen)
{
	/* TODO: Remove commandlen arg from this function */
	while (commandlen-- && buf->pos < buf->end) {
		*buf->pos++ = command & 0xFF;
		command >>= 8;
	}
}

static struct buf_info *delcore30m_job_get_bufinfo(
		struct delcore30m_job_desc *desc, int fd)
{
	int i;

	for (i = 0; i < desc->job.inum; ++i)
		if (desc->job.input[i] == fd)
			return &desc->bufs[i];

	for (i = 0; i < desc->job.onum; ++i)
		if (desc->job.output[i] == fd)
			return &desc->bufs[desc->job.inum + i];

	return NULL;
}

static void sdma_addr_add(struct sdma_program_buf *program_buf, u8 cmd,
			  u32 value)
{
	int i;

	if (value == 0)
		return;

	for (i = 0; i < value / U16_MAX; ++i)
		sdma_command_add(program_buf, cmd + (U16_MAX << 8), 3);

	value %= U16_MAX;
	if (value)
		sdma_command_add(program_buf, cmd + (value << 8), 3);
}

static void sdma_program_tile(struct sdma_program_buf *program_buf,
			      struct sdma_descriptor sd,
			      enum sdma_channel_type type, u8 channel)
{
	char *loop_start;
	ptrdiff_t loop_length;
	const u32 acnt = sd.asize / SDMA_BURST_SIZE(sd.ccr);
	u32 i, trans16_pack = (acnt / 16);
	const u32 trans_pack = (acnt % 16);

	sdma_command_add(program_buf, SDMA_DMAMOVE_SAR, 2);
	if (type == SDMA_CHANNEL_INPUT)
		sdma_command_add(program_buf, sd.a0e, 4);
	else
		sdma_command_add(program_buf, sd.a0i, 4);

	sdma_command_add(program_buf, SDMA_DMAMOVE_DAR, 2);
	if (type == SDMA_CHANNEL_INPUT)
		sdma_command_add(program_buf, sd.a0i, 4);
	else
		sdma_command_add(program_buf, sd.a0e, 4);

	if (sd.type == SDMA_DESCRIPTOR_E1I1 ||
	    sd.type == SDMA_DESCRIPTOR_E1I0) {
		sdma_command_add(program_buf,
				 SDMA_DMAWFE +
					((MAX_SDMA_CHANNELS + channel) << 11),
				 2);
	}
	sdma_command_add(program_buf, SDMA_DMALP(0) + ((sd.bcnt-1) << 8), 2);

	loop_start = program_buf->pos;
	if (trans16_pack) {
		sdma_command_add(program_buf, SDMA_DMAMOVE_CCR, 2);
		sdma_command_add(program_buf, sd.ccr | (15 << 18) | (15 << 4),
				 4);
	}

	for (i = 0; i < trans16_pack / 256; ++i) {
		sdma_command_add(program_buf, SDMA_DMALP(1) + (255 << 8), 2);
		sdma_command_add(program_buf, SDMA_DMALD, 1);
		sdma_command_add(program_buf, SDMA_DMAST, 1);
		sdma_command_add(program_buf, SDMA_DMALPEND(1) + (2 << 8), 2);
	}

	trans16_pack = trans16_pack % 256;
	if (trans16_pack) {
		sdma_command_add(program_buf,
				 SDMA_DMALP(1) + ((trans16_pack - 1) << 8), 2);
		sdma_command_add(program_buf, SDMA_DMALD, 1);
		sdma_command_add(program_buf, SDMA_DMAST, 1);
		sdma_command_add(program_buf, SDMA_DMALPEND(1) + (2 << 8), 2);
	}

	if (trans_pack) {
		sdma_command_add(program_buf, SDMA_DMAMOVE_CCR, 2);
		sdma_command_add(program_buf,
			    sd.ccr | (trans_pack-1) << 18 | (trans_pack-1) << 4,
			    4);

		sdma_command_add(program_buf, SDMA_DMALD, 1);
		sdma_command_add(program_buf, SDMA_DMAST, 1);
	}

	if (type == SDMA_CHANNEL_INPUT)
		sdma_addr_add(program_buf, SDMA_DMAADDH_SAR,
			      sd.astride - sd.asize);
	else
		sdma_addr_add(program_buf, SDMA_DMAADDH_DAR,
			      sd.astride - sd.asize);



	/* FIXME: Using barrier SDMA_DMARMB or/and SDMA_DMAWMB? */

	loop_length = program_buf->pos - loop_start;
	sdma_command_add(program_buf, SDMA_DMALPEND(0) + (loop_length << 8), 2);

	if ((sd.type == SDMA_DESCRIPTOR_E0I1) ||
	    (sd.type == SDMA_DESCRIPTOR_E1I1))
		sdma_command_add(program_buf, SDMA_DMASEV + (channel << 11), 2);
}

static int sdma_program(struct delcore30m_dmachain dmachain,
			dma_addr_t *code_address)
{
	int rc = 0, odd = 0;
	int chainsize;
	struct sdma_descriptor *sd, temp_sd;
	struct buf_info *external, *internal[2], *chain, *codebuf;
	void *sdmaptr;
	struct file *jobfile;
	struct delcore30m_job_desc *jobdesc;
	u8 *code;
	struct sdma_program_buf program_buf;

	jobfile = fget(dmachain.job);
	if (!jobfile)
		return -EBADF;

	if (jobfile->f_op != &delcore30m_job_fops) {
		fput(jobfile);
		return -EBADF;
	}

	jobdesc = jobfile->private_data;

	external = delcore30m_job_get_bufinfo(jobdesc, dmachain.external);
	internal[0] = delcore30m_job_get_bufinfo(jobdesc, dmachain.internal[0]);
	internal[1] = delcore30m_job_get_bufinfo(jobdesc, dmachain.internal[1]);
	chain = delcore30m_job_get_bufinfo(jobdesc, dmachain.chain);
	codebuf = delcore30m_job_get_bufinfo(jobdesc, dmachain.codebuf);

	chainsize = chain->attach->dmabuf->size;
	sd = sdmaptr = dma_buf_vmap(chain->attach->dmabuf);
	code = dma_buf_vmap(codebuf->attach->dmabuf);
	program_buf.pos = program_buf.start = code;
	program_buf.end = program_buf.start + codebuf->attach->dmabuf->size;

	do {
		void *next = sdmaptr + sd->a_init;

		if (next - sdmaptr > chainsize) {
			rc = -EFAULT;
			break;
		}

		if ((sd->type == SDMA_DESCRIPTOR_E0I1) ||
		    (sd->type == SDMA_DESCRIPTOR_E0I0))
			odd ^= 1;

		memcpy(&temp_sd, sd, sizeof(struct sdma_descriptor));

		temp_sd.a0e += sg_dma_address(external->sgt->sgl);
		temp_sd.a0i += sg_dma_address(internal[odd]->sgt->sgl);

		sdma_program_tile(&program_buf, temp_sd,
				  dmachain.channel.type, dmachain.channel.num);

		odd ^= 1;
		if (sd->a_init)
			sd = next;
		else
			break;
	} while (true);

	dma_buf_vunmap(chain->attach->dmabuf, sdmaptr);

	sdma_command_add(&program_buf, SDMA_DMAWMB, 1);
	sdma_command_add(&program_buf, SDMA_DMAEND, 1);

	dma_buf_vunmap(codebuf->attach->dmabuf, code);

	*code_address = sg_dma_address(codebuf->sgt->sgl);

	fput(jobfile);
	return rc;
}

static int delcore30m_dmachain_setup(struct delcore30m_private_data *pdata,
				     void __user *arg)
{
	int rc;
	struct delcore30m_dmachain dmachain;
	dma_addr_t code_addr;
	u32 channel_status, inten_value;
	u32 dbg_status, qmaskr0_val;
	u8 core_id;

	rc = copy_from_user(&dmachain, arg,
			    sizeof(struct delcore30m_dmachain));
	if (rc)
		return -EACCES;

	if (dmachain.channel.num >= MAX_SDMA_CHANNELS)
		return -EINVAL;

	regmap_read(pdata->sdma, CHANNEL_STATUS(dmachain.channel.num),
		    &channel_status);
	if (channel_status & 0xF)
		return -EBUSY;

	rc = sdma_program(dmachain, &code_addr);
	if (rc)
		return rc;
	core_id = dmachain.core;

	/* TODO: Move DSP registers setup to try_run() */

	/* FIXME: interrupt handler address */
	delcore30m_writel(pdata, core_id, DELCORE30M_INVAR,
			  cpu_to_delcore30m(phys_to_xyram(0x0C)));

	regmap_read(pdata->sdma, INTEN, &inten_value);
	inten_value |= 1 << dmachain.channel.num;
	regmap_write(pdata->sdma, INTEN, inten_value);

	delcore30m_writel(pdata, core_id, DELCORE30M_IMASKR, (1 << 30));

	qmaskr0_val = delcore30m_readl(pdata, core_id, DELCORE30M_QMASKR0);
	qmaskr0_val |= 1 << (8 + dmachain.channel.num);
	delcore30m_writel(pdata, core_id, DELCORE30M_QMASKR0, qmaskr0_val);

	rc = delcore30m_spinlock_try(pdata, 1000);
	if (rc)
		return rc;

	do {
		regmap_read(pdata->sdma, DBGSTATUS, &dbg_status);
	} while (dbg_status & 1);

	regmap_write(pdata->sdma, DBGINST0,
		     (0xA0 << 16) | (dmachain.channel.num << 8) |
		     (dmachain.channel.num << 24));
	regmap_write(pdata->sdma, DBGINST1, code_addr);
	regmap_write(pdata->sdma, DBGCMD, 0);

	delcore30m_spinlock_unlock(pdata);
	return 0;
}

static int delcore30m_get_caps(struct delcore30m_private_data *pdata,
			       void __user *arg)
{
	struct elcore_caps elcore_caps;
	int ret;

	strcpy(elcore_caps.drvname, pdata->dev->driver->name);
	elcore_caps.hw_id = delcore30m_readw(pdata, 0, DELCORE30M_IDR);

	ret = copy_to_user(arg, &elcore_caps, sizeof(struct elcore_caps));
	if (ret)
		return ret;

	return 0;
}

static long delcore30m_ioctl(struct file *file, unsigned int cmd,
			     unsigned long arg)
{
	struct delcore30m_private_data *pdata = file->private_data;
	void __user *const uptr = (void __user *)arg;

	switch (cmd) {
	case ELCIOC_JOB_CREATE:
		return delcore30m_job_create(pdata, uptr);
	case ELCIOC_JOB_ENQUEUE:
		return delcore30m_job_enqueue(pdata, uptr);
	case ELCIOC_BUF_ALLOC:
		return delcore30m_buf_alloc(pdata, uptr);
	case ELCIOC_JOB_STATUS:
		return delcore30m_job_status(uptr);
	case ELCIOC_JOB_CANCEL:
		return delcore30m_job_cancel(pdata, uptr);
	case ELCIOC_RESOURCE_REQUEST:
		return delcore30m_resource_request(pdata, uptr);
	case ELCIOC_SYS_INFO:
		return delcore30m_sys_info(pdata, uptr);
	case ELCIOC_DMACHAIN_SETUP:
		return delcore30m_dmachain_setup(pdata, uptr);
	case ELCIOC_GET_CAPS:
		return delcore30m_get_caps(pdata, uptr);
	}

	dev_err(pdata->dev, "%d ioctl is not supported\n", cmd);
	return -ENOTTY;
}

static int delcore30m_open(struct inode *inode, struct file *file)
{
	struct delcore30m_private_data *pdata;

	pdata = container_of(inode->i_cdev,
			     struct delcore30m_private_data, cdev);
	file->private_data = pdata;

	atomic_inc(&pdata->count);

	return 0;
}

static int delcore30m_release(struct inode *inode, struct file *file)
{
	struct delcore30m_private_data *pdata = file->private_data;
	int i;

	if (atomic_dec_and_test(&pdata->count))
		for (i = 0; i < MAX_CORES; ++i)
			pdata->fwready[i] = false;
	return 0;
}

static const struct file_operations delcore30m_fops = {
	.owner = THIS_MODULE,
	.open = delcore30m_open,
	.release = delcore30m_release,
	.unlocked_ioctl = delcore30m_ioctl
};

static void search_pair_tags(struct device *dev, struct timestamp *timestamps,
			     u32 tag, size_t i, size_t nstamps)
{
	int j;
	u32 diff, calls = 0, max = 0, min = U32_MAX;
	u64 sum = 0;

	while (i < nstamps) {
		/* Find second tag of a pair */
		for (j = i + 1; j < nstamps && timestamps[j].tag != tag; ++j)
			continue;

		if (j == nstamps) {
			dev_warn(dev, "Unpaired profile tag %x\n", tag);
			break;
		}

		diff = timestamps[j].time - timestamps[i].time;
		max = max(diff, max);
		min = min(diff, min);
		sum += diff;
		calls++;

		timestamps[i].tag = 0;
		timestamps[j].tag = 0;

		/* Find first tag of the next pair */
		for (i = j + 1; i < nstamps && timestamps[i].tag != tag; ++i)
			continue;
	}

	if (calls)
		dev_info(dev, "Tag %x: calls = %u, min/max/avg = %u/%u/%llu\n",
			 tag, calls, min, max, div_u64(sum, calls));
}

static void parse_profilebuf(struct delcore30m_job_desc *desc)
{
	struct timestamp *timestamps;
	int core, i;
	size_t size, nstamps;

	for_each_set_bit(core, &desc->cores, MAX_CORES) {
		size = delcore30m_to_cpu(delcore30m_readl(desc->pdata, core,
							  DELCORE30M_A5)) -
				desc->profilebuf[core].paddr;

		if (size == 0)
			continue;

		timestamps = kmalloc(size, GFP_KERNEL);
		memcpy(timestamps, desc->profilebuf[core].vaddr, size);

		nstamps = size / sizeof(struct timestamp);

		for (i = 0; i < nstamps; ++i) {
			if (timestamps[i].tag == 0)
				continue;

			search_pair_tags(desc->pdata->dev, timestamps,
					 timestamps[i].tag, i, nstamps);
		}
		kfree(timestamps);
	}
}

static enum delcore30m_job_rc delcore30m_job_rc(struct delcore30m_private_data
						*pdata,
						const unsigned long mask)
{
	int i;

	for_each_set_bit(i, &mask, MAX_CORES) {
		u32 val = delcore30m_readw(pdata, i, DELCORE30M_DCSR);

		if (val & (DELCORE30M_DCSR_PI | DELCORE30M_DCSR_SE |
			   DELCORE30M_DCSR_BRK))
			return DELCORE30M_JOB_ERROR;
	}

	return DELCORE30M_JOB_SUCCESS;
}

static irqreturn_t delcore30m_interrupt(int irq, void *arg)
{
	struct delcore30m_private_data *pdata = arg;
	struct delcore30m_job_desc *desc, *next;
	u32 stopped_cores;
	u32 val, maskr_dsp;
	int i;

	/* Check STOP interrupt by DSP0 and DSP1 */
	val = delcore30m_readl_cmn(pdata, DELCORE30M_QSTR_DSP);

	stopped_cores = 0;
	for (i = 0; i < MAX_CORES; ++i)
		if (val & DELCORE30M_QSTR_CORE_MASK(i))
			stopped_cores |= BIT(i);

	if (!stopped_cores)
		return IRQ_NONE;

	maskr_dsp = delcore30m_readl_cmn(pdata, DELCORE30M_MASKR_DSP);

	maskr_dsp &= ~(val & DELCORE30M_QSTR_MASK);
	delcore30m_writel_cmn(pdata, DELCORE30M_MASKR_DSP, maskr_dsp);
	spin_lock(&pdata->lock);

	list_for_each_entry_safe(desc, next, &pdata->running, list) {
		if ((desc->cores & stopped_cores) == desc->cores) {
			list_del(&desc->list);
			desc->job.rc = delcore30m_job_rc(pdata, desc->cores);
			desc->job.status = DELCORE30M_JOB_IDLE;

			if (desc->job.flags & DELCORE30M_PROFILE)
				parse_profilebuf(desc);

			reset_cores(desc);

			val = delcore30m_readl_cmn(pdata, DELCORE30M_CSR_DSP);
			val &= ~1;
			delcore30m_writel_cmn(pdata, DELCORE30M_CSR_DSP, val);
			wake_up_interruptible(&desc->wait);
		}
	}

	if (!list_empty(&pdata->enqueued))
		delcore30m_try_run(pdata);

	spin_unlock(&pdata->lock);

	return IRQ_HANDLED;
}

static int delcore30m_clock_init(struct delcore30m_private_data *data,
				 struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int i, ret;

	data->clock_count = of_clk_get_parent_count(np);
	if (!data->clock_count)
		return 1;

	data->clocks = devm_kcalloc(&pdev->dev, data->clock_count,
				    sizeof(struct clk *), GFP_KERNEL);
	if (!data->clocks)
		return -ENOMEM;

	for (i = 0; i < data->clock_count; ++i) {
		data->clocks[i] = of_clk_get(np, i);
		if (data->clocks[i]) {
			ret = clk_prepare_enable(data->clocks[i]);
			if (ret) {
				dev_err(&pdev->dev, "clock %d error: %ld\n",
					i, PTR_ERR(data->clocks[i]));
				clk_put(data->clocks[i]);
				data->clocks[i] = NULL;
				return ret;
			}
		}
	}

	return 0;
}

static void delcore30m_clock_destroy(struct delcore30m_private_data *data)
{
	int i;

	if (!data->clocks)
		return;

	for (i = 0; i < data->clock_count; ++i) {
		if (data->clocks[i]) {
			clk_disable_unprepare(data->clocks[i]);
			clk_put(data->clocks[i]);
		}
	}

	kfree(data->clocks[i]);
}

static int delcore30m_hwinfo_init(struct delcore30m_private_data *pdata,
				  struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret, ncores;

	ret = of_property_read_u32(np, "ncores", &ncores);
	if (ret) {
		dev_err(&pdev->dev, "Failed to read \"ncores\" property\n");
		return ret;
	}

	pdata->hw.ncores = ncores;
	pdata->hw.core_pram_size = BANK_SIZE;
	pdata->hw.xyram_size = ncores * (4 * BANK_SIZE);

	return 0;
}

static int delcore30m_probe(struct platform_device *pdev)
{
	struct delcore30m_private_data *pdata;
	struct resource *res;
	struct device *dev;
	char device_name[50], dsp_pram_name[50];
	static int device_count;
	int irq, ret, i;
	dma_cap_mask_t mask;
	struct dma_chan *chan;

	pdata = devm_kzalloc(&pdev->dev, sizeof(struct delcore30m_private_data),
			     GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->dev = &pdev->dev;

	ret = delcore30m_hwinfo_init(pdata, pdev);
	if (ret)
		return ret;

	INIT_LIST_HEAD(&pdata->enqueued);
	INIT_LIST_HEAD(&pdata->running);
	spin_lock_init(&pdata->lock);
	spin_lock_init(&pdata->reslock);

	irq = platform_get_irq(pdev, 0);
	ret = devm_request_irq(&pdev->dev, irq, delcore30m_interrupt,
			       IRQF_SHARED, pdev->name, pdata);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request IRQ\n");
		return -ENOENT;
	}

	for (i = 0; i < MAX_CORES; ++i) {
		sprintf(dsp_pram_name, "dsp%d_pram", i);
		pdata->pram[i].pram_res = platform_get_resource_byname(pdev,
				IORESOURCE_MEM, dsp_pram_name);
		if (!pdata->pram[i].pram_res) {
			dev_err(&pdev->dev, "Failed to get PRAM%d resource\n",
				i);
			return -ENOENT;
		}

		pdata->xyram_pool[i] = of_gen_pool_get(pdev->dev.of_node,
						       "xyram", i);
		if (!pdata->xyram_pool[i]) {
			dev_err(&pdev->dev, "Failed to get XYRAM%d pool\n", i);
			return -ENOENT;
		}
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dsp0_regs");
	if (!res) {
		dev_err(&pdev->dev, "Failed to get DSP0 REG memory resource\n");
		return -ENOENT;
	}
	pdata->dsp_regs[0] = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pdata->dsp_regs[0]))
		return PTR_ERR(pdata->dsp_regs[0]);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dsp1_regs");
	if (!res) {
		dev_err(&pdev->dev, "Failed to get DSP1 REG memory resource\n");
		return -ENOENT;
	}

	pdata->dsp_regs[1] = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pdata->dsp_regs[1]))
		return PTR_ERR(pdata->dsp_regs[1]);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cmn_regs");
	if (!res) {
		dev_err(&pdev->dev, "Failed to get memory resource\n");
		return -ENOENT;
	}

	pdata->cmn_regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pdata->cmn_regs))
		return PTR_ERR(pdata->cmn_regs);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "spinlock");
	if (!res) {
		dev_err(&pdev->dev, "Failed to get spinlock resource\n");
		return -ENOENT;
	}

	pdata->spinlock = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pdata->spinlock))
		return PTR_ERR(pdata->spinlock);

	pdata->sdma = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						      "sdma");
	if (IS_ERR(pdata->sdma)) {
		dev_err(&pdev->dev, "Failed to get SDMA regmap\n");
		return -ENOENT;
	}

	ret = alloc_chrdev_region(&pdata->dev_num, 0, 1, "delcore");
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Failed to allocate region for delcore chrdev\n");
		return ret;
	}

	/* FIXME: For each bind/unbind driver will be new device */
	sprintf(device_name, "elcore%d", device_count++);
	dev = device_create(class, NULL, pdata->dev_num, NULL, device_name);
	if (IS_ERR(dev)) {
		dev_err(&pdev->dev, "Failed to create elcore device\n");
		ret = PTR_ERR(dev);
		goto err_unregister_chrdev_region;
	}

	cdev_init(&pdata->cdev, &delcore30m_fops);
	ret = cdev_add(&pdata->cdev, pdata->dev_num, 1);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to add delcore cdev\n");
		goto err_destroy_device;
	}

	ret = delcore30m_clock_init(pdata, pdev);
	if (ret)
		goto err_destroy_clock;

	platform_set_drvdata(pdev, pdata);

	/* TODO: proper power management */

	for (i = 0; i < MAX_CORES; ++i) {
		struct delcore30m_stack *stack = &pdata->stack[i];

		stack->vaddr = gen_pool_dma_alloc(pdata->xyram_pool[i],
						  STACK_SIZE,
						  &stack->paddr);
		if (!stack->vaddr) {
			dev_err(pdata->dev,
				"Failed to allocate stack for DSP %d\n", i);
			ret = -ENOMEM;
			goto free_stack;
		}
	}

	/* TODO: This code can find channel of another DMA controller */
	dma_cap_zero(mask);
	dma_cap_set(0, mask);
	chan = dma_request_channel(mask, NULL, NULL);
	if (chan) {
		ret = -EACCES;
		dma_release_channel(chan);
		dev_err(pdata->dev, "pl330 must be unloaded");
		goto free_stack;
	}

	return 0;

free_stack:
	while (--i >= 0)
		gen_pool_free(pdata->xyram_pool[i],
			      (unsigned long) pdata->stack[i].vaddr,
			      STACK_SIZE);
err_destroy_clock:
	delcore30m_clock_destroy(pdata);
err_destroy_device:
	device_destroy(class, pdata->dev_num);
err_unregister_chrdev_region:
	unregister_chrdev_region(pdata->dev_num, 1);
	return ret;
}

static int delcore30m_remove(struct platform_device *pdev)
{
	struct delcore30m_private_data *pdata = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < MAX_CORES; ++i)
		if (pdata->stack[i].vaddr)
			gen_pool_free(pdata->xyram_pool[i],
				      (unsigned long) pdata->stack[i].vaddr,
				      STACK_SIZE);

	delcore30m_clock_destroy(pdata);
	device_destroy(class, pdata->dev_num);
	unregister_chrdev_region(pdata->dev_num, 1);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id delcore30m_dt_ids[] = {
	{ .compatible = "elvees,delcore30m" },
	{}
};
MODULE_DEVICE_TABLE(of, delcore30m_dt_ids);
#endif

static struct platform_driver delcore30m_driver = {
	.driver = {
		.name = "delcore30m",
		.of_match_table = of_match_ptr(delcore30m_dt_ids),
	},
	.probe = delcore30m_probe,
	.remove = delcore30m_remove,
};

static int __init delcore30m_init(void)
{
	int ret = 0;

	class = class_create(THIS_MODULE, "delcore");
	if (IS_ERR(class)) {
		ret = PTR_ERR(class);
		goto class_err;
	}

	platform_driver_register(&delcore30m_driver);
class_err:
	return ret;
}

static void __exit delcore30m_exit(void)
{
	platform_driver_unregister(&delcore30m_driver);
	class_destroy(class);
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ELVEES DELcore-30M driver");
MODULE_AUTHOR("Georgy Macharadze <gmacharadze@elvees.com>");

module_init(delcore30m_init);
module_exit(delcore30m_exit);
