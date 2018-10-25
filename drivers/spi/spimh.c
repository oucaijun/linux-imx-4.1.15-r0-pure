/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <linux/uaccess.h>

#define FPGA_CS 0
/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
#define SPIDEV_MAJOR			153	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */


#define MDEBUG  (printk("XXX MDEBUG %s (%d) <%s> ",__FILE__,__LINE__,__FUNCTION__),printk)
#define MWARN  (printk("XXX MWARN %s (%d) <%s>" ,__FILE__,__LINE__,__FUNCTION__),printk)

static DECLARE_BITMAP(minors, N_SPI_MINORS);


/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *	is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY | SPI_TX_DUAL \
				| SPI_TX_QUAD | SPI_RX_DUAL | SPI_RX_QUAD)

struct spidev_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	/* TX/RX buffers are NULL unless this device is open (users > 0) */
	struct mutex		buf_lock;
	unsigned		users;
	u8			*tx_buffer;
	u8			*rx_buffer;
	u32			speed_hz;
};

//adc spi dev info
struct  fpga_spidev_info
{
    struct spidev_data *spidev;
    int32_t allocated;
    int32_t ok;
};

static struct  fpga_spidev_info  fpga_spidev={NULL, 0, 0};


static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

/*-------------------------------------------------------------------------*/

/*
 * We can't use the standard synchronous wrappers for file I/O; we
 * need to protect against async removal of the underlying spi_device.
 */
static void spidev_complete(void *arg)
{
	complete(arg);
}

static ssize_t
spidev_sync(struct spidev_data *spidev, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = spidev_complete;
	message->context = &done;

	spin_lock_irq(&spidev->spi_lock);
	if (spidev->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(spidev->spi, message);
	spin_unlock_irq(&spidev->spi_lock);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		if (status == 0)
			status = message->actual_length;
	}
	return status;
}

static inline ssize_t
spidev_sync_write(struct spidev_data *spidev, size_t len)
{
	struct spi_transfer	t = {
			.tx_buf		= spidev->tx_buffer,
			.len		= len,
			.speed_hz	= spidev->speed_hz,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spidev_sync(spidev, &m);
}

static inline ssize_t
spidev_sync_read(struct spidev_data *spidev, size_t len)
{
	struct spi_transfer	t = {
			.rx_buf		= spidev->rx_buffer,
			.len		= len,
			.speed_hz	= spidev->speed_hz,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spidev_sync(spidev, &m);
}

/*-------------------------------------------------------------------------*/

/* Read-only message with current device setup */
static ssize_t
spidev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct spidev_data	*spidev;
	ssize_t			status = 0;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	spidev = filp->private_data;

	mutex_lock(&spidev->buf_lock);
	status = spidev_sync_read(spidev, count);
	if (status > 0) {
		unsigned long	missing;

		missing = copy_to_user(buf, spidev->rx_buffer, status);
		if (missing == status)
			status = -EFAULT;
		else
			status = status - missing;
	}
	mutex_unlock(&spidev->buf_lock);

	return status;
}

/* Write-only message with current device setup */
static ssize_t
spidev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct spidev_data	*spidev;
	ssize_t			status = 0;
	unsigned long		missing;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	spidev = filp->private_data;

	mutex_lock(&spidev->buf_lock);
	missing = copy_from_user(spidev->tx_buffer, buf, count);
	if (missing == 0)
		status = spidev_sync_write(spidev, count);
	else
		status = -EFAULT;
	mutex_unlock(&spidev->buf_lock);

	return status;
}

static int spidev_message(struct spidev_data *spidev,
		struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	struct spi_message	msg;
	struct spi_transfer	*k_xfers;
	struct spi_transfer	*k_tmp;
	struct spi_ioc_transfer *u_tmp;
	unsigned		n, total, tx_total, rx_total;
	u8			*tx_buf, *rx_buf;
	int			status = -EFAULT;

	spi_message_init(&msg);
	k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
	if (k_xfers == NULL)
		return -ENOMEM;

	/* Construct spi_message, copying any tx data to bounce buffer.
	 * We walk the array of user-provided transfers, using each one
	 * to initialize a kernel version of the same transfer.
	 */
	tx_buf = spidev->tx_buffer;
	rx_buf = spidev->rx_buffer;
	total = 0;
	tx_total = 0;
	rx_total = 0;
	for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
			n;
			n--, k_tmp++, u_tmp++) {
		k_tmp->len = u_tmp->len;

		total += k_tmp->len;
		/* Since the function returns the total length of transfers
		 * on success, restrict the total to positive int values to
		 * avoid the return value looking like an error.  Also check
		 * each transfer length to avoid arithmetic overflow.
		 */
		if (total > INT_MAX || k_tmp->len > INT_MAX) {
			status = -EMSGSIZE;
			goto done;
		}

		if (u_tmp->rx_buf) {
			/* this transfer needs space in RX bounce buffer */
			rx_total += k_tmp->len;
			if (rx_total > bufsiz) {
				status = -EMSGSIZE;
				goto done;
			}
			k_tmp->rx_buf = rx_buf;
			if (!access_ok(VERIFY_WRITE, (u8 __user *)
						(uintptr_t) u_tmp->rx_buf,
						u_tmp->len))
				goto done;
			rx_buf += k_tmp->len;
		}
		if (u_tmp->tx_buf) {
			/* this transfer needs space in TX bounce buffer */
			tx_total += k_tmp->len;
			if (tx_total > bufsiz) {
				status = -EMSGSIZE;
				goto done;
			}
			k_tmp->tx_buf = tx_buf;
			if (copy_from_user(tx_buf, (const u8 __user *)
						(uintptr_t) u_tmp->tx_buf,
					u_tmp->len))
				goto done;
			tx_buf += k_tmp->len;
		}

		k_tmp->cs_change = !!u_tmp->cs_change;
		k_tmp->tx_nbits = u_tmp->tx_nbits;
		k_tmp->rx_nbits = u_tmp->rx_nbits;
		k_tmp->bits_per_word = u_tmp->bits_per_word;
		k_tmp->delay_usecs = u_tmp->delay_usecs;
		k_tmp->speed_hz = u_tmp->speed_hz;
		if (!k_tmp->speed_hz)
			k_tmp->speed_hz = spidev->speed_hz;
#ifdef VERBOSE
		dev_dbg(&spidev->spi->dev,
			"  xfer len %zd %s%s%s%dbits %u usec %uHz\n",
			u_tmp->len,
			u_tmp->rx_buf ? "rx " : "",
			u_tmp->tx_buf ? "tx " : "",
			u_tmp->cs_change ? "cs " : "",
			u_tmp->bits_per_word ? : spidev->spi->bits_per_word,
			u_tmp->delay_usecs,
			u_tmp->speed_hz ? : spidev->spi->max_speed_hz);
#endif
		spi_message_add_tail(k_tmp, &msg);
	}

	status = spidev_sync(spidev, &msg);
	if (status < 0)
		goto done;

	/* copy any rx data out of bounce buffer */
	rx_buf = spidev->rx_buffer;
	for (n = n_xfers, u_tmp = u_xfers; n; n--, u_tmp++) {
		if (u_tmp->rx_buf) {
			if (__copy_to_user((u8 __user *)
					(uintptr_t) u_tmp->rx_buf, rx_buf,
					u_tmp->len)) {
				status = -EFAULT;
				goto done;
			}
			rx_buf += u_tmp->len;
		}
	}
	status = total;

done:
	kfree(k_xfers);
	return status;
}

static struct spi_ioc_transfer *
spidev_get_ioc_message(unsigned int cmd, struct spi_ioc_transfer __user *u_ioc,
		unsigned *n_ioc)
{
	struct spi_ioc_transfer	*ioc;
	u32	tmp;

	/* Check type, command number and direction */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC
			|| _IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
			|| _IOC_DIR(cmd) != _IOC_WRITE)
		return ERR_PTR(-ENOTTY);

	tmp = _IOC_SIZE(cmd);
	if ((tmp % sizeof(struct spi_ioc_transfer)) != 0)
		return ERR_PTR(-EINVAL);
	*n_ioc = tmp / sizeof(struct spi_ioc_transfer);
	if (*n_ioc == 0)
		return NULL;

	/* copy into scratch area */
	ioc = kmalloc(tmp, GFP_KERNEL);
	if (!ioc)
		return ERR_PTR(-ENOMEM);
	if (__copy_from_user(ioc, u_ioc, tmp)) {
		kfree(ioc);
		return ERR_PTR(-EFAULT);
	}
	return ioc;
}

static long
spidev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int			err = 0;
	int			retval = 0;
	struct spidev_data	*spidev;
	struct spi_device	*spi;
	u32			tmp;
	unsigned		n_ioc;
	struct spi_ioc_transfer	*ioc;

	/* Check type and command number */
	//if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)  //ocj for special ioctl cmd;
	//	return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	spidev = filp->private_data;
	spin_lock_irq(&spidev->spi_lock);
	spi = spi_dev_get(spidev->spi);
	spin_unlock_irq(&spidev->spi_lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	/* use the buffer lock here for triple duty:
	 *  - prevent I/O (from us) so calling spi_setup() is safe;
	 *  - prevent concurrent SPI_IOC_WR_* from morphing
	 *    data fields while SPI_IOC_RD_* reads them;
	 *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
	 */
	mutex_lock(&spidev->buf_lock);

	switch (cmd) {
	/* read requests */
	case SPI_IOC_RD_MODE:
		retval = __put_user(spi->mode & SPI_MODE_MASK,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MODE32:
		retval = __put_user(spi->mode & SPI_MODE_MASK,
					(__u32 __user *)arg);
		break;
	case SPI_IOC_RD_LSB_FIRST:
		retval = __put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_BITS_PER_WORD:
		retval = __put_user(spi->bits_per_word, (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MAX_SPEED_HZ:
		retval = __put_user(spidev->speed_hz, (__u32 __user *)arg);
		break;

	/* write requests */
	case SPI_IOC_WR_MODE:
	case SPI_IOC_WR_MODE32:
		if (cmd == SPI_IOC_WR_MODE)
			retval = __get_user(tmp, (u8 __user *)arg);
		else
			retval = __get_user(tmp, (u32 __user *)arg);
		if (retval == 0) {
			u32	save = spi->mode;

			if (tmp & ~SPI_MODE_MASK) {
				retval = -EINVAL;
				break;
			}

			tmp |= spi->mode & ~SPI_MODE_MASK;
			spi->mode = (u16)tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "spi mode %x\n", tmp);
		}
		break;
	case SPI_IOC_WR_LSB_FIRST:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u32	save = spi->mode;

			if (tmp)
				spi->mode |= SPI_LSB_FIRST;
			else
				spi->mode &= ~SPI_LSB_FIRST;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "%csb first\n",
						tmp ? 'l' : 'm');
		}
		break;
	case SPI_IOC_WR_BITS_PER_WORD:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->bits_per_word;

			spi->bits_per_word = tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->bits_per_word = save;
			else
				dev_dbg(&spi->dev, "%d bits per word\n", tmp);
		}
		break;
	case SPI_IOC_WR_MAX_SPEED_HZ:
		retval = __get_user(tmp, (__u32 __user *)arg);
		if (retval == 0) {
			u32	save = spi->max_speed_hz;

			spi->max_speed_hz = tmp;
			retval = spi_setup(spi);
			if (retval >= 0)
				spidev->speed_hz = tmp;
			else
				dev_dbg(&spi->dev, "%d Hz (max)\n", tmp);
			spi->max_speed_hz = save;
		}
		break;

	default:
	    if( (cmd &(~0xff)) == SPI_WRITE_THEN_READ )
	    {
	        /* ocj add for spi read reg >>>*/
	        size_t byte_in = (cmd&0xf0)>>4;
	        size_t byte_out = (cmd&0x0f);
            char* k_buf_in = kmalloc(byte_in, GFP_KERNEL);
            char* k_buf_out = kmalloc(byte_out, GFP_KERNEL);
            void __user *buf_in = (void __user *)arg;
            void __user *buf_out = (void __user *)(arg + byte_in);            
    	    if (__copy_from_user(k_buf_in, buf_in, byte_in)) {
    			kfree(k_buf_in);
    			kfree(k_buf_out);
    			retval = -EFAULT;
    			break;
    		}

            retval = spi_write_then_read(spi, k_buf_in, byte_in, k_buf_out, byte_out);
            if(retval>=0) {
                if (__copy_to_user(buf_out, k_buf_out, byte_out)) {
        			kfree(k_buf_in);
        			kfree(k_buf_out);
        			retval = -EFAULT;
        			break;
        		}
    		}	       
//		printk(" ker--- byte_in %d. byte_out %d\n", byte_in, byte_out);
//		hexdump("byte_in", k_buf_in, byte_in);
//		hexdump("byte_out", k_buf_out, byte_out);
            kfree(k_buf_in);
            kfree(k_buf_out);
            break;
            /* ocj add for spi read reg <<<*/
	    }
	    
	    if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC) {
            retval = -ENOTTY;
		    break;
		}
		/* segmented and/or full-duplex I/O request */
		/* Check message and copy into scratch area */
		ioc = spidev_get_ioc_message(cmd,
				(struct spi_ioc_transfer __user *)arg, &n_ioc);
		if (IS_ERR(ioc)) {
			retval = PTR_ERR(ioc);
			break;
		}
		if (!ioc)
			break;	/* n_ioc is also 0 */

		/* translate to spi_message, execute */
		retval = spidev_message(spidev, ioc, n_ioc);
		kfree(ioc);
		break;
	}

	mutex_unlock(&spidev->buf_lock);
	spi_dev_put(spi);
	return retval;
}

#ifdef CONFIG_COMPAT
static long
spidev_compat_ioc_message(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	struct spi_ioc_transfer __user	*u_ioc;
	int				retval = 0;
	struct spidev_data		*spidev;
	struct spi_device		*spi;
	unsigned			n_ioc, n;
	struct spi_ioc_transfer		*ioc;

	u_ioc = (struct spi_ioc_transfer __user *) compat_ptr(arg);
	if (!access_ok(VERIFY_READ, u_ioc, _IOC_SIZE(cmd)))
		return -EFAULT;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	spidev = filp->private_data;
	spin_lock_irq(&spidev->spi_lock);
	spi = spi_dev_get(spidev->spi);
	spin_unlock_irq(&spidev->spi_lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	/* SPI_IOC_MESSAGE needs the buffer locked "normally" */
	mutex_lock(&spidev->buf_lock);

	/* Check message and copy into scratch area */
	ioc = spidev_get_ioc_message(cmd, u_ioc, &n_ioc);
	if (IS_ERR(ioc)) {
		retval = PTR_ERR(ioc);
		goto done;
	}
	if (!ioc)
		goto done;	/* n_ioc is also 0 */

	/* Convert buffer pointers */
	for (n = 0; n < n_ioc; n++) {
		ioc[n].rx_buf = (uintptr_t) compat_ptr(ioc[n].rx_buf);
		ioc[n].tx_buf = (uintptr_t) compat_ptr(ioc[n].tx_buf);
	}

	/* translate to spi_message, execute */
	retval = spidev_message(spidev, ioc, n_ioc);
	kfree(ioc);

done:
	mutex_unlock(&spidev->buf_lock);
	spi_dev_put(spi);
	return retval;
}

static long
spidev_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	if (_IOC_TYPE(cmd) == SPI_IOC_MAGIC
			&& _IOC_NR(cmd) == _IOC_NR(SPI_IOC_MESSAGE(0))
			&& _IOC_DIR(cmd) == _IOC_WRITE)
		return spidev_compat_ioc_message(filp, cmd, arg);

	return spidev_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define spidev_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int spidev_open(struct inode *inode, struct file *filp)
{
	struct spidev_data	*spidev;
	int			status = -ENXIO;

	mutex_lock(&device_list_lock);

	list_for_each_entry(spidev, &device_list, device_entry) {
		if (spidev->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (status) {
		pr_debug("spidev: nothing for minor %d\n", iminor(inode));
		goto err_find_dev;
	}

	if (!spidev->tx_buffer) {
		spidev->tx_buffer = kmalloc(bufsiz, GFP_KERNEL);
		if (!spidev->tx_buffer) {
				dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			goto err_find_dev;
			}
		}

	if (!spidev->rx_buffer) {
		spidev->rx_buffer = kmalloc(bufsiz, GFP_KERNEL);
		if (!spidev->rx_buffer) {
			dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
			status = -ENOMEM;
			goto err_alloc_rx_buf;
		}
	}

	spidev->users++;
	filp->private_data = spidev;
	nonseekable_open(inode, filp);

	mutex_unlock(&device_list_lock);
	return 0;

err_alloc_rx_buf:
	kfree(spidev->tx_buffer);
	spidev->tx_buffer = NULL;
err_find_dev:
	mutex_unlock(&device_list_lock);
	return status;
}

static int spidev_release(struct inode *inode, struct file *filp)
{
	struct spidev_data	*spidev;
	int			status = 0;

	mutex_lock(&device_list_lock);
	spidev = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	spidev->users--;
	if (!spidev->users) {
		int		dofree;

		kfree(spidev->tx_buffer);
		spidev->tx_buffer = NULL;

		kfree(spidev->rx_buffer);
		spidev->rx_buffer = NULL;

		if (spidev->spi)
			spidev->speed_hz = spidev->spi->max_speed_hz;

		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&spidev->spi_lock);
		dofree = (spidev->spi == NULL);
		spin_unlock_irq(&spidev->spi_lock);

		if (dofree)
			kfree(spidev);
	}
	mutex_unlock(&device_list_lock);

	return status;
}

static const struct file_operations spidev_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.write =	spidev_write,
	.read =		spidev_read,
	.unlocked_ioctl = spidev_ioctl,
	.compat_ioctl = spidev_compat_ioctl,
	.open =		spidev_open,
	.release =	spidev_release,
	.llseek =	no_llseek,
};

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *spidev_class;

#ifdef CONFIG_OF
static const struct of_device_id spidev_dt_ids[] = {
	{ .compatible = "rohm,dh2228fv" },
	{ .compatible = "spidev" },
	{},
};
MODULE_DEVICE_TABLE(of, spidev_dt_ids);
#endif

#define FPGA_SPI_BASE 0xc0a00000
/*-------------------------------------------------------------------------*/
extern void __iomem *spi_master_get_devbase(struct spi_master *master);
static int spidev_probe(struct spi_device *spi)
{
   struct spidev_data	*spidev;
    int			status;
    unsigned long		minor;

    /*
     * spidev should never be referenced in DT without a specific
     * compatbile string, it is a Linux implementation thing
     * rather than a description of the hardware.
     */
    if (spi->dev.of_node && !of_match_device(spidev_dt_ids, &spi->dev))
    {
        dev_err(&spi->dev, "buggy DT: spidev listed directly in DT\n");
        WARN_ON(spi->dev.of_node &&
                !of_match_device(spidev_dt_ids, &spi->dev));
    }

    /* Allocate driver data */
    spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
    if (!spidev)
        return -ENOMEM;

    /* Initialize the driver data */
    spidev->spi = spi;
    spin_lock_init(&spidev->spi_lock);
//MDEBUG("mutex_init(&spidev->buf_lock);\n");
    mutex_init(&spidev->buf_lock);

    INIT_LIST_HEAD(&spidev->device_entry);

    /* If we can allocate a minor number, hook up this device.
     * Reusing minors is fine so long as udev or mdev is working.
     */
    mutex_lock(&device_list_lock);
    minor = find_first_zero_bit(minors, N_SPI_MINORS);
    if (minor < N_SPI_MINORS)
    {
        struct device *dev;

        spidev->devt = MKDEV(SPIDEV_MAJOR, minor);
        dev = device_create(spidev_class, &spi->dev, spidev->devt,
                            spidev, "spimh%d.%d",
                            spi->master->bus_num, spi->chip_select);
        status = PTR_ERR_OR_ZERO(dev);
    }
    else
    {
        dev_dbg(&spi->dev, "no minor number available!\n");
        status = -ENODEV;
    }
    if (status == 0)
    {
        set_bit(minor, minors);
        list_add(&spidev->device_entry, &device_list);
    }

    spidev->speed_hz = spi->max_speed_hz;

    if (status == 0)
        spi_set_drvdata(spi, spidev);
    else
        kfree(spidev);

    if(status == 0)
    {
        MDEBUG(" spidev_probe, ok, spi->chip_select = %d \n", spi->chip_select);
        if (!spidev->tx_buffer)
        {
            spidev->tx_buffer = kmalloc(bufsiz, GFP_KERNEL);
            if (!spidev->tx_buffer)
            {
                    dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
                    status = -ENOMEM;
                    goto err_probe_dev;
            }
        }

        if (!spidev->rx_buffer)
        {
            spidev->rx_buffer = kmalloc(bufsiz, GFP_KERNEL);
            if (!spidev->rx_buffer)
            {
                dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
                status = -ENOMEM;
                goto err_alloc_rx_buf;
            }
        }            
    }
	
	printk("+++++spidev_probe spi base:%08x\n++++++++++",spi_master_get_devbase(spi->master));
    if(status == 0 && spi->chip_select==FPGA_CS && (FPGA_SPI_BASE==spi_master_get_devbase(spi->master)) /* (!fpga_spidev.allocated)*/)//ad的spi怎么确定的,以SPI的基地址为准!!!
    {
        if(fpga_spidev.spidev== spidev)
        {
            MWARN(" fpga_spidev already have this dev \n");
        }
        fpga_spidev.spidev = spidev;

        fpga_spidev.allocated = 1;
	if(fpga_spidev.spidev== NULL)
	{
		fpga_spidev.allocated = 0;
	}
 	else{
		fpga_spidev.ok = true;
		printk("fpga spi init ok.\n");
 	}
    }	
	
    mutex_unlock(&device_list_lock);
    return status;
    
err_alloc_rx_buf:
	kfree(spidev->tx_buffer);
	spidev->tx_buffer = NULL;
err_probe_dev:
    mutex_unlock(&device_list_lock);
    return status;
}

static int spidev_remove(struct spi_device *spi)
{
	struct spidev_data	*spidev = spi_get_drvdata(spi);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&spidev->spi_lock);
	spidev->spi = NULL;
	spin_unlock_irq(&spidev->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&spidev->device_entry);
	device_destroy(spidev_class, spidev->devt);
	clear_bit(MINOR(spidev->devt), minors);
	if (spidev->users == 0)
		kfree(spidev);
	mutex_unlock(&device_list_lock);

	return 0;
}

static struct spi_driver spidev_spi_driver = {
	.driver = {
		.name =		"spidev",
		.owner =	THIS_MODULE,
		.of_match_table = of_match_ptr(spidev_dt_ids),
	},
	.probe =	spidev_probe,
	.remove =	spidev_remove,

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};

/*-------------------------------------------------------------------------*/

static int __init spidev_init(void)
{
	int status;

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, "spi", &spidev_fops);
	if (status < 0)
		return status;

	spidev_class = class_create(THIS_MODULE, "spidev");
	if (IS_ERR(spidev_class)) {
		unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
		return PTR_ERR(spidev_class);
	}

	status = spi_register_driver(&spidev_spi_driver);
	if (status < 0) {
		class_destroy(spidev_class);
		unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
	}
	return status;
}
module_init(spidev_init);

static void __exit spidev_exit(void)
{
	spi_unregister_driver(&spidev_spi_driver);
	class_destroy(spidev_class);
	unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
}
module_exit(spidev_exit);

//////对外接口的封装////////////////////////////////////////////////
#define SPI_FPGA_CH 0


#define SPI_CMD_OPEN 0xAA000000
#define SPI_CMD_CLOSE 0xAA000001
int fpga_spi_open(void)
{
	struct spidev_data	*spidev = fpga_spidev.spidev;
	int			status = -ENXIO;

	mutex_lock(&device_list_lock);

	if (!spidev->tx_buffer) {
		spidev->tx_buffer = kmalloc(bufsiz, GFP_KERNEL);
		if (!spidev->tx_buffer) {
				dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			goto err_find_dev;
			}
		}

	if (!spidev->rx_buffer) {
		spidev->rx_buffer = kmalloc(bufsiz, GFP_KERNEL);
		if (!spidev->rx_buffer) {
			dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
			status = -ENOMEM;
			goto err_alloc_rx_buf;
		}
	}

	spidev->users++;
	//filp->private_data = spidev;
	//nonseekable_open(inode, filp);

	mutex_unlock(&device_list_lock);
	return 0;

err_alloc_rx_buf:
	kfree(spidev->tx_buffer);
	spidev->tx_buffer = NULL;
err_find_dev:
	mutex_unlock(&device_list_lock);
	return status;
}
EXPORT_SYMBOL_GPL(fpga_spi_open);


static int fpga_spi_release(void)
{
	struct spidev_data	*spidev = fpga_spidev.spidev;
	int			status = 0;

	mutex_lock(&device_list_lock);

	/* last close? */
	spidev->users--;
	if (!spidev->users) {
		int		dofree;

		kfree(spidev->tx_buffer);
		spidev->tx_buffer = NULL;

		kfree(spidev->rx_buffer);
		spidev->rx_buffer = NULL;

		if (spidev->spi)
			spidev->speed_hz = spidev->spi->max_speed_hz;

		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&spidev->spi_lock);
		dofree = (spidev->spi == NULL);
		spin_unlock_irq(&spidev->spi_lock);

		if (dofree)
			kfree(spidev);
	}
	mutex_unlock(&device_list_lock);

	return status;
}

long fpga_spi_cfg(unsigned int cmd, unsigned long arg)
{
    int			err = 0;
    int			retval = 0;
    struct spidev_data	*spidev;
    struct spi_device	*spi;
    u32			tmp;
    unsigned		n_ioc;
    struct spi_ioc_transfer	*ioc;

	//printk("fpga_spi_cfg: cmd:%08x,arg:%08x\n",cmd,arg);
    /* Check access direction once here; don't repeat below.
     * IOC_DIR is from the user perspective, while access_ok is
     * from the kernel perspective; so they look reversed.
     */	
    /*if (_IOC_DIR(cmd) & _IOC_READ)
        err = !access_ok(VERIFY_WRITE,
                         (void __user *)arg, _IOC_SIZE(cmd));
    if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
        err = !access_ok(VERIFY_READ,
                         (void __user *)arg, _IOC_SIZE(cmd));
    if (err) {
		MWARN("fpga_spi_cfg access error\n");
        return -EFAULT;
    }
*/
   if((!fpga_spidev.allocated)||(fpga_spidev.spidev==NULL)) {
		MWARN("fpga_spi_cfg dev error\n");
        return -EFAULT;
    }
	spidev = fpga_spidev.spidev;	
    spin_lock_irq(&spidev->spi_lock);
    spi = spi_dev_get(spidev->spi);//增加使用索引数
    spin_unlock_irq(&spidev->spi_lock);
    if (spi == NULL){
		MWARN("fpga_spi_cfg spi == NULL error\n");
        return -ESHUTDOWN;
    	}

   mutex_lock(&spidev->buf_lock);	
    switch (cmd)
    {
	        /* read requests */
	    case SPI_IOC_RD_MODE:
		*((__u8 *)arg) = spi->mode & SPI_MODE_MASK;
		retval = 0;
	        break;
	    case SPI_IOC_RD_MODE32:
		*((__u32 *)arg) = spi->mode & SPI_MODE_MASK;
		retval = 0;			
	        break;
	    case SPI_IOC_RD_LSB_FIRST:
		*((__u8 *)arg) = (spi->mode & SPI_LSB_FIRST) ?  1 : 0;	
		retval = 0;	
	        break;
	    case SPI_IOC_RD_BITS_PER_WORD:
	        retval = 0;
	        *((__u8 __user *)arg) = spi->bits_per_word;
	        break;
	    case SPI_IOC_RD_MAX_SPEED_HZ:
	        retval = 0;
		 *((__u32 __user *)arg) = spidev->speed_hz;	
	        break;

	        /* write requests */
	    case SPI_IOC_WR_MODE:
	    case SPI_IOC_WR_MODE32:
	        if (cmd == SPI_IOC_WR_MODE)
			tmp = *((u8 *)arg);
	        else
	            tmp = *((u32*)arg);
			
		 retval = 0;
		 //printk("fpga spi mode:get user%d\n",retval);
	        if (retval == 0)
	        {
	            u32	save = spi->mode;

	            if (tmp & ~SPI_MODE_MASK)
	            {
	            	printk("fpga spi mode:tmp & ~SPI_MODE_MASK\n");
	                retval = -EINVAL;
	                break;
	            }

	            tmp |= spi->mode & ~SPI_MODE_MASK;
	            spi->mode = (u16)tmp;
	            retval = spi_setup(spi);
				printk("fpga spi mode:%d\n",spi->mode);
	            if (retval < 0)
	                spi->mode = save;
	            else
	                dev_dbg(&spi->dev, "spi mode %x\n", tmp);
	        }
	        break;
	    case SPI_IOC_WR_LSB_FIRST:
	        retval = 0;
		tmp = *((__u8*)arg);
	        if (retval == 0)
	        {
	            u32	save = spi->mode;

	            if (tmp)
	                spi->mode |= SPI_LSB_FIRST;
	            else
	                spi->mode &= ~SPI_LSB_FIRST;
	            retval = spi_setup(spi);
	            if (retval < 0)
	                spi->mode = save;
	            else
	                dev_dbg(&spi->dev, "%csb first\n",
	                        tmp ? 'l' : 'm');
	        }
	        break;
	    case SPI_IOC_WR_BITS_PER_WORD:
	        retval = 0;
		 tmp = *((__u8*)arg);
	        if (retval == 0)
	        {
	            u8	save = spi->bits_per_word;

	            spi->bits_per_word = tmp;
	            retval = spi_setup(spi);
	            if (retval < 0)
	                spi->bits_per_word = save;
	            else
	                dev_dbg(&spi->dev, "%d bits per word\n", tmp);
	        }
	        break;
	    case SPI_IOC_WR_MAX_SPEED_HZ:
	        retval = 0;
		 tmp = *((__u32*)arg);
	        if (retval == 0)
	        {
	            u32	save = spi->max_speed_hz;

	            spi->max_speed_hz = tmp;
	            retval = spi_setup(spi);
	            if (retval >= 0)
	                spidev->speed_hz = tmp;
	            else
	                dev_dbg(&spi->dev, "%d Hz (max)\n", tmp);
	            spi->max_speed_hz = save;
	        }
	        break;
	    case SPI_CMD_OPEN:
			fpga_spi_open();
			break;
	   case SPI_CMD_CLOSE:
			fpga_spi_release();
			break;
	    default:
		break;
    }
	mutex_unlock(&spidev->buf_lock);
	
    spi_dev_put(spi);//减少使用索引数
    return retval;	
}
EXPORT_SYMBOL_GPL(fpga_spi_cfg);


 int fpga_spi_read(char  *buf, size_t count)
{
    int retval,i;
    struct spidev_data *spidev = fpga_spidev.spidev;
	
   if((!fpga_spidev.allocated)||(fpga_spidev.spidev==NULL)) {
   	MWARN("fpga_spi_read dev error\n");
        return -EFAULT;
    }
	
    if (count > bufsiz)
    {
        return -EFAULT;
    }

   if((!fpga_spidev.allocated)||(fpga_spidev.spidev==NULL)) {
   	MWARN("fpga_spi_read dev error\n");
        return -EFAULT;
    }
	//mutex_lock(&spidev->buf_lock);
    retval = spidev_sync_read(spidev, count);
    if (retval > 0)
    {
        memcpy(buf, spidev->rx_buffer, retval);
    }
	//mutex_unlock(&spidev->buf_lock);
#if 0
	printk("fpga_spi_read (count:%d,ret=%d):",count,retval);
	for(i = 0; i < count; i++){
		printk("\t%02x",spidev->rx_buffer[i]);
	}
	printk("\n");
#endif
    return retval;
}
 EXPORT_SYMBOL_GPL(fpga_spi_read);

 int fpga_spi_write(char *buf, size_t count)
{
    int retval;
    struct spidev_data *spidev = fpga_spidev.spidev;
	
   if(!fpga_spidev.allocated) {
   	MWARN("fpga_spi_write dev error\n");
        return -EFAULT;
    }

    if (count > bufsiz)
    {
        MWARN("fpga_spi_write count > bufsiz\n");
        return -EFAULT;
    }
    if (spidev == NULL)
    {
      MWARN("fpga_spi_write spidev NULL\n");
        return -EFAULT;
    }
   if(spidev->tx_buffer==NULL){
   	MWARN("fpga_spi_write tx_buffer NULL\n");
        return -EFAULT;   	
   }
   
    memcpy(spidev->tx_buffer, buf, count);
    retval = spidev_sync_write(spidev, count);
    if(retval < 0)
    {
        MWARN(" ads1256_spi_write failed\n");
    }
    return retval;
}
  EXPORT_SYMBOL_GPL(fpga_spi_write);

int fpga_write_then_read(uint8_t  *buf_in,
                           size_t byte_in,
                           uint8_t *buf_out,
                           size_t byte_out)
{
    int ret = -1;
	struct spi_device	*spi;
	struct spidev_data *spidev;
	unsigned int cmd = SPI_WRITE_THEN_READ;
	char *k_buf_in,*k_buf_out;
	
    if(byte_in > 0xf || byte_out > 0xf)
    {
        return ret;
    }
    
	spidev = fpga_spidev.spidev;
	
   if((!fpga_spidev.allocated)||(fpga_spidev.spidev==NULL)) {
   	MWARN("fpga_write_then_read dev error\n");
        return -EFAULT;
    }
    spin_lock_irq(&spidev->spi_lock);
    spi = spi_dev_get(spidev->spi);//增加使用索引数
    spin_unlock_irq(&spidev->spi_lock);
	if (spi == NULL)
		return -ESHUTDOWN;
	
    
    //uint8_t *buf = (uint8_t *) kmalloc(byte_in + byte_out, GFP_KERNEL);
	k_buf_in = kmalloc(byte_in, GFP_KERNEL);
    k_buf_out = kmalloc(byte_out, GFP_KERNEL);
	
    cmd |= byte_in<<4;
    cmd |= byte_out;
    memcpy(k_buf_in, buf_in, byte_in);
    ret = spi_write_then_read(spi, k_buf_in, byte_in, k_buf_out, byte_out);
    memcpy(buf_out, k_buf_out, byte_out);
	spi_dev_put(spi);
	kfree(k_buf_in);
	kfree(k_buf_out);
    return ret;
}
EXPORT_SYMBOL_GPL(fpga_write_then_read);

int fpga_spi_status(void)
{
	return fpga_spidev.ok;
}
EXPORT_SYMBOL_GPL(fpga_spi_status);

//////////////////////////////////////////////////////////////


MODULE_AUTHOR("Andrea Paterniani, <a.paterniani@swapp-eng.it>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spidev");
