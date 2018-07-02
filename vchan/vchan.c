/*  vchan.c - Kernel module for VM -> VM communication

 * Copyright (C) 2018 DornerWorks, Ltd.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.

 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License along
 *   with this program. If not, see <http://www.gnu.org/licenses/>.

 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>

/* Standard module information, edit as appropriate */
MODULE_LICENSE("GPL");
MODULE_AUTHOR
("DornerWorks, Ltd.");
MODULE_DESCRIPTION
("vchan - virtual channel module for seL4 VMs");

#define DRIVER_NAME "vchan"

static struct cdev vchan_cdev;
static dev_t vchan_dev;
static struct class *vchan_cdev_class;

#define VCHAN_READ    0
#define VCHAN_WRITE   1
#define VCHAN_NUM_DIR 2

#define VCHAN_OFF     0
#define VCHAN_ON      1

#define VCHAN_SIZE(v)  (v->end - v->start)

#define VCHAN_READ_TOKEN     0xfabbdad
#define VCHAN_WRITE_TOKEN    0xfabbdab
#define VCHAN_REGISTER_TOKEN 0xfabbdac

#define VCHAN_BAD_CHECKSUM  0xFF
#define VCHAN_MALLOC_FAILED 0xEE
#define VCHAN_NO_DIR        0xBAD

#define MAX_VCHANS 10

struct vchan_local {
    u64 start;
    u64 end;
    void __iomem *addr;
    u32 dir;
    u32 port;
    dev_t major;
};

static struct vchan_local *lp[MAX_VCHANS];
static int num_vchans = 0;

static const char * directions[VCHAN_NUM_DIR] = {"rx", "tx"};

int vchan_open(struct inode *inode, struct file *filp) {
    return 0;
}

int vchan_release(struct inode *inode, struct file *filp) {
    return 0;
}

static void call_into_hypervisor(u8 read, u8 *chk, size_t *len, int device)
{
    register uint64_t port asm("x0") = lp[device]->port;
    register uint64_t checksum asm("x1") = *chk;
    register uint64_t length asm("x2") = *len;
    register uint64_t token asm("x7") = read == VCHAN_READ ? VCHAN_READ_TOKEN : VCHAN_WRITE_TOKEN;

    asm volatile ("hvc #0"
                  : "+r" (length), "+r" (checksum)
                  : "r" (port), "r"(token));

    *len = length;
    *chk = checksum;
}

static int get_major(dev_t major)
{
    int i;

    for (i = 0; i < num_vchans; i++) {
        if (MAJOR(major) == lp[i]->major) {
            break;
        }
    }
    return (i < num_vchans) ? i : -1;
}

static int register_vchan(int device, int status)
{
    register uint64_t port asm("x0") = lp[device]->port;
    register uint64_t direction asm("x1") = lp[device]->dir;
    register uint64_t stat asm("x2") = status;
    register uint64_t token asm("x7") = VCHAN_REGISTER_TOKEN;

    asm volatile ("hvc #0"
                  :
                  : "r" (port), "r" (stat), "r" (direction), "r"(token));

    return 0;
}

static ssize_t vchan_read(struct file *f, char __user *buf, size_t len, loff_t *off)
{
    int id = get_major(f->f_inode->i_rdev);

    /* Need to get the information of the calling device */
    if (id == -1)
    {
        return -EFAULT;
    }

    struct vchan_local *vchan = lp[id];

    if (vchan->dir == VCHAN_READ)
    {
        int i;
        u8 byte, chk = 0, hyp_chk;
        size_t size = VCHAN_SIZE(vchan);

        call_into_hypervisor(VCHAN_READ, &hyp_chk, &len, id);

        if(len > size)
        {
            printk("Message size greater than available buffer somehow\n");
            return -EAGAIN;
        }
        for (i = 0; i < len; i++)
        {
            byte = ioread8((u8 *)vchan->addr + i);
            chk += byte;
            if (copy_to_user(buf + i, &byte, 1))
            {
                return -EFAULT;
            }
        }
        if (chk != hyp_chk)
        {
            return -EAGAIN;
        }
        return len;
    }
    else
    {
        return -EFAULT;
    }
    return 0;
}

static ssize_t vchan_write(struct file *f, const char __user *buf, size_t len, loff_t *off)
{
    int id = get_major(f->f_inode->i_rdev);

    /* Need to get the information of the calling device */
    if (id == -1)
    {
        return -EFAULT;
    }

    struct vchan_local *vchan = lp[id];

    if (vchan->dir == VCHAN_WRITE)
    {
        int i;
        u8 byte;
        size_t size = VCHAN_SIZE(vchan);
        u8 chk = 0;

        if(len > size)
        {
            len = size;
        }
        for (i = 0; i < len; i++)
        {
            if (copy_from_user(&byte, buf + i, 1))
            {
                return -EFAULT;
            }
            chk += byte;
            iowrite8(byte, (u8 *)vchan->addr + i);
        }

        call_into_hypervisor(VCHAN_WRITE, &chk, &len, id);

        if ((len == 0) && (chk == VCHAN_MALLOC_FAILED)) {
            return -ENOMEM;
        }
        else if ((len == 0) && (chk == VCHAN_BAD_CHECKSUM)) {
            return -EBADE;
        }
        else {
            return len;
        }
    }
    else
    {
        return -EFAULT;
    }
    return 0;
}

static struct file_operations vchan_fops =
{
    .owner = THIS_MODULE,
    .open = vchan_open,
    .release = vchan_release,
    .read = vchan_read,
    .write = vchan_write
};

/* Initialize a character device for vchan.
 *
 * Major Number: Device ID
 * Minor Number: Read (0) / Write (1)
 */
static int vchan_init_cdev(struct device *dev)
{
    struct vchan_local *vchan = lp[num_vchans];

    if (alloc_chrdev_region(&vchan_dev, vchan->dir, 1, DRIVER_NAME) != 0)
    {
        dev_err(dev, "failed to register character device region\n");
        goto error1;
    }

    vchan->major = MAJOR(vchan_dev);

    if (vchan_cdev_class == NULL) {
        if ((vchan_cdev_class = class_create(THIS_MODULE, "chardrv")) == NULL)
        {
            dev_err(dev, "Failed to create class\n");
            goto error2;
        }
    }

    if (device_create(vchan_cdev_class, NULL, vchan_dev, NULL, \
                      "vchan%d_%s", vchan->port, directions[vchan->dir]) == NULL)
    {
        dev_err(dev, "Failed to create device\n");
        goto error3;
    }

    cdev_init(&vchan_cdev, &vchan_fops);

    if (cdev_add(&vchan_cdev, vchan_dev, 1) == -1)
    {
        dev_err(dev, "Failed to add device\n");
        goto error4;
    }
    return 0;

error4:
    device_destroy(vchan_cdev_class, vchan_dev);
error3:
    class_destroy(vchan_cdev_class);
error2:
    unregister_chrdev_region(vchan_dev, 1);
error1:
    return -1;
}

static int vchan_probe(struct platform_device *pdev)
{
    struct resource *r_mem; /* IO mem resources Read and Write page */
    struct device *dev = &pdev->dev;

    struct vchan_local *current_vchan;

    int rc = 0;

    current_vchan = (struct vchan_local *) kzalloc(sizeof(struct vchan_local), GFP_KERNEL);
    if (!current_vchan)
    {
        printk("Cound not allocate vchan device\n");
        return -ENOMEM;
    }

    lp[num_vchans] = current_vchan;

    dev_set_drvdata(dev, lp);

    /* Get port from device tree node */
    void *port_dts;
    port_dts = of_get_property(pdev->dev.of_node, "port", NULL);

    if (port_dts == NULL)
    {
        dev_err(dev, "Could not find port in device tree\n");
        return -ENODEV;
    }

    current_vchan->port = be32_to_cpup(port_dts);

    /* Get direction from device tree node */
    void *dir_dts;
    dir_dts = of_get_property(pdev->dev.of_node, "dir", NULL);

    if (dir_dts == NULL)
    {
        dev_err(dev, "Could not find direction in device tree\n");
        return -ENODEV;
    }

    current_vchan->dir = VCHAN_NO_DIR;
    int i;

    for (i = 0; i < VCHAN_NUM_DIR; i++) {
        if (memcmp(dir_dts, directions[i], 2) == 0) {
            current_vchan->dir = i;
            break;
        }
    }

    if (current_vchan->dir == VCHAN_NO_DIR)
    {
        dev_err(dev, "Direction in device tree does not match rx/tx\n");
        return -ENODEV;
    }

    r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!r_mem)
    {
        dev_err(dev, "invalid address\n");
        return -ENODEV;
    }

    current_vchan->start = r_mem->start;
    current_vchan->end = r_mem->end;

    if (!request_mem_region(current_vchan->start,
                            current_vchan->end - current_vchan->start + 1,
                            DRIVER_NAME))
    {
        dev_err(dev, "Couldn't lock memory region at %p\n",
                (void *)current_vchan->start);
        rc = -EBUSY;
        goto error1;
    }

    current_vchan->addr = ioremap(current_vchan->start, current_vchan->end - current_vchan->start + 1);
    if (!current_vchan->addr)
    {
        dev_err(dev, "Could not allocate iomem\n");
        rc = -EIO;
        goto error2;
    }

    if(vchan_init_cdev(dev) == -1)
    {
        dev_err(dev, "Failed to initialize character device\n");
        goto error2;
    }

    register_vchan(num_vchans, VCHAN_ON);

    num_vchans++;

    return 0;

error2:
    release_mem_region(current_vchan->start, current_vchan->end - current_vchan->start + 1);
error1:
    kfree(current_vchan);
    dev_set_drvdata(dev, NULL);
    return rc;
}

static int vchan_remove(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    int id = get_major(dev->devt);
    struct vchan_local *vchan = lp[id];

    release_mem_region(vchan->start, vchan->end - vchan->start + 1);

    kfree(vchan);

    num_vchans--;

    dev_set_drvdata(dev, NULL);
    return 0;
}

#ifdef CONFIG_OF
static struct of_device_id vchan_of_match[] = {
    { .compatible = "vchan", },
    { /* end of list */ },
};
MODULE_DEVICE_TABLE(of, vchan_of_match);
#else
# define vchan_of_match
#endif


static struct platform_driver vchan_driver = {
    .driver =
    {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,
        .of_match_table = vchan_of_match,
    },
    .probe      = vchan_probe,
    .remove     = vchan_remove,
};

static int __init vchan_init(void)
{
    return platform_driver_register(&vchan_driver);
}

static void __exit vchan_exit(void)
{
    int i;

    for (i = 0; i < MAX_VCHANS; i++) {
        if(lp[i] != NULL) {
            kfree(lp[i]);
        }
    }

    platform_driver_unregister(&vchan_driver);
    printk(KERN_ALERT "Removing vchan.\n");
}

module_init(vchan_init);
module_exit(vchan_exit);
