/*
 *  Driver for Ring buffer and clip buffer reserved memory.
 *
 *  Copyright (C)2017 Neuroptics
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 */
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <linux/random.h>
#include <linux/init.h>
#include <linux/raw.h>
#include <linux/tty.h>
#include <linux/capability.h>
#include <linux/ptrace.h>
#include <linux/device.h>
#include <linux/highmem.h>
#include <linux/backing-dev.h>
#include <linux/splice.h>
#include <linux/pfn.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/uio.h>
#include <linux/module.h>
#include <linux/uaccess.h>

MODULE_AUTHOR("Neuroptics");
MODULE_DESCRIPTION("Ring buffer and clip buffer reserved memory driver");
MODULE_LICENSE("GPL");

static int major;	/* default is dynamic major device number */
static void __iomem *Clip_base,*Ring_base;

#define Ring_START	0x28000000UL
#define Clip_START	0x80000000UL
#define Ring_SIZE	0x08000000UL
#define Clip_SIZE	0x20000000UL


extern int block_backlight_change;


#ifndef ARCH_HAS_VALID_PHYS_ADDR_RANGE
static inline int valid_phys_addr_range(phys_addr_t addr, size_t count)
{
	return addr + count <= __pa(high_memory);
}

static inline int valid_mmap_phys_addr_range(unsigned long pfn, size_t size)
{
	return 1;
}
#endif

static inline int range_is_allowed(unsigned long pfn, unsigned long size)
{
	return 1;
}

int __weak phys_mem_access_prot_allowed(struct file *file,
	unsigned long pfn, unsigned long size, pgprot_t *vma_prot)
{
	return 1;
}

#ifndef __HAVE_PHYS_MEM_ACCESS_PROT

/*
 * Architectures vary in how they handle caching for addresses
 * outside of main memory.
 *
 */
#ifdef pgprot_noncached
static int uncached_access(struct file *file, phys_addr_t addr)
{
#if defined(CONFIG_IA64)
	/*
	 * On ia64, we ignore O_DSYNC because we cannot tolerate memory
	 * attribute aliases.
	 */
	return !(efi_mem_attributes(addr) & EFI_MEMORY_WB);
#elif defined(CONFIG_MIPS)
	{
		extern int __uncached_access(struct file *file,
					     unsigned long addr);

		return __uncached_access(file, addr);
	}
#else
	/*
	 * Accessing memory above the top the kernel knows about or through a
	 * file pointer
	 * that was marked O_DSYNC will be done non-cached.
	 */
	if (file->f_flags & O_DSYNC)
		return 1;
	return addr >= __pa(high_memory);
#endif
}
#endif

static pgprot_t phys_mem_access_prot(struct file *file, unsigned long pfn,
				     unsigned long size, pgprot_t vma_prot)
{
#ifdef pgprot_noncached
	phys_addr_t offset = pfn << PAGE_SHIFT;

	if (uncached_access(file, offset))
		return pgprot_noncached(vma_prot);
#endif
	return vma_prot;
}
#endif


static const struct vm_operations_struct mmap_mem_ops = {
#ifdef CONFIG_HAVE_IOREMAP_PROT
	.access = generic_access_phys
#endif
};

static inline int private_mapping_ok(struct vm_area_struct *vma)
{
	return 1;
}

static int mmap_mem(struct file *file, struct vm_area_struct *vma)
{
	size_t size = vma->vm_end - vma->vm_start;

	if (!valid_mmap_phys_addr_range(vma->vm_pgoff, size))
		return -EINVAL;

	if (!private_mapping_ok(vma))
		return -ENOSYS;

	if (!range_is_allowed(vma->vm_pgoff, size))
		return -EPERM;

	if (!phys_mem_access_prot_allowed(file, vma->vm_pgoff, size,
						&vma->vm_page_prot))
		return -EINVAL;

	vma->vm_page_prot = phys_mem_access_prot(file, vma->vm_pgoff,
						 size,
						 vma->vm_page_prot);

	vma->vm_ops = &mmap_mem_ops;

	/* Remap-pfn-range will mark the range VM_IO */
	if (remap_pfn_range(vma,
			    vma->vm_start,
			    vma->vm_pgoff,
			    size,
			    vma->vm_page_prot)) {
		return -EAGAIN;
	}
	return 0;
}

static const struct file_operations __maybe_unused clip_ring1_fops = {
	.mmap		= mmap_mem,
};

static const struct memdev {
	const char *name;
	umode_t mode;
	const struct file_operations *fops;
	fmode_t fmode;
} devlist[] = {
	 [1] = { "clip_ring", 0, &clip_ring1_fops, FMODE_UNSIGNED_OFFSET },
};

static int memory_open(struct inode *inode, struct file *filp)
{
	int minor;
	const struct memdev *dev;

	minor = iminor(inode);
	if (minor >= ARRAY_SIZE(devlist))
		return -ENXIO;

	dev = &devlist[minor];
	if (!dev->fops)
		return -ENXIO;

	filp->f_op = dev->fops;
	filp->f_mode |= dev->fmode;

	if (dev->fops->open)
		return dev->fops->open(inode, filp);

	return 0;
}

static const struct file_operations clip_ring_fops = {
	.owner		= THIS_MODULE,
	.open 		= memory_open,
	.llseek		= no_llseek,
};

static struct class *mem_class;

static char *mem_devnode(struct device *dev, umode_t *mode)
{
	if (mode && devlist[MINOR(dev->devt)].mode)
		*mode = devlist[MINOR(dev->devt)].mode;
	return NULL;
}

static int __init clip_ring_init(void)

{
	int retval;
	int minor;

	printk("blocking backlight change...\n");
	block_backlight_change = 1;
		printk("clip_ring: major number %d\n", major);
	if (request_mem_region(Ring_START, Ring_SIZE, "ring") == NULL){
		printk("request_mem_region ring busy\n");
		return -EBUSY;
	}


	Ring_base = ioremap(Ring_START, Ring_SIZE);
	if (Ring_base == NULL) {
		printk("ioremap ring no memory\n");
		release_mem_region(Ring_START, Ring_SIZE);
		return -ENOMEM;
	}
	else
	{
		printk("Ring_base at:%p  \n", Ring_base);
        	//memset(Ring_base,0x80, Ring_SIZE);
	}

	if (request_mem_region(Clip_START, Clip_SIZE, "clip") == NULL){
		printk("request_mem_region clip busy\n");
		return -EBUSY;
	}
	
	Clip_base = ioremap(Clip_START, Clip_SIZE);
	if (Clip_base == NULL) {
		printk("ioremap clip no memory\n");
		release_mem_region(Clip_START, Clip_SIZE);
		return -ENOMEM;
	}
	else
	{
		printk("Clip_base at:%p  \n", Clip_base);
        //	memset(Clip_base,0x80, Clip_SIZE);
	}

	retval = register_chrdev(major, "clip_ring", &clip_ring_fops);
	if (retval < 0) {
		iounmap(Ring_base);
		iounmap(Clip_base);
		Ring_base = NULL;
		Clip_base = NULL;
		release_mem_region(Ring_START, Ring_SIZE);
		release_mem_region(Clip_START, Clip_SIZE);
		return retval;
	}

	if (major == 0) {
		major = retval;
		printk("clip_ring: major number %d\n", major);
	}

	mem_class = class_create(THIS_MODULE, "clip_ring");
	if (IS_ERR(mem_class))
		return PTR_ERR(mem_class);
	
		mem_class->devnode = mem_devnode;
	for (minor = 1; minor < ARRAY_SIZE(devlist); minor++) {
		if (!devlist[minor].name)
			continue;

	device_create(mem_class, NULL, MKDEV(major, minor),
	     NULL, devlist[minor].name);
	}

	return 0;
}

static void __exit clip_ring_exit(void)
{
	iounmap(Ring_base);
	iounmap(Clip_base);

	Ring_base = NULL;
	Clip_base = NULL;

	release_mem_region(Ring_START, Ring_SIZE);
	release_mem_region(Clip_START, Clip_SIZE);

	unregister_chrdev(major, "clip_ring");

}

module_init(clip_ring_init);
module_exit(clip_ring_exit);
