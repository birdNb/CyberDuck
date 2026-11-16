// SPDX-License-Identifier: GPL-2.0
/*
 * 把 TLMM（0x0f100000）映射给用户态。
 * 本机内核没开 /dev/mem，LED_Ring 的 WS2812 位bang 需要这块寄存器。
 */
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pfn.h>

#define TLMM_PHYS 0x0f100000UL
#define TLMM_SIZE 0x00100000UL

static int tlmm_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long size = vma->vm_end - vma->vm_start;

	if (vma->vm_pgoff != 0 || size == 0 || size > TLMM_SIZE)
		return -EINVAL;

	vm_flags_set(vma, VM_IO | VM_PFNMAP | VM_DONTEXPAND | VM_DONTDUMP);
	vma->vm_page_prot = pgprot_device(vma->vm_page_prot);
	return remap_pfn_range(vma, vma->vm_start, TLMM_PHYS >> PAGE_SHIFT,
			       size, vma->vm_page_prot);
}

static const struct file_operations tlmm_fops = {
	.owner = THIS_MODULE,
	.mmap = tlmm_mmap,
};

static struct miscdevice tlmm_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ws2812_tlmm",
	.fops = &tlmm_fops,
	.mode = 0600,
};

static int __init tlmm_init(void)
{
	return misc_register(&tlmm_dev);
}

static void __exit tlmm_exit(void)
{
	misc_deregister(&tlmm_dev);
}

module_init(tlmm_init);
module_exit(tlmm_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Map QCM6490 TLMM for WS2812 GPIO bitbang");
