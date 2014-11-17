/**
 *
 * Copyright (c) Xiphos Systems Corp. 2014
 *
 * An ECC translation layer for MTD devices developed originally for
 * use on a spacecraft processor using QSPI (m25p80) NOR flash.
 *
 * For terrestrial use, NOR errors are extremely rare. In Low-Earth Orbit,
 * exposed to high-energy protons, single bit errors can occur in NOR flash
 * including 'erase' events (0 -> 1).
 */

 
#include <linux/err.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mtd/mtd.h>

static struct mtd_info * mtd_under;

static struct mtd_info * mtd_ecc;

static const char * mtd_ecc_name = "MTD_ECC";

static int mtd_ecc_lock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	return(mtd_under->_lock(mtd_under,ofs,len));
}

static int mtd_ecc_unlock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	return(mtd_under->_unlock(mtd_under,ofs,len));
}

static int mtd_ecc_is_locked(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	return(mtd_under->_is_locked(mtd_under,ofs,len));
}
/**
 * mtd_ecc_read - read operation of ECC encapsulated MTD devices.
 * @mtd: MTD device description object
 * @from: absolute offset from where to read
 * @len: how many bytes to read
 * @retlen: count of read bytes is returned here
 * @buf: buffer to store the read data
 *
 * This function returns zero in case of success and a negative error code in
 * case of failure.
 */
static int mtd_ecc_read(struct mtd_info *mtd, loff_t from, size_t len,
		       size_t *retlen, unsigned char *buf)
{
	return(mtd_under->_read(mtd_under,from,len,retlen,buf));
}
#if 0
static void _read()
{
	// Determine the region that needs to be read.
	// Each readable chunk is going to be 3584 bytes
	// 64 bytes of ECC for each 512 byte chunk
	//

}
#endif // foo
/**
 * mtd_ecc_write - write operation of ECC encapsulated MTD devices.
 * @mtd: MTD device description object
 * @to: absolute offset where to write
 * @len: how many bytes to write
 * @retlen: count of written bytes is returned here
 * @buf: buffer with data to write
 *
 * This function returns zero in case of success and a negative error code in
 * case of failure.
 */
static int mtd_ecc_write(struct mtd_info *mtd, loff_t to, size_t len,
			size_t *retlen, const u_char *buf)
{
	return(mtd_under->_write(mtd_under,to,len,retlen,buf));
}

/**
 * mtd_ecc_erase - erase operation of ECC encapsulated MTD devices.
 * @mtd: the MTD device description object
 * @instr: the erase operation description
 *
 * This function calls the erase callback when finishes. Returns zero in case
 * of success and a negative error code in case of failure.
 */
static int mtd_ecc_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	return(mtd_under->_erase(mtd_under,instr));
}
/**
 * mtd_ecc_get_device - get MTD device reference.
 * @mtd: the MTD device description object
 *
 * This function is called every time the MTD device is being opened and
 * implements the MTD get_device() operation. Returns zero in case of success
 * and a negative error code in case of failure.
 */
static int mtd_ecc_get_device(struct mtd_info *mtd)
{
	return(mtd_under->_get_device(mtd_under));
}
	

/**
 * mtd_ecc_put_device - put MTD device reference.
 * @mtd: the MTD device description object
 *
 * This function is called every time the MTD device is being put. Returns
 * zero in case of success and a negative error code in case of failure.
 */
static void mtd_ecc_put_device(struct mtd_info *mtd)
{
	return(mtd_under->_put_device(mtd_under));
}

static int __init mtdecc_init(void)
{
	int err = 0;
  int i;
#define MTD_NUM 39

	mtd_under = get_mtd_device(NULL, MTD_NUM);
  if (!mtd_under) {
		printk(KERN_ERR "Unable to get MTD device 39.\n");
		return -ENODEV;
	}

	mtd_ecc = kzalloc(sizeof(struct mtd_info),GFP_KERNEL);
	if (!mtd_ecc) {
		err = -ENOMEM; 
		goto init_return_mtd_under;
	}

	mtd_ecc->name = mtd_ecc_name;
	mtd_ecc->type = MTD_NORFLASH;
	printk(KERN_NOTICE "MTD Under flags: 0x%08x\n",mtd_under->flags);
	mtd_ecc->flags = mtd_under->flags; // TODO: Add a switch
	// mtd_ecc->writesize = 3840; // TODO: Re-enable SECT_4K flag for Q7 NOR flash
	mtd_ecc->writesize = mtd_under->writesize; // Using the same as under for now
	mtd_ecc->erasesize = mtd_under->erasesize; // TODO: This needs to be 64k for UBI above
	mtd_ecc->_read = mtd_ecc_read;
	mtd_ecc->_write = mtd_ecc_write;
	mtd_ecc->_erase = mtd_ecc_erase;
  if (mtd_under->_get_device) 
	  mtd_ecc->_get_device = mtd_ecc_get_device;
  if (mtd_under->_put_device)
	  mtd_ecc->_put_device = mtd_ecc_put_device;
	mtd_ecc->size = mtd_under->size;
	//mtd_ecc->size = (mtd->size * 3840) / 4096; // TODO: Verify this
	mtd_ecc->numeraseregions = mtd_under->numeraseregions;
	mtd_ecc->eraseregions = kmalloc(sizeof(struct mtd_erase_region_info)
				    * mtd_ecc->numeraseregions, GFP_KERNEL);
	printk(KERN_NOTICE "configuring %d erase regions\n",mtd_ecc->numeraseregions);
	for (i=0;i<mtd_ecc->numeraseregions;i++) {
		mtd_ecc->eraseregions[i].offset = mtd_under->eraseregions[i].offset;
		mtd_ecc->eraseregions[i].erasesize= mtd_under->eraseregions[i].erasesize;
		mtd_ecc->eraseregions[i].numblocks= mtd_under->eraseregions[i].numblocks;
		if (mtd_under->eraseregions[i].lockmap)
			printk(KERN_WARNING "MTD under has non-null lockmap\n");
		mtd_ecc->eraseregions[i].lockmap= mtd_under->eraseregions[i].lockmap;
	}
	if (mtd_under->_lock)
		mtd_ecc->_lock = mtd_ecc_lock;

	if (mtd_under->_unlock)
		mtd_ecc->_unlock = mtd_ecc_unlock;

	if (mtd_under->_is_locked)
		mtd_ecc->_is_locked = mtd_ecc_is_locked;

	mtd_ecc->dev.parent = &mtd_under->dev;

	mtd_ecc->writebufsize = mtd_under->writebufsize;
	printk(KERN_NOTICE "MTD writebufsize: %d\n",mtd_under->writebufsize);
	
	if (mtd_device_register(mtd_ecc,NULL,0)) {
		printk(KERN_ERR "Cannot add MTD device");
		err = -ENFILE;
		goto init_free_mtd_ecc;
	}
	return 0;
/*
init_free_mtd_ecc_name:
	kfree(mtd_ecc->name);
*/

init_free_mtd_ecc:
	kfree(mtd_ecc);

init_return_mtd_under:
	put_mtd_device(mtd_under);
	
	return err;
}

static void __exit mtdecc_exit(void) 
{

	mtd_device_unregister(mtd_ecc);

}

module_init(mtdecc_init);
module_exit(mtdecc_exit);

/*
MODULE_PARM_DESC(mtd, "MTD devices to overlay");
*/
MODULE_DESCRIPTION("MTD ECC Overlay");
MODULE_AUTHOR("Joshua Lamorie");
MODULE_LICENSE("GPL");

