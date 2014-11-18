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

//static const char * mtd_ecc_name = "MTD_ECC";

static char * mtd_ecc_buf;

static DEFINE_MUTEX(mtd_ecc_buf_mutex);

static int mtd_ecc_lock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	// calculate the number of blocks
	uint64_t lock_len = (len * 4096ULL);
	loff_t lock_ofs = (ofs * 4096LL);
  do_div(lock_len,3840ULL);
  do_div(lock_ofs,3840ULL);
	return(mtd_under->_lock(mtd_under,lock_ofs,lock_len));
}

static int mtd_ecc_unlock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	// calculate the number of blocks
	uint64_t lock_len = (len * 4096ULL);
	loff_t lock_ofs = (ofs * 4096LL);
  do_div(lock_len,3840ULL);
  do_div(lock_ofs,3840ULL);
	return(mtd_under->_unlock(mtd_under,lock_ofs,lock_len));
}

static int mtd_ecc_is_locked(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	// calculate the number of blocks
	uint64_t lock_len = (len * 4096ULL);
	loff_t lock_ofs = (ofs * 4096LL);
  do_div(lock_len,3840ULL);
  do_div(lock_ofs,3840ULL);
	return(mtd_under->_is_locked(mtd_under,lock_ofs,lock_len));
}
/**
 * mtd_ecc_read - read operation of ECC encapsulated MTD devices.
 * @mtd: MTD device description object
 * @from: absolute offset from where to read
 * @len: how many bytes to read
 * @retlen: count of read bytes is returned here
 * @buf: buffer to store the read data
 *
 *( This function returns zero in case of success and a negative error code in
 * case of failure.
 */
static int mtd_ecc_read(struct mtd_info *mtd, loff_t from, size_t len,
		       size_t *retlen, unsigned char *buf)
{
	loff_t start_sect = from * 4096LL;
	loff_t from_offset = from;
	size_t start_offset;
	size_t under_sects;
	size_t whole_sect_length;
	size_t read_remaining = len;
	int rc,i;

  do_div(start_sect,3840LL);
  // NOTE: do_div returns remainder
  do_div(start_sect,0x10000LL);
	start_offset = do_div(from_offset,0xf000);
	dev_dbg(&mtd->dev,"read: from: 0x%llx, len: %d\n",from,len);
 	whole_sect_length = len + start_offset;
	under_sects = (whole_sect_length / 61440) + ((whole_sect_length % 61440) > 0);

#if 0
	if (len > 61440) {
		dev_err(&mtd->dev, "Read length: %d too long.\n",len);
		return -EINVAL;
	}
#endif
	mutex_lock(&mtd_ecc_buf_mutex);
	for (i=0;i<under_sects;i++) {
		size_t buf_out_index = 0;
		size_t buf_in_index = 0;
		size_t buf_cpy_len = 0;
		loff_t under_from;
		size_t under_retlen = 0;

		under_from = (start_sect + i) * 0x10000;
		dev_dbg(&mtd->dev,"under->_read : from: %llx\n",under_from);
		if ((rc = mtd_under->_read(mtd_under,under_from,0x10000,&under_retlen,mtd_ecc_buf))<0) {
			dev_err(&mtd->dev,"Problem reading %d bytes from 0x%08llx\n",0x10000,under_from);
			mutex_unlock(&mtd_ecc_buf_mutex);
			return rc;
		}
		dev_dbg(&mtd->dev,"under->_read rc: %d retlen: %d\n",rc,under_retlen);

		if (under_retlen != 0x10000 ) {
			dev_err(&mtd->dev,"Under read. %d instead of %d\n",under_retlen,0x10000);
			mutex_unlock(&mtd_ecc_buf_mutex);
			return -EINVAL;
		}
#if 0
		if ((start_offset + len) > 0x10000) {
			dev_err(&mtd->dev,"Trying read beyond input buffer. %lld + %d\n",start_offset,len);
			return -EINVAL;
		}
#endif

		buf_out_index = i * 0xf000;
		if (i == 0) {
			buf_in_index = start_offset;
		} else {
			buf_in_index = 0;
		}
		if (read_remaining > 0xf000) {
			buf_cpy_len = 0xf000 - buf_in_index;
		} else {
			buf_cpy_len = read_remaining;
		}
		read_remaining -= buf_cpy_len;
		dev_dbg(&mtd->dev,"memcpy %d bytes: 0x%08x -> 0x%08x\n",buf_cpy_len,buf_in_index,buf_out_index);
		// get the data from the requested offset in the buffer
		memcpy(&buf[buf_out_index],&mtd_ecc_buf[buf_in_index],buf_cpy_len);

		*retlen += buf_cpy_len;
	}
	mutex_unlock(&mtd_ecc_buf_mutex);

	return 0;
}
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
	loff_t start_sect = to * 4096LL;
	loff_t to_offset = to;
	size_t start_offset;
	size_t under_sects;
	size_t whole_sect_length;
	size_t write_remaining = len;
	int rc,i;

  do_div(start_sect,3840LL);
  // NOTE: do_div returns remainder
	do_div(start_sect,0x10000LL);
	start_offset = do_div(to_offset,0xf000);
	dev_dbg(&mtd->dev,"write : to : 0x%llx, len: %d\n",to,len);
 	whole_sect_length = len + start_offset;
	under_sects = (whole_sect_length / 61440) + ((whole_sect_length % 61440) > 0);

	mutex_lock(&mtd_ecc_buf_mutex);
	for (i=0;i<under_sects;i++) {
		size_t buf_out_index = 0;
		size_t buf_in_index = 0;
		size_t buf_cpy_len = 0;
		loff_t under_to;
		size_t under_retlen = 0;




		buf_in_index = i * 0xf000;
		if (i == 0) {
			buf_out_index = start_offset;
		} else {
			buf_out_index = 0;
		}
		if (write_remaining > 0xf000) {
			buf_cpy_len = 0xf000 - buf_out_index;
		} else {
			buf_cpy_len = write_remaining;
		}
		write_remaining -= buf_cpy_len;

		under_to = (start_sect + i) * 0x10000;
	
		if (buf_cpy_len < 0xf000) {
			// We have to read the sector so we can calculate the BCH
			if ((rc = mtd_under->_read(mtd_under,under_to,0x10000,&under_retlen,mtd_ecc_buf))<0) {
				dev_err(&mtd->dev,"Problem reading %d bytes from 0x%08llx\n",0x10000,under_to);
				mutex_unlock(&mtd_ecc_buf_mutex);
				return rc;
			}
			dev_dbg(&mtd->dev,"under->_read rc: %d retlen: %d\n",rc,under_retlen);
		}

		// get the data from the requested offset in the buffer
		memcpy(&mtd_ecc_buf[buf_out_index],&buf[buf_in_index],buf_cpy_len);

		dev_dbg(&mtd->dev,"under->_write: to 0x%llx\n",under_to);
		if ((rc = mtd_under->_write(mtd_under,under_to,0x10000,&under_retlen,mtd_ecc_buf))<0) {
			dev_err(&mtd->dev,"Problem writing %d bytes to 0x%08llx\n",0x10000,under_to);
			mutex_unlock(&mtd_ecc_buf_mutex);
			return rc;
		}
		if (under_retlen != 0x10000 ) {
			dev_err(&mtd->dev,"Under write. %d instead of %d\n",under_retlen,0x10000);
			mutex_unlock(&mtd_ecc_buf_mutex);
			return -EINVAL;
		}
		*retlen += buf_cpy_len;
	}
	mutex_unlock(&mtd_ecc_buf_mutex);
	
	return 0;
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
	struct erase_info under_erase;
	struct erase_info * current_instr = instr;
	int rc=0;

	while (current_instr) {

		memcpy(&under_erase,current_instr,sizeof(struct erase_info));
		
		under_erase.mtd = mtd_under;

		// let's assume that current_instr->address will always be a whole sector
		under_erase.addr = current_instr->addr * 4096ULL;
		do_div(under_erase.addr,3840ULL);
		under_erase.len = current_instr->len * 4096ULL;
		do_div(under_erase.len,3840ULL);

		dev_dbg(&mtd->dev,"Erase request: 0x%llx bytes from 0x%llx\n",current_instr->len, current_instr->addr);
		dev_dbg(&mtd->dev,"Translated to: 0x%llx bytes from 0x%llx\n",under_erase.len, under_erase.addr);

		dev_dbg(&mtd->dev,"EI.fail_addr: 0x%llx\n",current_instr->fail_addr);
		dev_dbg(&mtd->dev,"EI.time: 0x%x\n",(unsigned int)current_instr->time);
		dev_dbg(&mtd->dev,"EI.retries: 0x%x\n",(unsigned int)current_instr->retries);
		dev_dbg(&mtd->dev,"EI.dev: 0x%x\n",current_instr->dev);
		dev_dbg(&mtd->dev,"EI.cell: 0x%x\n",current_instr->cell);
		dev_dbg(&mtd->dev,"EI.callback: 0x%p\n",current_instr->callback);
		// clear the callback so we can execute it correctly
		under_erase.callback = NULL;
		dev_dbg(&mtd->dev,"EI.priv: 0x%x\n",(unsigned int)current_instr->priv);
		dev_dbg(&mtd->dev,"EI.state: 0x%x\n",current_instr->state);
		dev_dbg(&mtd->dev,"EI.next: 0x%p\n",current_instr->next);
		under_erase.next = NULL;

		if ((rc = mtd_under->_erase(mtd_under,&under_erase))<0){
			dev_err(&mtd->dev,"Error erasing.\n");
			break;
		}
		dev_dbg(&mtd->dev,"under->_erase returned %d\n",rc);
		dev_dbg(&mtd->dev,"EI.fail_addr: 0x%llx\n",under_erase.fail_addr);
		dev_dbg(&mtd->dev,"EI.time: 0x%x\n",(unsigned int)under_erase.time);
		dev_dbg(&mtd->dev,"EI.retries: 0x%x\n",(unsigned int)under_erase.retries);
		dev_dbg(&mtd->dev,"EI.dev: 0x%x\n",under_erase.dev);
		dev_dbg(&mtd->dev,"EI.cell: 0x%x\n",under_erase.cell);
		dev_dbg(&mtd->dev,"EI.callback: 0x%p\n",under_erase.callback);
		// clear the callback so we can execute it correctly
		under_erase.callback = NULL;
		dev_dbg(&mtd->dev,"EI.priv: 0x%x\n",(unsigned int)under_erase.priv);
		dev_dbg(&mtd->dev,"EI.state: 0x%x\n",under_erase.state);
		dev_dbg(&mtd->dev,"EI.next: 0x%p\n",under_erase.next);
			// copy in the state
		current_instr->state = under_erase.state;
		mtd_erase_callback(current_instr);
		
		
		current_instr = current_instr->next;
	}
	return rc;
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
	dev_dbg(&mtd->dev,"get_device: %p\n",mtd_under->_get_device);
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
  struct mtd_info * mtd;
#define MTD_NUM 39

	mtd_under = get_mtd_device(NULL, MTD_NUM);
  if (!mtd_under) {
		printk(KERN_ERR "Unable to get MTD device 39.\n");
		return -ENODEV;
	}

	mtd = kzalloc(sizeof(struct mtd_info),GFP_KERNEL);
	if (!mtd) {
		err = -ENOMEM; 
		goto init_return_mtd_under;
	}

	// TODO: get the length from somewhere else... and it should be 0x1000
	mtd_ecc_buf = kzalloc(0x10000,GFP_KERNEL);
	if (!mtd_ecc_buf) {
		err = -ENOMEM;
		goto init_free_mtd_ecc;
	}

	mtd->name = "MTD ECC";
	mtd->type = MTD_ECCNORFLASH;
	printk(KERN_NOTICE "MTD Under flags: 0x%08x\n",mtd_under->flags);
	mtd->flags = mtd_under->flags; // TODO: Add a switch
	// mtd->writesize = 3840; // TODO: Re-enable SECT_4K flag for Q7 NOR flash
	mtd->erasesize = (mtd_under->erasesize * 3840)/4096; // Using the same as under for now
	printk(KERN_NOTICE "Setting erase/writesize to %d\n",mtd->erasesize);
	mtd->writesize = 256; // TODO: This needs to be 64k for UBI above
	mtd->_read = mtd_ecc_read;
	mtd->_write = mtd_ecc_write;
	mtd->_erase = mtd_ecc_erase;
  if (mtd_under->_get_device) 
	  mtd->_get_device = mtd_ecc_get_device;
  if (mtd_under->_put_device)
	  mtd->_put_device = mtd_ecc_put_device;
	mtd->size = mtd_under->size * 3840ULL;
  do_div(mtd->size, 4096ULL);
	//mtd->size = (mtd->size * 3840) / 4096; // TODO: Verify this
	mtd->numeraseregions = mtd_under->numeraseregions;
	mtd->eraseregions = kmalloc(sizeof(struct mtd_erase_region_info)
				    * mtd->numeraseregions, GFP_KERNEL);
	printk(KERN_NOTICE "configuring %d erase regions\n",mtd->numeraseregions);
	for (i=0;i<mtd->numeraseregions;i++) {
		mtd->eraseregions[i].offset = mtd_under->eraseregions[i].offset;
		mtd->eraseregions[i].erasesize= mtd_under->eraseregions[i].erasesize;
		mtd->eraseregions[i].numblocks= mtd_under->eraseregions[i].numblocks;
		if (mtd_under->eraseregions[i].lockmap)
			printk(KERN_WARNING "MTD under has non-null lockmap\n");
		mtd->eraseregions[i].lockmap= mtd_under->eraseregions[i].lockmap;
	}
	if (mtd_under->_lock)
		mtd->_lock = mtd_ecc_lock;

	if (mtd_under->_unlock)
		mtd->_unlock = mtd_ecc_unlock;

	if (mtd_under->_is_locked)
		mtd->_is_locked = mtd_ecc_is_locked;

	mtd->dev.parent = &mtd_under->dev;

	mtd->writebufsize = 256;
	printk(KERN_NOTICE "MTD under writebufsize: %d\n",mtd_under->writebufsize);
	
	if (mtd_device_register(mtd,NULL,0)) {
		printk(KERN_ERR "Cannot add MTD device");
		err = -ENFILE;
		goto init_free_mtd_ecc_buf;
	}

	mtd_ecc = mtd;

	return 0;
/*
init_free_mtd_ecc_name:
	kfree(mtd_ecc->name);
*/
init_free_mtd_ecc_buf:
	kfree(mtd_ecc_buf);

init_free_mtd_ecc:
	kfree(mtd);

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

