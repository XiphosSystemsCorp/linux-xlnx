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
#include <linux/bch.h>

static struct mtd_info * mtd_under;

static struct mtd_info * mtd_ecc;

static struct bch_control * mtd_ecc_bch;

static unsigned char * mtd_ecc_eccmask;

static unsigned int * mtd_ecc_errloc;

//static const char * mtd_ecc_name = "MTD_ECC";

#ifdef MTD_ECC_DEBUG
#define DBG_MTDECC(...) DBG_MTDECC(__VA_ARGS__)
#else
#define DBG_MTDECC(...)
#endif

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
	char * mtd_ecc_buf;

  do_div(start_sect,3840LL);
  // NOTE: do_div returns remainder
  do_div(start_sect,0x10000LL);
	start_offset = do_div(from_offset,0xf000);
	DBG_MTDECC(&mtd->dev,"read: from: 0x%llx, len: %d\n",from,len);
 	whole_sect_length = len + start_offset;
	under_sects = (whole_sect_length / 61440) + ((whole_sect_length % 61440) > 0);

#if 0
	if (len > 61440) {
		dev_err(&mtd->dev, "Read length: %d too long.\n",len);
		return -EINVAL;
	}
#endif
	mtd_ecc_buf = kmalloc(0x10000,GFP_KERNEL);
	for (i=0;i<under_sects;i++) {
		size_t buf_out_index = 0;
		size_t buf_in_index = 0;
		size_t buf_cpy_len = 0;
		loff_t under_from;
		size_t under_retlen = 0;
    int j = 0;

		under_from = (start_sect + i) * 0x10000;
		DBG_MTDECC(&mtd->dev,"under->_read : from: %llx\n",under_from);
		if ((rc = mtd_under->_read(mtd_under,under_from,0x10000,&under_retlen,mtd_ecc_buf))<0) {
			dev_err(&mtd->dev,"Problem reading %d bytes from 0x%08llx\n",0x10000,under_from);
			kfree(mtd_ecc_buf);
			return rc;
		}
		DBG_MTDECC(&mtd->dev,"under->_read rc: %d retlen: %d\n",rc,under_retlen);

		if (under_retlen != 0x10000 ) {
			dev_err(&mtd->dev,"Under read. %d instead of %d\n",under_retlen,0x10000);
			kfree(mtd_ecc_buf);
			return -EINVAL;
		}

		// correct data
		for (j=0;j<16;j++) {
			int count;
			unsigned char * read_ecc = &mtd_ecc_buf[0xf000 + (j*0x100)];
      unsigned char * page = &mtd_ecc_buf[j * 0xf00];
      int k;
			
      unsigned char calc_ecc[0x100];

      memset(calc_ecc,0,sizeof(calc_ecc));
      encode_bch(mtd_ecc_bch,page,0xf00,calc_ecc);


      /* apply mask so that an erased page is a valid codeword */
      for (k=0;k<0xff;k++) {
        calc_ecc[k] ^= mtd_ecc_eccmask[k];
      }

			count = decode_bch(mtd_ecc_bch,page,0xf00,read_ecc,calc_ecc,NULL,mtd_ecc_errloc); 

      if (count > 0) {
        for (k=0;k<count;k++) {
          if (mtd_ecc_errloc[k] < (0xf00*8) ) {
            // error is in data, correct it
            page[mtd_ecc_errloc[k] >> 3] ^= (1 << (mtd_ecc_errloc[k] & 7));
          }
          dev_notice(&mtd->dev,"%s: corrected bitflip %u\n",__func__,mtd_ecc_errloc[k]);
        }
      } else if (count < 0) {
        dev_err(&mtd->dev,"ECC unrecoverable error (count:%d)\n",count);
      }

		}


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
		DBG_MTDECC(&mtd->dev,"memcpy %d bytes: 0x%08x -> 0x%08x\n",buf_cpy_len,buf_in_index,buf_out_index);
		// get the data from the requested offset in the buffer
		memcpy(&buf[buf_out_index],&mtd_ecc_buf[buf_in_index],buf_cpy_len);

		*retlen += buf_cpy_len;
	}
	kfree(mtd_ecc_buf);

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
	char * mtd_ecc_buf;

  do_div(start_sect,3840LL);
  // NOTE: do_div returns remainder
	do_div(start_sect,0x10000LL);
	start_offset = do_div(to_offset,0xf000);
	DBG_MTDECC(&mtd->dev,"write : to : 0x%llx, len: %d\n",to,len);
 	whole_sect_length = len + start_offset;
	under_sects = (whole_sect_length / 61440) + ((whole_sect_length % 61440) > 0);

	mtd_ecc_buf = kmalloc(0x10000,GFP_KERNEL);
	for (i=0;i<under_sects;i++) {
		size_t buf_out_index = 0;
		size_t buf_in_index = 0;
		size_t buf_cpy_len = 0;
		loff_t under_to;
		size_t under_retlen = 0;

    int j;


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
				kfree(mtd_ecc_buf);
				return rc;
			}
			DBG_MTDECC(&mtd->dev,"under->_read rc: %d retlen: %d\n",rc,under_retlen);
		}

		// get the data from the requested offset in the buffer
		memcpy(&mtd_ecc_buf[buf_out_index],&buf[buf_in_index],buf_cpy_len);

    // encode the ecc stuff
    for (j=0;j<16;j++) {
      int k;
      unsigned char * code = &mtd_ecc_buf[0xf000 + (j*0x100)];
      unsigned char * page = &mtd_ecc_buf[j*0xf00];
      

      // Clear parity bits to zero
      memset(code,0,0xff);
      encode_bch(mtd_ecc_bch,page,0xf00,code);

      /* apply mask so that an erased page is a valid codeword */
      for (k=0;k<0xff;k++) {
        code[k] ^= mtd_ecc_eccmask[k];
      }
    }


		DBG_MTDECC(&mtd->dev,"under->_write: to 0x%llx\n",under_to);
		if ((rc = mtd_under->_write(mtd_under,under_to,0x10000,&under_retlen,mtd_ecc_buf))<0) {
			dev_err(&mtd->dev,"Problem writing %d bytes to 0x%08llx\n",0x10000,under_to);
			kfree(mtd_ecc_buf);
			return rc;
		}
		if (under_retlen != 0x10000 ) {
			dev_err(&mtd->dev,"Under write. %d instead of %d\n",under_retlen,0x10000);
			kfree(mtd_ecc_buf);
			return -EINVAL;
		}
		*retlen += buf_cpy_len;
	}
	kfree(mtd_ecc_buf);
	
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

		DBG_MTDECC(&mtd->dev,"Erase request: 0x%llx bytes from 0x%llx\n",current_instr->len, current_instr->addr);
		DBG_MTDECC(&mtd->dev,"Translated to: 0x%llx bytes from 0x%llx\n",under_erase.len, under_erase.addr);

		DBG_MTDECC(&mtd->dev,"EI.fail_addr: 0x%llx\n",current_instr->fail_addr);
		DBG_MTDECC(&mtd->dev,"EI.time: 0x%x\n",(unsigned int)current_instr->time);
		DBG_MTDECC(&mtd->dev,"EI.retries: 0x%x\n",(unsigned int)current_instr->retries);
		DBG_MTDECC(&mtd->dev,"EI.dev: 0x%x\n",current_instr->dev);
		DBG_MTDECC(&mtd->dev,"EI.cell: 0x%x\n",current_instr->cell);
		DBG_MTDECC(&mtd->dev,"EI.callback: 0x%p\n",current_instr->callback);
		// clear the callback so we can execute it correctly
		under_erase.callback = NULL;
		DBG_MTDECC(&mtd->dev,"EI.priv: 0x%x\n",(unsigned int)current_instr->priv);
		DBG_MTDECC(&mtd->dev,"EI.state: 0x%x\n",current_instr->state);
		DBG_MTDECC(&mtd->dev,"EI.next: 0x%p\n",current_instr->next);
		under_erase.next = NULL;

		if ((rc = mtd_under->_erase(mtd_under,&under_erase))<0){
			dev_err(&mtd->dev,"Error erasing.\n");
			break;
		}
		DBG_MTDECC(&mtd->dev,"under->_erase returned %d\n",rc);
		DBG_MTDECC(&mtd->dev,"EI.fail_addr: 0x%llx\n",under_erase.fail_addr);
		DBG_MTDECC(&mtd->dev,"EI.time: 0x%x\n",(unsigned int)under_erase.time);
		DBG_MTDECC(&mtd->dev,"EI.retries: 0x%x\n",(unsigned int)under_erase.retries);
		DBG_MTDECC(&mtd->dev,"EI.dev: 0x%x\n",under_erase.dev);
		DBG_MTDECC(&mtd->dev,"EI.cell: 0x%x\n",under_erase.cell);
		DBG_MTDECC(&mtd->dev,"EI.callback: 0x%p\n",under_erase.callback);
		// clear the callback so we can execute it correctly
		under_erase.callback = NULL;
		DBG_MTDECC(&mtd->dev,"EI.priv: 0x%x\n",(unsigned int)under_erase.priv);
		DBG_MTDECC(&mtd->dev,"EI.state: 0x%x\n",under_erase.state);
		DBG_MTDECC(&mtd->dev,"EI.next: 0x%p\n",under_erase.next);
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
	DBG_MTDECC(&mtd->dev,"get_device: %p\n",mtd_under->_get_device);
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
		goto init_free_mtd_ecc;
	}

	mtd_ecc = mtd;


  /* set-up BCH ECC */
  /*
	* examples:
	* docg3.c:
	*  	cascade->bch = init_bch(DOC_ECC_BCH_M,
	*  		DOC_ECC_BCH_T,DOC_ECC_BCH_PRIMPOLY);
	*
	*  #define DOC_ECC_BCH_M                   14
	*  #define DOC_ECC_BCH_T                   4
	*  #define DOC_ECC_BCH_PRIMPOLY            0x4443
	*
	* docg4.c:
	* 	doc->bch = init_bch(DOCG4_M, DOCG4_T, DOCG4_PRIMITIVE_POLY);
	* 	#define DOCG4_M                14  // Galois field is of order 2^14 
	*		#define DOCG4_T                4   // BCH alg corrects up to 4 bit errors
	*		#define DOCG4_PRIMITIVE_POLY   0x4443
	*
	*	nand_bch.c:
	*		nbc->bch = init_bch(m, t, 0);
	*
	*		m = fls(1+8*eccsize);
	*		t = (eccbytes*8)/m;
	*
	*   from nand_base.c has 
	*		eccsize = 512
	*		eccbytes = 7
	*
	*		We are going to have 
	*
	*		eccsize = 3840
	*		eccbytes = 255
	*/
#define MTD_ECC_BCH_M 15
#define MTD_ECC_BCH_T 136
	dev_notice(&mtd->dev,"Starting init_bch\n");
	mtd_ecc_bch = init_bch(MTD_ECC_BCH_M,MTD_ECC_BCH_T,0);
	if (!mtd_ecc_bch) {
		printk(KERN_ERR "Error initializing BCH\n");
    err = -ENOMEM;
		goto init_free_mtd_ecc;
	}
	if (mtd_ecc_bch->ecc_bytes != 255) {
		printk(KERN_ERR "Invalid eccbytes %u, should be 255\n",mtd_ecc_bch->ecc_bytes);
    err = -EINVAL;
		goto init_free_mtd_ecc_bch;
	}
  dev_notice(&mtd->dev,"ecc_bits: %d n:%d (n-ecc_bits:%d)\n",mtd_ecc_bch->ecc_bits,
      mtd_ecc_bch->n,(mtd_ecc_bch->n-mtd_ecc_bch->ecc_bits));
  {
    unsigned char * erased_page;
    mtd_ecc_eccmask = kmalloc(mtd_ecc_bch->ecc_bytes, GFP_KERNEL);
    mtd_ecc_errloc = kmalloc(MTD_ECC_BCH_T*sizeof(unsigned int),GFP_KERNEL);
    if (!mtd_ecc_eccmask || !mtd_ecc_errloc) {
      printk(KERN_ERR "Unable to allocate %d bytes for mtd_ecc_eccmask\n",mtd_ecc_bch->ecc_bytes);
      goto init_free_eccmask;
    }
    erased_page = kmalloc(3840,GFP_KERNEL);
    if (!erased_page) {
      printk(KERN_ERR "Unable to allocate erased_page of %d bytes\n",0xf00);
      goto init_free_eccmask;
    }
    memset(erased_page,0xff,0xf00);
    memset(mtd_ecc_eccmask,0,mtd_ecc_bch->ecc_bytes);
    encode_bch(mtd_ecc_bch,erased_page,0xf00,mtd_ecc_eccmask);
    kfree(erased_page);
    for (i=0;i<mtd_ecc_bch->ecc_bytes;i++) {
      mtd_ecc_eccmask[i] ^= 0xff;
    }
#if 0
    dev_notice(&mtd->dev,"eccmask:\n");
    for (i=0;i<0xff;i+=8) {
      dev_notice(&mtd->dev,"%02x %02x %02x %02x %02x %02x %02x %02x\n",mtd_ecc_eccmask[i],
        mtd_ecc_eccmask[i+1],mtd_ecc_eccmask[i+2],mtd_ecc_eccmask[i+3],
mtd_ecc_eccmask[i+4],mtd_ecc_eccmask[i+5],mtd_ecc_eccmask[i+6],mtd_ecc_eccmask[i+7]);
    }
#endif
  }
  dev_notice(&mtd->dev,"Finished init_bch\n");

	return 0;

init_free_eccmask:
  kfree(mtd_ecc_eccmask);
  kfree(mtd_ecc_errloc);

init_free_mtd_ecc_bch:
	free_bch(mtd_ecc_bch);

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

