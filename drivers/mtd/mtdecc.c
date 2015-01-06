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


struct mtd_ecc_info {

	/* MTD device provided by instance */
	struct mtd_info * mtd_ecc;

	/* MTD device over which the MTD ECC is attached */
	struct mtd_info * mtd_under;

	/* BCH structures */
	// TODO: Add mutex
	struct bch_control * bch;
	unsigned char * eccmask;
	unsigned int * errloc;

	/* MTD ECC sector sizes */

#define MTD_ECC_PSIZE (4096)
#define MTD_ECC_DSIZE (3840)
#define MTD_ECC_PSIZEULL (4096ULL)
#define MTD_ECC_PSIZELL (4096LL)
#define MTD_ECC_DSIZEULL (3840ULL)
#define MTD_ECC_DSIZELL (3840LL)
#define MTD_ECC_ESIZE (256)

	unsigned int sect_psize;
	unsigned int sect_dsize;
	unsigned int sect_esize;

	loff_t under_ofs;
	uint64_t under_len;
	size_t under_sects;
	
	
	

	char * rd_ecc_buf;
	char * wr_ecc_buf;

  struct list_head list;

};

struct list_head me_list;
LIST_HEAD(me_list);


struct mtd_ecc_under_req {
	loff_t start_sect;
	size_t under_sects;
	size_t whole_sect_length;
	size_t start_offset;
};

//static const char * mtd_ecc_name = "MTD_ECC";

#ifdef MTD_ECC_DEBUG
#define DBG_MTDECC(...) DBG_MTDECC(__VA_ARGS__)
#else
#define DBG_MTDECC(...)
#endif

static inline void mtd_ecc_calc_lock_off(struct mtd_ecc_info * me, loff_t ofs, uint64_t len) {
	
	me->under_len = (len * MTD_ECC_PSIZEULL);
  do_div(me->under_len,MTD_ECC_DSIZEULL);
	me->under_ofs = (ofs * MTD_ECC_PSIZELL);
  do_div(me->under_ofs,MTD_ECC_DSIZEULL);
}

static int mtd_ecc_lock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	struct mtd_ecc_info * me = (struct mtd_ecc_info *)mtd->priv;
	mtd_ecc_calc_lock_off(me,ofs,len);
	return(me->mtd_under->_lock(me->mtd_under,me->under_ofs,me->under_len));
}

static int mtd_ecc_unlock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	struct mtd_ecc_info * me = (struct mtd_ecc_info *)mtd->priv;
	mtd_ecc_calc_lock_off(me,ofs,len);
	return(me->mtd_under->_unlock(me->mtd_under,me->under_ofs,me->under_len));
}

static int mtd_ecc_is_locked(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	struct mtd_ecc_info * me = (struct mtd_ecc_info *)mtd->priv;
	mtd_ecc_calc_lock_off(me,ofs,len);
	return(me->mtd_under->_is_locked(me->mtd_under,me->under_ofs,me->under_len));
}

static inline void mtd_ecc_calc_under_sect(struct mtd_ecc_info * me, struct mtd_ecc_under_req * mureq, loff_t ofs,size_t len) {
	loff_t offset = ofs;
	size_t whole_sect_length;

	// Calculate the physical start byte to be read from mtd_under
	// start_sect = (ofs * psize)/dsize
 	mureq->start_sect = ofs * MTD_ECC_PSIZELL;
  do_div(mureq->start_sect,MTD_ECC_DSIZELL);
	// Turn the start_byte into a sector
  do_div(mureq->start_sect,MTD_ECC_PSIZEULL);
	// Calculate the offset within the first physical sector
	mureq->start_offset = do_div(offset,MTD_ECC_DSIZEULL);
 	whole_sect_length = len + mureq->start_offset;
	mureq->under_sects = (whole_sect_length / MTD_ECC_DSIZE) + ((whole_sect_length % MTD_ECC_DSIZE) > 0);

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
	size_t read_remaining = len;
	int rc,i;
	struct mtd_ecc_under_req mureq;
	struct mtd_ecc_info * me = (struct mtd_ecc_info *)mtd->priv;

	mtd_ecc_calc_under_sect(me,&mureq,from,len);
	

#if 0
	if (len > 61440) {
		dev_err(&mtd->dev, "Read length: %d too long.\n",len);
		return -EINVAL;
	}
#endif
  /* TODO: To increase performance, the reads from the lower device should be
 * as large as possible and then perform the ECC.
 * In an ideal world we would somehow schedule the underlying read to happen
 * while performing the ECC calcs in the 'foreground' */
	for (i=0;i<mureq.under_sects;i++) {
		size_t buf_out_index = 0;
		size_t buf_in_index = 0;
		size_t buf_cpy_len = 0;
		loff_t under_from;
		size_t under_retlen = 0;

		under_from = (mureq.start_sect + i) * MTD_ECC_PSIZE;
		DBG_MTDECC(&mtd->dev,"under->_read : from: %llx\n",under_from);
		if ((rc = me->mtd_under->_read(me->mtd_under,under_from,MTD_ECC_PSIZE,&under_retlen,me->rd_ecc_buf))<0) {
			dev_err(&mtd->dev,"Problem reading %d bytes from 0x%08llx\n",MTD_ECC_PSIZE,under_from);
			return rc;
		}
		DBG_MTDECC(&mtd->dev,"under->_read rc: %d retlen: %d\n",rc,under_retlen);

		if (under_retlen != MTD_ECC_PSIZE) {
			dev_err(&mtd->dev,"Under read. %d instead of %d\n",under_retlen,MTD_ECC_PSIZE);
			return -EINVAL;
		}

		// correct data
		{
			int count;
			unsigned char * read_ecc = &me->rd_ecc_buf[MTD_ECC_DSIZE];
			unsigned char * page = me->rd_ecc_buf;
			int k;
      unsigned char calc_ecc[MTD_ECC_ESIZE];

      memset(calc_ecc,0,sizeof(calc_ecc));
      encode_bch(me->bch,page,MTD_ECC_DSIZE,calc_ecc);

      /* apply mask so that an erased page is a valid codeword */
      for (k=0;k<0xff;k++) {
        calc_ecc[k] ^= me->eccmask[k];
      }

			count = decode_bch(me->bch,page,MTD_ECC_DSIZE,read_ecc,calc_ecc,NULL,me->errloc); 

      if (count > 0) {
        for (k=0;k<count;k++) {
          if (me->errloc[k] < (MTD_ECC_DSIZE*8) ) {
            // error is in data, correct it
            page[me->errloc[k] >> 3] ^= (1 << (me->errloc[k] & 7));
          }
          dev_notice(&mtd->dev,"%s: corrected bitflip %u\n",__func__,me->errloc[k]);
        }
      } else if (count < 0) {
        dev_err(&mtd->dev,"ECC unrecoverable error (count:%d)\n",count);
      }
		}

		buf_out_index = i * MTD_ECC_DSIZE;
		if (i == 0) {
			buf_in_index = mureq.start_offset;
		} else {
			buf_in_index = 0;
		}
		if (read_remaining > MTD_ECC_DSIZE) {
			buf_cpy_len = MTD_ECC_DSIZE - buf_in_index;
		} else {
			buf_cpy_len = read_remaining;
		}
		read_remaining -= buf_cpy_len;
		DBG_MTDECC(&mtd->dev,"memcpy %d bytes: 0x%08x -> 0x%08x\n",buf_cpy_len,buf_in_index,buf_out_index);
		// get the data from the requested offset in the buffer
		memcpy(&buf[buf_out_index],&me->rd_ecc_buf[buf_in_index],buf_cpy_len);

		*retlen += buf_cpy_len;
	}

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
	size_t write_remaining = len;
	int rc,i;
	struct mtd_ecc_under_req mureq;
	struct mtd_ecc_info * me = (struct mtd_ecc_info *)mtd->priv;

	mtd_ecc_calc_under_sect(me,&mureq,to,len);
	
	for (i=0;i<mureq.under_sects;i++) {
		size_t buf_out_index = 0;
		size_t buf_in_index = 0;
		size_t buf_cpy_len = 0;
		loff_t under_to;
		size_t under_retlen = 0;

		buf_in_index = i * MTD_ECC_DSIZE;
		if (i == 0) {
			buf_out_index = mureq.start_offset;
		} else {
			buf_out_index = 0;
		}
		if (write_remaining > MTD_ECC_DSIZE) {
			buf_cpy_len = MTD_ECC_DSIZE - buf_out_index;
		} else {
			buf_cpy_len = write_remaining;
		}
		write_remaining -= buf_cpy_len;

		under_to = (mureq.start_sect + i) * MTD_ECC_PSIZE;
	
		if (buf_cpy_len < MTD_ECC_DSIZE) {
			// We have to read the sector so we can calculate the BCH
			if ((rc = me->mtd_under->_read(me->mtd_under,under_to,MTD_ECC_PSIZE,&under_retlen,me->wr_ecc_buf))<0) {
				dev_err(&mtd->dev,"Problem reading %d bytes from 0x%08llx\n",MTD_ECC_PSIZE,under_to);
				return rc;
			}
			DBG_MTDECC(&mtd->dev,"under->_read rc: %d retlen: %d\n",rc,under_retlen);
		}

		// get the data from the requested offset in the buffer
		memcpy(&me->wr_ecc_buf[buf_out_index],&buf[buf_in_index],buf_cpy_len);

    // encode the ecc stuff
    {
      int k;
      unsigned char * code = &me->wr_ecc_buf[MTD_ECC_DSIZE];
      unsigned char * page = me->wr_ecc_buf;
      

      // Clear parity bits to zero
      memset(code,0,0xff);
      encode_bch(me->bch,page,MTD_ECC_DSIZE,code);

      /* apply mask so that an erased page is a valid codeword */
      for (k=0;k<0xff;k++) {
        code[k] ^= me->eccmask[k];
      }
    }


		DBG_MTDECC(&mtd->dev,"under->_write: to 0x%llx\n",under_to);
		if ((rc = me->mtd_under->_write(me->mtd_under,under_to,MTD_ECC_PSIZE,&under_retlen,me->wr_ecc_buf))<0) {
			dev_err(&mtd->dev,"Problem writing %d bytes to 0x%08llx\n",MTD_ECC_PSIZE,under_to);
			return rc;
		}
		if (under_retlen != MTD_ECC_PSIZE) {
			dev_err(&mtd->dev,"Under write. %d instead of %d\n",under_retlen,MTD_ECC_PSIZE);
			return -EINVAL;
		}
		*retlen += buf_cpy_len;
	}
	
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
	struct mtd_ecc_info * me = (struct mtd_ecc_info *)mtd->priv;

	while (current_instr) {

		memcpy(&under_erase,current_instr,sizeof(struct erase_info));
		
		under_erase.mtd = me->mtd_under;

		// let's assume that current_instr->address will always be a whole sector
		under_erase.addr = current_instr->addr * MTD_ECC_PSIZEULL;
		do_div(under_erase.addr,MTD_ECC_DSIZEULL);
		under_erase.len = current_instr->len * MTD_ECC_PSIZEULL;
		do_div(under_erase.len,MTD_ECC_DSIZEULL);

		DBG_MTDECC(&mtd->dev,"Erase request: 0x%llx bytes from 0x%llx\n",current_instr->len, current_instr->addr);
		DBG_MTDECC(&mtd->dev,"Translated to: 0x%llx bytes from 0x%llx\n",under_erase.len, under_erase.addr);

		// clear the callback so we can execute it correctly
		under_erase.callback = NULL;
		under_erase.next = NULL;

		if ((rc = me->mtd_under->_erase(me->mtd_under,&under_erase))<0){
			dev_err(&mtd->dev,"Error erasing.\n");
			break;
		}
		// clear the callback so we can execute it correctly
		under_erase.callback = NULL;
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
	struct mtd_ecc_info * me = (struct mtd_ecc_info *)mtd->priv;
	if (me->mtd_under->_get_device) {
		DBG_MTDECC(&mtd->dev,"get_device: %p\n",me->mtd_under->_get_device);
		return(me->mtd_under->_get_device(me->mtd_under));
	} else {
		return -ENOSYS;
	}
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
	struct mtd_ecc_info * me = (struct mtd_ecc_info *)mtd->priv;
	return(me->mtd_under->_put_device(me->mtd_under));
}

static int __init mtdecc_init(void)
{
	int err = 0;
  int i;
  struct mtd_info * mtd;
	struct mtd_ecc_info * me;

	me = kzalloc(sizeof(struct mtd_ecc_info),GFP_KERNEL);
	if (!me) {
		printk(KERN_ERR "Unable to allocate memory for mtd_ecc_info\n");
		return -ENOMEM;
	}
	me->sect_psize = MTD_ECC_PSIZE;
	me->sect_dsize = MTD_ECC_DSIZE;
	me->sect_esize = MTD_ECC_ESIZE;

	me->rd_ecc_buf = kzalloc(me->sect_psize,GFP_KERNEL);
	if (!me->rd_ecc_buf) {
		err = -ENOMEM;
		goto init_return_me_alloc;
	}
	me->wr_ecc_buf = kzalloc(me->sect_psize,GFP_KERNEL);
	if (!me->wr_ecc_buf) {
		err = -ENOMEM;
		goto init_return_rd_buf_alloc;
	}

// TODO: Get this from parameters
#define MTD_NUM 39

	me->mtd_under = get_mtd_device(NULL, MTD_NUM);
  if (!me->mtd_under) {
		printk(KERN_ERR "Unable to get MTD device 39.\n");
		err = -ENODEV;
		goto init_return_wr_buf_alloc;
	}

	mtd = kzalloc(sizeof(struct mtd_info),GFP_KERNEL);
	if (!mtd) {
		err = -ENOMEM; 
		goto init_return_mtd_under;
	}
	mtd->priv = (void *)me;
	mtd->name = "MTD ECC";
	mtd->type = MTD_ECCNORFLASH;
	printk(KERN_NOTICE "MTD Under flags: 0x%08x\n",me->mtd_under->flags);
	mtd->flags = me->mtd_under->flags; // TODO: Add a switch
	// mtd->writesize = 3840; // TODO: Re-enable SECT_4K flag for Q7 NOR flash
	mtd->erasesize = (me->mtd_under->erasesize * 3840)/4096; // Using the same as under for now
	printk(KERN_NOTICE "Setting erase/writesize to %d\n",mtd->erasesize);
	mtd->writesize = 256; // TODO: This needs to be 64k for UBI above
	mtd->_read = mtd_ecc_read;
	mtd->_write = mtd_ecc_write;
	mtd->_erase = mtd_ecc_erase;

	//====================================================================
	// TODO: Is this wise? This is probably going to conflict with the
	// get_mtd_device() we just called above
  if (me->mtd_under->_get_device) 
	  mtd->_get_device = mtd_ecc_get_device;
  if (me->mtd_under->_put_device)
	  mtd->_put_device = mtd_ecc_put_device;
	//====================================================================


	//mtd->size = (mtd->size * 3840) / 4096; 
	mtd->size = me->mtd_under->size * me->sect_dsize;
  do_div(mtd->size, (uint64_t)me->sect_psize);

	mtd->numeraseregions = me->mtd_under->numeraseregions;
	mtd->eraseregions = kmalloc(sizeof(struct mtd_erase_region_info)
				    * mtd->numeraseregions, GFP_KERNEL);
	printk(KERN_NOTICE "configuring %d erase regions\n",mtd->numeraseregions);
	for (i=0;i<mtd->numeraseregions;i++) {
		mtd->eraseregions[i].offset = me->mtd_under->eraseregions[i].offset;
		mtd->eraseregions[i].erasesize= me->mtd_under->eraseregions[i].erasesize;
		mtd->eraseregions[i].numblocks= me->mtd_under->eraseregions[i].numblocks;
		if (me->mtd_under->eraseregions[i].lockmap) {
			printk(KERN_WARNING "MTD under has non-null lockmap\n");
			mtd->eraseregions[i].lockmap=me->mtd_under->eraseregions[i].lockmap;
		}
	}
	if (me->mtd_under->_lock)
		mtd->_lock = mtd_ecc_lock;

	if (me->mtd_under->_unlock)
		mtd->_unlock = mtd_ecc_unlock;

	if (me->mtd_under->_is_locked)
		mtd->_is_locked = mtd_ecc_is_locked;

	mtd->dev.parent = &me->mtd_under->dev;

	mtd->writebufsize = 256;
	printk(KERN_NOTICE "MTD under writebufsize: %d\n",me->mtd_under->writebufsize);
	
	if (mtd_device_register(mtd,NULL,0)) {
		printk(KERN_ERR "Cannot add MTD device");
		err = -ENFILE;
		goto init_free_mtd_ecc;
	}

	me->mtd_ecc = mtd;


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
	dev_notice(&me->mtd_ecc->dev,"Starting init_bch\n");
	me->bch = init_bch(MTD_ECC_BCH_M,MTD_ECC_BCH_T,0);
	if (!me->bch) {
		printk(KERN_ERR "Error initializing BCH\n");
    err = -ENOMEM;
		goto init_free_mtd_ecc;
	}
	if (me->bch->ecc_bytes != 255) {
		printk(KERN_ERR "Invalid eccbytes %u, should be 255\n",me->bch->ecc_bytes);
    err = -EINVAL;
		goto init_free_mtd_ecc_bch;
	}
  dev_notice(&mtd->dev,"ecc_bits: %d n:%d (n-ecc_bits:%d)\n",me->bch->ecc_bits,
      me->bch->n,(me->bch->n - me->bch->ecc_bits));
  {
    unsigned char * erased_page;
    me->eccmask = kmalloc(me->bch->ecc_bytes, GFP_KERNEL);
    me->errloc = kmalloc(MTD_ECC_BCH_T*sizeof(unsigned int),GFP_KERNEL);
    if (!me->eccmask || !me->errloc) {
      printk(KERN_ERR "Unable to allocate %d bytes for mtd_ecc_eccmask\n",me->bch->ecc_bytes);
      goto init_free_eccmask;
    }
    erased_page = kmalloc(me->sect_dsize,GFP_KERNEL);
    if (!erased_page) {
      printk(KERN_ERR "Unable to allocate erased_page of %d bytes\n",me->sect_dsize);
      goto init_free_eccmask;
    }
    memset(erased_page,0xff,me->sect_dsize);
    memset(me->eccmask,0,me->bch->ecc_bytes);
    encode_bch(me->bch,erased_page,me->sect_dsize,me->eccmask);
    kfree(erased_page);
    for (i=0;i<me->bch->ecc_bytes;i++) {
      me->eccmask[i] ^= 0xff;
    }
#if 0
    dev_notice(&mtd->dev,"eccmask:\n");
    for (i=0;i<0xff;i+=8) {
      dev_notice(&mtd->dev,"%02x %02x %02x %02x %02x %02x %02x %02x\n",me->eccmask[i],
				me->eccmask[i+1],me->eccmask[i+2],me->eccmask[i+3],
				me->eccmask[i+4],me->eccmask[i+5],me->eccmask[i+6],me->eccmask[i+7]);
    }
#endif
  }
  dev_notice(&mtd->dev,"Finished init_bch\n");

  list_add(&me->list,&me_list);


	return 0;



init_free_eccmask:
  kfree(me->eccmask);
  kfree(me->errloc);

init_free_mtd_ecc_bch:
	free_bch(me->bch);

init_free_mtd_ecc:
	kfree(mtd);

init_return_mtd_under:
	put_mtd_device(me->mtd_under);

init_return_wr_buf_alloc:
	kfree(me->wr_ecc_buf);

init_return_rd_buf_alloc:
	kfree(me->rd_ecc_buf);

init_return_me_alloc:
	kfree(me);
	
	return err;
}

static void __exit mtdecc_exit(void) 
{
  struct list_head *m;

	list_for_each(m, &me_list) {
		struct mtd_ecc_info * me;
		me = list_entry(m,struct mtd_ecc_info, list);
		mtd_device_unregister(me->mtd_ecc);
	}

}

module_init(mtdecc_init);
module_exit(mtdecc_exit);

/*
MODULE_PARM_DESC(mtd, "MTD devices to overlay");
*/
MODULE_DESCRIPTION("MTD ECC Overlay");
MODULE_AUTHOR("Joshua Lamorie");
MODULE_LICENSE("GPL");

