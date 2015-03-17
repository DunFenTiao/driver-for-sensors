/*
 * iaq.c: driver for iaq sensor.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/byteorder/generic.h>

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/types.h>/*ssize_t...*/

#include <linux/gpio.h>
#include <linux/interrupt.h>

#include "nl.h"
#include "iaq.h"

#define PINT_PIN_GPIO15 15
#define verbose(x)

static struct i2c_board_info iaq_info = {
    I2C_BOARD_INFO("iaq", 0x5a),
};

struct i2c_client * iaq_i2c_client;

typedef struct _IAQ {
    //uint16_t Clear;
    //uint16_t Proximity;
    int16_t prediction;                 //??格式？int_t? resistance 会长一些？
	int16_t status;          //8 OR 16??
	int16_t resistance;
	int16_t tvoc;
}IAQ;

IAQ iaq_data;
char *data=NULL;

static int iaq_major = 0;
int service = -1;


struct iaq_sensor {
    struct i2c_client   *i2c;
    IAQ sens;
    const char * name;
};
/*---------------iaq-----------------*/

size_t getIaq4(struct i2c_client *client, IAQ* iaq)
{
    verbose(printk("-->%s\n", __func__));

    uint8_t  buffer[10]={10};
    uint8_t bytenum=10;
    size_t ret=0;
      	
	struct i2c_rdwr_ioctl_data ioctl_data;
	struct i2c_msg msgs;

	msgs.addr= 0x5a;
	msgs.len= bytenum;
	msgs.flags=I2C_M_RD;
	msgs.buf= &buffer;

	ioctl_data.nmsgs= 1;
	ioctl_data.msgs= &msgs;
        
	ret=ioctl(i2cfd, I2C_RDWR, &ioctl_data);
		
	//iaq_data.Prediction=(uint16_t)buffer[0];
	iaq_data.Prediction = be16_to_cpup((__be16 *)(buffer[0])<<8|(_be16*)(buffer[1]));
    iaq_data.Statusflag = be16_to_cpup((__be16 *)(buffer[2]));
    iaq_data.Resistance = be16_to_cpup((__be16 *)(buffer[4])); 
    iaq_data.Tvoc = be16_to_cpup((__be16 *)(buffer[7])<<8|(_be16*)(buffer[8]));

    verbose(printk("iaq.prediction=%d\n",iaq_data.Prediciton));
    verbose(printk("iaq.statusflag=%d\n",iaq_data.Statusflag));
    verbose(printk("iaq.Resistance=%d\n",iaq_data.Resistance));
    verbose(printk("iaq.Tvoc=%d\n",iaq_data.Tvoc));
 
    return ret;
}




static int iaq_remove(struct i2c_client *client)
{
    struct iaq_sensor *iaq = i2c_get_clientdata(client);

    printk("iaq: i2c detatch called for %s\n", iaq->name);
    
    disable_irq(gpio_to_irq(PINT_PIN_GPIO15));  
    kfree(iaq);

    return 0;

}

static int iaq_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct iaq_sensor *iaq = NULL;
    iaq = (struct iaq_sensor *) kmalloc (sizeof(struct iaq_sensor), GFP_KERNEL);
    if (!iaq) {
       printk("%s: no enough memory for %d\n",__func__, sizeof(struct iaq_sensor)); 
       return -1;
    }

    iaq->i2c = client;
    iaq->name = id->name;
    i2c_set_clientdata(client, iaq);
    iaq_i2c_client = client; 

    verbose(printk("Enter iaq module! \n"));

   
return 0;
}

/*----------------file_operations-------------------*/


ssize_t iaq_read(struct file *filp,
       char __user *buf, size_t count, loff_t *f_pos)
{
    verbose(printk("-->read_iaq\n"));
   
    /*---------read iaq ------------*/	
    msleep_interruptible(100);
    getiaqALS(iaq_i2c_client,&iaq_data);
    msleep_interruptible(100);
    //getiaqProximity(iaq_i2c_client,&iaq_data);
    getiaqProIntMode(iaq_i2c_client,&iaq_data);    
    msleep_interruptible(100);

    copy_to_user(buf, &iaq_data,sizeof(iaq_data));
   

    return sizeof(iaq_data);

}

ssize_t iaq_write(struct file *filp,
       const char __user *buf, size_t count, loff_t *f_pos)
{
    return 0;
}

/*  file operations :Open & close the device; in fact, there's nothing to do.*/
int iaq_open_release(struct inode *inode, struct file *filp)
{
    verbose(printk("-->%s\n", __func__));
    return 0;
}
static struct file_operations iaq_ops = {
    .owner = THIS_MODULE,
    .open  = iaq_open_release,
    .release = iaq_open_release,
    .read = iaq_read,
    .write = iaq_write,
};


/*------------------i2c_driver ----------------*/
static const struct i2c_device_id iaq_id[] = {
    { "iaq", 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, iaq_id);

static struct i2c_driver iaq_drv = {
    .driver = {
        .name   = "iaq",
    },
       .probe      = iaq_probe,
       .remove     = iaq_remove,
       .id_table   = iaq_id,
};


/*------------------init----------------*/
static struct cdev iaq_dev;

static int iaq_dev_init(void)
{

    int result;
    struct i2c_adapter *i2c_adap;

    dev_t dev = MKDEV(iaq_major, 0);

    verbose(printk("-->%s\n", __func__));

    /* Figure out our device number. */

    if (iaq_major)
       result = register_chrdev_region(dev, 1, "iaq");//need change number 1?
    else {
       result = alloc_chrdev_region(&dev, 0, 1, "iaq");//need change number 1?
       iaq_major = MAJOR(dev);
    }
    if (result < 0) {
       verbose(printk(KERN_WARNING "iaq: unable to get major %d\n", iaq_major));
       return result;
    }

    /* i2c get adapter*/
    i2c_adap=i2c_get_adapter(6);
    iaq_i2c_client=i2c_new_device(i2c_adap,&iaq_info);
    i2c_put_adapter(i2c_adap);
	
    /* Now set up cdev. */
  
    dev = MKDEV(iaq_major, 0);
    
    cdev_init(&iaq_dev, &iaq_ops);
    iaq_dev.owner = THIS_MODULE;
    result = cdev_add(&iaq_dev, dev, 1);
    if (result)
       verbose(printk(KERN_NOTICE "Error %d adding iaq", result));

    service = nl_register_service("TEST", NULL);

    printk("Init iaq module! %d\n", iaq_major);
    return i2c_add_driver(&iaq_drv);
}

static void iaq_dev_cleanup(void)
{
    if (data) kfree(data);
    cdev_del(&iaq_dev);
    unregister_chrdev_region(MKDEV(iaq_major,0),1);	
    nl_remove_service(service);
    /* remove the device from i2c bus */
    i2c_unregister_device(iaq_i2c_client);
    /* remove the driver for i2c device*/
    i2c_del_driver(&iaq_drv);
}

module_init(iaq_dev_init);
module_exit(iaq_dev_cleanup);
MODULE_LICENSE("GPL");
