#Device Driver Assignment 

##Consists of three parts:

###Part1: Device Driver Source Code name motor1.c
-------
#####Supports these services:
+ static ssize_t motor_stop(struct file *file) <br />
+ static ssize_t motor_rotate(struct file *file, int direction) <br />
+ static int motor_open(struct inode *inode, struct file *file) <br />
+ static int motor_release(struct inode *inode, struct file *file) <br />
+ static ssize_t motor_read(struct file *file, char *buf, size_t count,   loff_t *ptr) <br />
+ static ssize_t motor_write(struct file *file, const char *buf,  size_t count, loff_t * ppos) <br />
+ static int motor_ioctl(struct inode *inode, struct file *file,  unsigned int cmd, unsigned long arg) <br />

```c
    /***********************************************************************
    * Sample Code, BeagleBoneBlack Device Driver
    *
    * DESCRIPTION :
    *        Device driver that moves a motor   
    * 
    * AUTHOR :    Luanna Ochoa         
    *
    * COURSE :    Embedded Operating Systems at 
    *             Florida International University 
    * CODE OUTLINE :
    *          Mutex Enabeled Throughout 
    *          Includes
    *          Macro Definitions
    *          Variable Declarations/initilizations 
    *          Function/Struct Prototypes
    *          Init Function 
    *          Exit Function 
    *          User Defined Functions        
    *          
    *************************************************************************/
             
    #include <linux/init.h> //Allows use of init macros
    #include <linux/module.h> //Needed by all modules 
    #include <linux/device.h> //supports module.h header
    #include <linux/kernel.h> //Needed for KERN_INFO
    #include <linux/fs.h> //FS support
    #include <asm/uaccess.h>
    #include <linux/mutex.h> //enables multiuser locks
    #include <liknux/errno.h>
    #include <linux/types.h> // alias for data types, allows for dd portability
     #include <linux/kdev_t.h> //ditto ^
    #include <linux/ioctl.h> //short for input output control; used for sending info from process to kernel and vice a versa 
    #include <linux/ioport.h> //for mapping physical and virutal memory to each other 
    #include <linux/highmem.h>//^ditto
   


    /** Define Macros **/
    #define DEVICE_NAME "MotorChar" //Determines name in /dev/[namehere]
    #define CLASS_NAME "Motor Char Dev" //Device Class -- char device driver
    static DEFINE_MUTEX(ebbchar_mutex); //Macro used for multiuser locking

    /** Module Macros **/
    MODULE_LICENSE("GPL");
    MODULE_AUTHOR("LUANNA OCHOA");
    MODULE_DESCRIPTION("Sample Linux Motor Driver");
    MODULE_VERSION("0.1");

    /** Variable Declarations/Initilizations **/
    //Without Mutex we can define the major number with a macro
    //However in this example we're dynamically allocating the majorNumber
    static int majorNumber;
    static int numberOpens = 0;
    static struct class* motorCharClass = NULL;
    static struct device* motorChar Device = NULL;

    /** Function Prototypes **/
    static ssize_t motor_stop(struct *file);
    static ssize_t motor_rotate(struct *file, int direction);
    static ssize_t motor_open(struct inode *inode, struct file *file);
    static ssize_t motor_release(struct inode *inode, struct file *file);
    static ssize_t motor_read(struct file *file , char *buf, size_t count, loff_t *ptr);
    static ssize_t motor_write(struct file *file, const char *buf, size_t count, loff_t * ppos);
    static ssize_t motor_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);

    /** Structure Prototypes **/
    static struct file_operations fops = {
        Owner: THIS_MODULE
        stop: motor_stop
        rotate: motor_rotate
        open: motor_open
        release: motor_release
        read: motor_read
        write: motor_write
        ioctl: motor_ioctl
    };

    /** Init and Exit Functions **/
    static int __init motorChar_init(void){
        printk(KERN_INFO "MotorChar: Initializing the MotorChar LKM \n");

        //Dynamically allocate a major number
        majorNumber = register_chrdev(0, DEVICE_NAME, &fops);
        if (majorNumber<0){
            printk(KERN_ALERT "MotorChar failed to register a major number \n");
            return majorNumber;
        }        

        printk(KERN_INFO "MotorChar: Registered correctly with major number %d \n", majorNumber);

        //Register the device driver 

        //Enable multiuser locking
        mutex_init(&ebbchar_mutex);
        return 0;
    }

    static void __exit motorChar_exit(void){
        //Destroy previously created multiuser lock
        mutex_destroy(&ebbchar_mutex);

        //Unregister and destroy device 
        device_destroy(motorCharClass, MKDEV(majorNumber, 0));
        class_unregister(motorCharClass);
        class_destroy(motorCharClass);
        unregister_chrdev(majorNumber, DEVICE_NAME);
        printk(KERN_INFO "MotorChar: Goodbye from the LKM!\n");
    }

    /** User Defined Functions (from fops struct) **/

    static ssize_t motor_stop(struct *file){
        //code that stops motor
    }

    static ssize_t motor_rotate(struct *file, int direction){
        //code that rotates motor
    }

    static ssize_t motor_open(struct inode *inode, struct file *file){
        //code that "opens" motor
        if(!mutex_trylock(&ebbchar_mutex)){
            printk(KERN_ALERT "MotorChar: Device is in use by another process");
            return -EBUSY;
        }


    }

    static ssize_t motor_release(struct inode *inode, struct file *file){
        //code that releases motor
        mutex_inlock(&ebbchar_mutex);

        printk(KERN_INFO "MotorChar: Device Succesfully Closed \n");
        return 0;
    }

    static ssize_t motor_read(struct file *file , char *buf, size_t count, loff_t *ptr){
        //code that reads from motor
    }

    static ssize_t motor_write(struct file *file, const char *buf, size_t count, loff_t * ppos){
        //code that writes to motor
    }

    static ssize_t motor_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg){
        //code that uses ioctl
    }


    /** Module init/exit macros from init.h **/
    module_init(motorChar_init);
    module_exit(moduleChar_exit);
    

```

#####Note:
+ All kernel modules must have at least an init and exit function
+ Can't use printf, must use printk
+ You can see what modules are already loaded into the kernel by running lsmod
    * This command gets info from /proc/modules
+ insmod and modprob are other important commands
    * insmod requires fullpath name and careful use 
    * modprob takes the module name, without any extension, and figures it all out by parsing /lib/modules/version/modules.dep

###Part2: Steps to prepare Device Driver motor1.ko
-------
1. `CD` into linux/drivers/char 
2. Create a directory named motorModule in the stated directory with the following command:

    mkdir motorModule

3. Place the C code for motor1.ko in this directory 
4. Create a makefile in this directory 
    * Makefile should contain:


    obj $(CONFIG_STEPPERMOTOR) += motor1.o

5. Run the `CD ..` command. Your present working directory should be /linux/drivers/char 
6. Edit the Kconfig file in this directory. The goal is to congigure the module to be loaded. The file should contain:

    config STEPPERMOTOR
    tristate "Enable Steppermotor"
    default M

7. In the same directory, make changes to the makefile. So at /linux/drivers/char/makefile add the following lines:

    obj $(CONFIG_STEPPERMOTOR) += motorModule/

8. Now compile the driver with the following command: `make ARCH=arm CROSS_COMPILE=linux-arm-gnuaebi-modules`. This will produce motor1.ko
9. Now we need to place motor1.ko in proper linux directory, we're going to use the following command for this: `make ARCH=arm CROSS_COMPILE=linux-arm-gnueabi` followed by `INSTALL_MOD_PATH=/home/develop/sandbox/fs modules_install` this places the module binary in the directory /home/develop/sandbox/fs/lib/modules/linux-3.8
10. To load the module once it resides in the modules/linux-3.8 directory and offload with the following commands depending on the desired use method:
    * Manually use: `Modprobe motor1` and `modprobe -r motor1`
    * Automatically as part of init process runlevel then in /etc/init.d/runlvl3.startup add the lines: `#load steppermotor driver modprobe motor1` and in /etc/init.d/sys.shutdown and /etc/init.d/sys.reboot add the lines: `#unload steppermotor driver modprobe -r motor1`
11. Finally make a device node for the device driver so that the application binds its request to a specific driver with the following command `mknod /dev/motor1 c 234 0`


###Part3: C Program testMotor.c
-------
Open the device Motor1, issue a start command to rotate in the left direction, stop the motor and close the device.

```c
    #include <stdio.h>//To open and close the motor 

    //use -d to specify degrees of rotation 
    //"l" is left and "r" is right

    int main (int argc, char *argv[]){
            //open motor
            if ( argc==3 && (!strcmp(argv[1],"-d")) ){
            char direction = argv[2];
           
            switch(direction) {
                case 'l': 
                    //rotate to left
                    break;

                case 'r':
                    //rotate to right
                    break;

                default: 
                    printf("Please restart program and enter l for left or r for right.");
                    break;
            }
            //close motor 

        return 0;
    }
```
