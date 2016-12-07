#Kernel Device Driver Assignment 

##By Luanna Ochoa for Course EEL4734
##Student ID: 5817288

## Tutorial Content
1. Overview
    1. Resources Needed
    2. Resources Cited/Referenced
    3. Disclaimers  
2. Code
    1. Device Driver
    2. User Space Program
2. Partition the SD
    1. Pre-reqs
    2. Steps
3. Build the BeagleBoneBlack Kernel
    1.  Pre-reqs 
    2.  Kernel File Changes
    3.  Compiling Kernel with the Device Driver
4. Bring all Parts together 
    1. Loading kernel to the SD Card  
    2. Boot BeagleBoneBlack off the SD Card 
    3. Test the Kernel Device Driver 

##Overview:

####Resources Needed
+ Computer with a VM running Ubuntu 64-img

+ Patience. 

+ BeagleBoneBlack and USB Connector 

+ SD Card

+ Debug Cable (Red, green and blue wires)

####Resources Cited/Referenced 
+ http://wiki.beyondlogic.org/index.php/BeagleBoneBlack_Building_Kernel
+ http://embedjournal.com/kernel-compilation-beaglebone-black/ 
+ http://www.howtogeek.com/howto/17001/how-to-format-a-usb-drive-in-ubuntu-using-gparted/

####Disclaimers 
+ If a command doesn't have 'permission', try `sudo [your-command]`
+ If a command doesn't work at all, try typing it correctly 

##Code:
+ All kernel modules must have at least an init and exit macros

+ You can't use printf on the kernel level, you must use printk

+ You can see what modules are already loaded into the kernel by running `lsmod`
    * This command gets info from /proc/modules
    
    * `insmod` and `modprob` are other important commands
    
    * `insmod` requires fullpath name and careful use 

    * `modprob` takes the module name, without any extension, and figures it all out by parsing /lib/modules/version/modules.dep

####Device Driver Code 
This is the device driver code for the BeagleBoneBlack LED.
```c
    
    /***********************************************************************
    * Beagle Bone Black Morse Code LED Blink Device Driver
    *
    * DESCRIPTION :
    *             Device driver flashes a string to the BBB in morse code.   
    * 
    * AUTHOR :    Luanna Ochoa         
    *
    * COURSE :    Embedded Operating Systems at 
    *             Florida International University
    *              
    * CONTENTS :  
    *             Includes Mutex throughout 
    *             Includes
    *             Macros
    *             Variable Declarations/Initilizations
    *             Prototype Functions 
    *             Struct Prototype
    *             Init Function
    *             Exit Function
    *             All Other Functions
    *              
    *************************************************************************/

    /**Include Section**/
    #include <linux/init.h>     //Has macros to mark up f(x), i.e. __init/exit
    #include <linux/module.h>   //Main header for loading LKMS to Kernel
    #include <linux/device.h>     //Header supports module.h
    #include <linux/kernel.h>     //Main lib(macros+f(x)) for kernel 
    #include <linux/fs.h>       //Headers for linux file system support
    #include <asm/uaccess.h>    //Required for the copy to user function (usr space to kernel)
    #include <linux/mutex.h>    //Multiuser capability
    #include <linux/string.h>   //Convert string to morse and size message
    #include <linux/errno.h>        //Error Codes -- like def. 
    #include <linux/delay.h>    //msleep();
    #include <linux/types.h>      // alias 4 data types; platform based
    #include <linux/kdev_t.h>     //
    #include <linux/ioport.h>   //for mapping physical and virtual
    #include <linux/highmem.h>    //used in conjunction with ioport.h
    #include <linux/pfn.h>      
    #include <linux/ioctl.h>    



    /** Define Macros **/
    #define DEVICE_NAME "testchar" //determines name in /dev/[namehere]
    #define CLASS_NAME "test" //device class -- char device driver
    #define CQ_DEFAULT 0 //For Morse Code 
    #define BUFFERSIZE 256 //To set array buffer size

    #define GPIO1_START_ADDR 0x4804C000 //Physical address of GPIO
    #define GPIO1_END_ADDR   0x4804e000 //Physical address of GPIO
    #define GPIO1_SIZE (GPIO1_END_ADDR - GPIO1_START_ADDR)

    #define GPIO_SETDATAOUT 0x194
    #define GPIO_CLEARDATAOUT 0x190
    #define USR3 (1<<24) //BBB Led; shifting the bit, 24 or 21 locations
    #define USR0 (1<<21) ///BBB Led

    #define USR_LED USR0
    #define LED0_PATH "/sys/class/leds/beaglebone:green:usr0" //access to changed mode of the led so tht its off
    static DEFINE_MUTEX(ebbchar_mutex); //macro used for multiuser locking 


    /** Module Macros **/
    MODULE_LICENSE("GPL");
    MODULE_AUTHOR("Luanna Ochoa");
    MODULE_DESCRIPTION("Simple Linux Char Driver");
    MODULE_VERSION("0.1");



    /** Variable Declarations and Initilizations **/
    static int majorNumber;
    static char message[256] = {0};
    static short size_of_message;
    static int numberOpens = 0;
    static struct class* testcharClass = NULL;
    static struct device* testcharDevice = NULL;
    char morseBuffer[BUFFERSIZE];
    int count;
    int morse_index;

    static volatile void *gpio_addr; //is virtual address, we'll need physical address 1st tho
    static volatile unsigned int *gpio_setdataout_addr; // look @ the macro 
    static volatile unsigned int *gpio_cleardataout_addr; // ^

    static struct file * f = NULL; //relates to setup_disk()
    static int reopen = 0;
    static char *filepath = 0;
    static char fullFileName[1024];
    static int dio = 0; //direct IO, relates to setup_disk()

    static char *morse_code[40] = {"",
    ".-","-...","-.-.","-..",".","..-.","--.","....","..",".---","-.-",
    ".-..","--","-.","---",".--.","--.-",".-.","...","-","..-","...-",
    ".--","-..-","-.--","--..","-----",".----","..---","...--","....-",
    ".....","-....","--...","---..","----.","--..--","-.-.-.","..--.."};



    /** Prototype Functions **/
    static int device_open(struct inode *, struct file *);
    static int device_release(struct inode *, struct file *);
    static ssize_t device_read(struct file *, char*, size_t, loff_t *);
    static ssize_t device_write(struct file *, const char*, size_t, loff_t*);
    static char * mcodestring(char);

    ssize_t write_vaddr_disk(void *, size_t); //arg1: where you want to write, and what to write
    //User defined functions, to change the mode of the led from what its doing to none and back 
    int setup_disk(void);
    void cleanup_disk(void);
    static void disable_dio(void);


    /**User defined, will light LED**/
    void BBBremoveTrigger(void);
    void BBBstartHeartbeat(void);
    void BBBledOn(void);
    void BBBledOff(void);

    //This function converts a single character into morse code 
    static char * mcodestring(char asciicode)
    {
       char *mc;   // this is the mapping from the ASCII code into the mcodearray of strings.

       if (asciicode > 122)  // Past 'z'
          mc = morse_code[CQ_DEFAULT];
       else if (asciicode > 96)  // Upper Case
          mc = morse_code[asciicode - 96];
       else if (asciicode > 90)  // uncoded punctuation
          mc = morse_code[CQ_DEFAULT];
       else if (asciicode > 64)  // Lower Case
          mc = morse_code[asciicode - 64];
       else if (asciicode == 63)  // Question Mark
          mc = morse_code[39];    // 36 + 3
       else if (asciicode > 57)  // uncoded punctuation
          mc = morse_code[CQ_DEFAULT];
       else if (asciicode > 47)  // Numeral
          mc = morse_code[asciicode - 21];  // 27 + (asciicode - 48)
       else if (asciicode == 46)  // Period
          mc = morse_code[38];  // 36 + 2
       else if (asciicode == 44)  // Comma
          mc = morse_code[37];   // 36 + 1
       else
          mc = morse_code[CQ_DEFAULT];
       return mc;

    }

    /** file_operations structure **/
      // -> From /linux/fs.h
      // -> Lists the callback f(x)s we want associated w/ file operations
    static struct file_operations fops = {
      .open = device_open,
      .read = device_read,
      .write = device_write,
      .release = device_release,
    };



    /** Function called when initialized **/
    static int __init testchar_init(void){
      printk(KERN_INFO "MorseModule: Initializing the TestChar LKM \n");

      //Dynamically allocate a major number
      majorNumber= register_chrdev(0, DEVICE_NAME, &fops);
      if (majorNumber<0){
        printk(KERN_ALERT "MorseModule failed to register a major number\n");
        return majorNumber;
      }
      printk(KERN_INFO "MorseModule: registered correctly with major number %d\n", majorNumber);

      //Register device to class
      testcharClass=class_create(THIS_MODULE, CLASS_NAME);
      if (IS_ERR(testcharClass)){
        unregister_chrdev(majorNumber, DEVICE_NAME);
        printk(KERN_ALERT "Failed to register device class\n");
        return PTR_ERR(testcharClass);
      }
      printk(KERN_INFO "MorseModule: device class registered correctly\n");

      //Register the device driver
      testcharDevice= device_create(testcharClass, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
      if (IS_ERR(testcharDevice)){
        class_destroy(testcharClass);
        unregister_chrdev(majorNumber, DEVICE_NAME);
        printk(KERN_ALERT "Faied to create the device\n");
        return PTR_ERR(testcharDevice);
      }
      printk(KERN_INFO "MorseModule: device class registered correctly \n");

      //Map memory for GPIO
      gpio_addr = ioremap(GPIO1_START_ADDR, GPIO1_SIZE);
       if(!gpio_addr) {
         printk (KERN_ERR "HI: ERROR: Failed to remap memory for GPIO Bank 1.\n");
      }
      //Look at Macros, used to offset bit values to change them
      gpio_setdataout_addr   = gpio_addr + GPIO_SETDATAOUT;
        gpio_cleardataout_addr = gpio_addr + GPIO_CLEARDATAOUT;



      //Enable multiuser locking
      mutex_init(&ebbchar_mutex); 
      return 0;
    }



    /** Function called upon exit **/
    static void __exit testchar_exit(void){
      //Enable multiuser lock
      mutex_destroy(&ebbchar_mutex);        

      //Turn off LED Capability 
      BBBledOff();
      BBBstartHeartbeat();

      //Unregister and destroy device 
      device_destroy(testcharClass, MKDEV(majorNumber,0));
      class_unregister(testcharClass);
      class_destroy(testcharClass);
      unregister_chrdev(majorNumber, DEVICE_NAME);
      printk(KERN_INFO "MorseModule: Goodbye from the LKM!\n");
    }

    /**-----------------------------------------------------------**/
    /**------------------ All other functions --------------------**/
    /**-----------------------------------------------------------**/

    /**  **/
    void BBBremoveTrigger(){
       // remove the trigger from the LED
       int err = 0;
      
      strcpy(fullFileName, LED0_PATH);
      strcat(fullFileName, "/");
      strcat(fullFileName, "trigger");
      printk(KERN_INFO "File to Open: %s\n", fullFileName);
      filepath = fullFileName; // set for disk write code
      err = setup_disk();
      err = write_vaddr_disk("none", 4);
      cleanup_disk();
    }

    /**  **/
    void BBBstartHeartbeat(){
       // start heartbeat from the LED
         int err = 0;
      

      strcpy(fullFileName, LED0_PATH);
      strcat(fullFileName, "/");
      strcat(fullFileName, "trigger");
      printk(KERN_INFO "File to Open: %s\n", fullFileName);
      filepath = fullFileName; // set for disk write code
      err = setup_disk();
      err = write_vaddr_disk("heartbeat", 9);
      cleanup_disk();
    }

    /**  **/
    void BBBledOn(){
    *gpio_setdataout_addr = USR_LED;
    }

    /**  **/
    void BBBledOff(){
    *gpio_cleardataout_addr = USR_LED;
    }

    /**  **/
    static void disable_dio() {
       dio = 0;
       reopen = 1;
       cleanup_disk();
       setup_disk();
    }

    /**  **/
    int setup_disk() {
       mm_segment_t fs;
       int err;

       fs = get_fs();
       set_fs(KERNEL_DS);
      
       // if (dio && reopen) {  
       //    f = filp_open(filepath, O_WRONLY | O_CREAT | O_LARGEFILE | O_SYNC | O_DIRECT, 0444);
       // } else if (dio) {
       //    f = filp_open(filepath, O_WRONLY | O_CREAT | O_LARGEFILE | O_TRUNC | O_SYNC | O_DIRECT, 0444);
       // }
      
       if(!dio || (f == ERR_PTR(-EINVAL))) {
          f = filp_open(filepath, O_WRONLY | O_CREAT | O_LARGEFILE | O_TRUNC, 0444);
          dio = 0;
       }
       if (!f || IS_ERR(f)) {
          set_fs(fs);
          err = (f) ? PTR_ERR(f) : -EIO;
          f = NULL;
          return err;
       }

       set_fs(fs); //declares/allows fs to be used
       return 0;
    }

    /** Close the file **/
    void cleanup_disk() {
       mm_segment_t fs;

       fs = get_fs();
       set_fs(KERNEL_DS);
       if(f) filp_close(f, NULL);
       set_fs(fs);
    }

    /**  **/
    ssize_t write_vaddr_disk(void * v, size_t is) {
       mm_segment_t fs;

       ssize_t s;
       loff_t pos;

       //opening fs
       fs = get_fs();
       set_fs(KERNEL_DS);
      
      //getting how much to write, computing position; recursive 
       pos = f->f_pos;
       s = vfs_write(f, v, is, &pos);
       if (s == is) {
          f->f_pos = pos;
       }          
       set_fs(fs);
       if (s != is && dio) {
          disable_dio();
          f->f_pos = pos;
          return write_vaddr_disk(v, is);
       }
       return s; //returns what was written
    }



    /** dev_open function definition**/
    static int device_open(struct inode *inodep, struct file *filep){
        if(!mutex_trylock(&ebbchar_mutex)){    /// Try to acquire the mutex (i.e., put the lock on/down)
                                          /// returns 1 if successful and 0 if there is contention
        printk(KERN_ALERT "MorseModule: Device in use by another process");
        return -EBUSY;
      }

      numberOpens++;
      printk(KERN_INFO "MorseModule: Device has been opened %d time(s)\n", numberOpens);
      return 0;
    }



    /** dev_read function definition **/
    static ssize_t device_read(struct file *filep, char *buffer, size_t len, loff_t *offset){
      int error_count = 0;
      error_count=copy_to_user(buffer, message, size_of_message);

      if(error_count==0){
        printk(KERN_INFO "MorseModule: Sent %d characters to the user \n", size_of_message);
        return (size_of_message=0);
      }
      else {
        printk(KERN_INFO "MorseModule: Failed to send %d characters to the user\n", error_count);
        return -EFAULT;
      }
    }



    /** dev_write function definition **/
    static ssize_t device_write(struct file *filep, const char *buffer, size_t len, loff_t *offset){
      sprintf(message, "%s", buffer);
      size_of_message = strlen(message);
      printk(KERN_INFO "MorseModule: Received %d  characters from the user \n", len);
      printk(KERN_INFO "Converting your string: \"%s\" into morse code... \n", message);
      
      //store string as morse code into morseBuffer
      for (count = 0; count <= size_of_message; count++ ){
        if (count == 0){ 
          strcpy(morseBuffer, mcodestring(message[count]));
              }
        else {
          strcat(morseBuffer, mcodestring(message[count]));
        }
      }
      
       BBBremoveTrigger();

      //Blink LED Depending on morseBuffer index value
      printk(KERN_INFO "%s\n", morseBuffer);

      for (morse_index=0; morse_index <= sizeof(morseBuffer); morse_index++){

        if(morseBuffer[morse_index] == '-'){
          msleep(500);      
          printk(KERN_INFO "LED STATUS: dash\n");
          BBBledOn();
          msleep(700);
          BBBledOff();
        }
        else if (morseBuffer[morse_index] == '.'){
          msleep(500);      
          printk(KERN_INFO "LED STATUS: dot\n");
          BBBledOn();
          msleep(300);
          BBBledOff();
        }
      }

      return len;
    }


    /** dev_release function definition **/
    static int device_release (struct inode *inodep, struct file *filep){
      mutex_unlock(&ebbchar_mutex);     

      printk(KERN_INFO "MorseModule: Device Succesfully Closed\n");
      return 0;
    }


    /** Module init/exit macros from init.h used **/
    module_init(testchar_init);
    module_exit(testchar_exit);
```


####User Space Program Code
This program takes one argument, the string we would like to convert to morse.

```c
    #include <stdio.h>
    #include <stdlib.h>
    #include <errno.h>
    #include <fcntl.h>
    #include <string.h>

    #define BUFFER_LENGTH 256 
    static char receive[BUFFER_LENGTH];

    int  read(  int  handle,  void  *buffer,  int  nbyte );
    int  write(  int  handle,  void  *buffer,  int  nbyte  );

    int main(){
        int ret, fd;
        char stringToSend[BUFFER_LENGTH];
        printf("Starting Device Test Code Example... \n");
        fd = open("/dev/testchar", O_RDWR);
        if (fd<0){
            perror("Failed to open the device...");
            return errno;
        }

        printf("Type in a short string to send to the kernel module:\n");
        scanf("%[^\n]%*c", stringToSend);

        printf("Writing message to the device [%s].\n", stringToSend);
        ret = write(fd, stringToSend, strlen(stringToSend));
        if (ret <0){
            perror("Failed to write the message to the device.");
            return errno;
        }

        printf("Press ENTER to read back from the device \n");
            getchar();

        printf("Reading from the device ...\n");
        ret = read(fd, receive, BUFFER_LENGTH);
        if (ret <0){
            perror("Failed to read the message from the device.");
            return errno;
        }

        printf("The received message is [%s] \n", receive);
        printf("End of program \n");
        return 0;
}
```

##Partition The SD:
####Pre-reqs
1. Have access to your SD through the VM 

    1. Consider using a USB SD Card Reader

    2. Consider using a computer with an SD Reader

2. Download gparted by running the command `sudo apt-get install gparted`

####Steps
1. Insert your SD card

2. Run `lsusb` in order to see that the application was picked. 

3. Fire up gparted by running `sudo gparted` 

4. Find your SD card in the drop-down box at the top right of GParted

5. Right click on the partition and unmount it if it is mounted

6. Then right click on the partition again and delete it 

7. Right click on the partition again and select new

8. On the right select create a new partition

9. Type the label `BOOT` and hit 'add'

10. Complete these steps a second time around, but change the label to `RFS`

12. Confirm that you've completed the afformentioned steps correctly 

##Build The BeagleBoneBlack Kernel:

####Pre-reqs

+ **The ARM Cross Compiler**
    * In order to complete this tutorial you will need an ARM cross compiler.

        `sudo apt-get install gcc-arm-linux-gnueabi`

+ **GIT**
    * In order to complete this tutorial you will also need GIT 

        `sudo apt-get install git`

    * Configure git with your identity.
    
        `git config --global user.email "your.email@here.com`

    * Clone the git repo by issuing the following command:
    `git clone git://github.com/beagleboard/linux.git`

    * `cd` into linux and run `git checkout 4.1`

+ **lzop Compression**
    * Install lzop Compression. You'll want to install this so you can uncompress the kernel. 
        
        `sudo apt-get install lzop`

+ **uBoot mkimage**
    * Install pre-reqs for u-Boot and then download and install u-Boot with the following commands

        `sudo apt-get install libssl-dev`

        `wget ftp://ftp.denx.de/pub/u-boot/u-boot-latest.tar.bz2`

        `tar -xjf u-boot-latest.tar.bz2`

        `cd` into `u-boot` directory

        `make sandbox_defconfig tools-only`

        `sudo install tools/mkimage /usr/local/bin`


+ **RFS(Root File System)**
    * download a premade RFS from the following link: `https://www.dropbox.com/s/k93doprl261hwn2/rootfs.tar.xz?dl=0`
    * Now run the following commands:
       `sudo tar -xvf rootfs.tar.xz -C /media/mani/RFS/`
       `cd /media/luanna/RFS/rootfs/`
       `sudo mv ./* ../`
       `cd ../`
       `sudo rmdir rootfs`


####Kernel File Changes

1. `cd` into linux/drivers/char 

2. Create a directory named morseModule in the stated directory with the following command:

    `mkdir morseModule`

3. Place the device driver C code for morseModule in this directory 

4. Create a makefile in this directory 
    * Makefile should contain:
       `obj $(CONFIG_MORSE_MODULE) += testchar.o`

    * This makefile will now look for any testchar.c or testchar.s files because we've included this line

6. Create a Kconfig file in this directory as well 
    * Kconfig file should contain: 
    
       `config MORSE_MODULE
            tristate "Enable MorseModule"
            default y
            help`

5. Run the `cd ..` command to go back one directory. Your present working directory should be '/linux/drivers/char'

6. Edit the Kconfig file in this directory. The goal is to congigure the module to be loaded. The file should contain:

    `source "drivers/char/morseModule/Kconfig" `


7. In the same directory, make changes to the makefile. So at /linux/drivers/char/makefile add the following lines:

    `obj $(CONFIG_MORSE_MODULE) += morseModule/ `

####Compiling Kernel with the Device Driver

1. Now `cd` into linux, and complete the following steps.

2. Now compile the driver with the following command: `make ARCH=arm CROSS_COMPILE=linux-arm-gnuaebi- -j4`. This will produce testchar.o and other files.

3. Now run `make ARCH=arm CROSS_COMPILE=arm-linux-gnueabi- uImage dtbs LOADADDR=0x80008000 -j4` . This command produces a compressed kernel. We can transfer this to the sd card or vmlinuz, the kernel proper. 


##Bring All Parts Together
####Loading the Kernel To The SD Card

1. Insert your SD card to your computer. Be sure it appears on your Ubuntu-VM

2. Open up a terminal and `cd` until your present working directory is 'linux/arch/arm/boot'

3. Transfer the uImage in the afformentioned directory to your partitioned SD card. Use the command `cp uImage /media/luanna/BOOT`

4. Now `cd` into 'dts'. Your current file path should be 'linux/arch/arm/boot/dts'. Complete the same task for your dtb. Run the command `cp am335x-boneblack.dtb /media/luanna/BOOT`

5. Now install your RFS(Root File System) with the following command `sudo make ARCH=arm CROSS_COMPILE=arm-linux-gnueabi- INSTALL_MOD_PATH=/media/luanna/RFS modules_install`

6. Check that the RFS was installed to the SD card RFS partition. 

7. Transfer the tester.c executable file to the home directory found on the RFS. Using the command `cp` should do the trick.

####Boot BeagleBoneBlack Off The SD Card

1. Connect your debug cable to the beaglebone and to your computer. Make sure your VM recognizes it

2. Run picocom with the following command `picocom -b 115200 /dev/ttyUSB0` and be sure the output of that command reads 'Terminal Ready'

3. Insert your SD into the BeagleBoneBlack

4. Hold the reset button on the BeagleBoneBlack (across from the USB Port)

5. Now power up the board with the usb cable

6. When prompted to login, enter `root` as the user login

7. You're in. 

####Test The Kernel Device Driver

1. Log into the BeagleBoneBlack using login `root`

2. `cd` to where you saved the tester.c executable and run the binary 

3. Pass ./tester executable the string you want converted into morse 

4. Your screen and beaglebone will now display the morse of that string

##Special Thanks
Special thanks to me, myself, and I, for making this assignment more difficult than it needed to be. 

