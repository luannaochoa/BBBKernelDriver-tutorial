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
  #include <linux/init.h> //Has macros to mark up f(x), i.e. __init/exit
  #include <linux/module.h> //Main header for loading LKMS to Kernel
  #include <linux/device.h> //Header supports module.h
  #include <linux/kernel.h> //Main lib(macros+f(x)) for kernel 
  #include <linux/fs.h> //Headers for linux file system support
  #include <asm/uaccess.h> //Required for the copy to user function
  #include <linux/mutex.h> //Multiuser capability
  #include <linux/string.h> //Convert string to morse and size message
  #include <linux/errno.h>     // error codes
  #include <linux/delay.h>
  #include <linux/types.h>  
  #include <linux/kdev_t.h> 
  #include <linux/ioport.h>
  #include <linux/highmem.h>
  #include <linux/pfn.h>
  #include <linux/version.h>
  #include <linux/ioctl.h>
  #include <net/sock.h>
  #include <net/tcp.h>

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
  #define USR3 (1<<24) //BBB Led
  #define USR0 (1<<21) ///BBB Led

  #define USR_LED USR0
  #define LED0_PATH "/sys/class/leds/beaglebone:green:usr0"

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

  static volatile void *gpio_addr;
  static volatile unsigned int *gpio_setdataout_addr;
  static volatile unsigned int *gpio_cleardataout_addr;

  static struct file * f = NULL;
  static int reopen = 0;
  static char *filepath = 0;
  static char fullFileName[1024];
  static int dio = 0;

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

  ssize_t write_vaddr_disk(void *, size_t);
  int setup_disk(void);
  void cleanup_disk(void);
  static void disable_dio(void);

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

     if (dio && reopen) { 
        f = filp_open(filepath, O_WRONLY | O_CREAT | O_LARGEFILE | O_SYNC | O_DIRECT, 0444);
     } else if (dio) {
        f = filp_open(filepath, O_WRONLY | O_CREAT | O_LARGEFILE | O_TRUNC | O_SYNC | O_DIRECT, 0444);
     }

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

     set_fs(fs);
     return 0;
  }

  /**  **/
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

     fs = get_fs();
     set_fs(KERNEL_DS);

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
     return s;
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