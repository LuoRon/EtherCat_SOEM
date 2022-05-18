#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/ioctl.h> //xsmd
#include <unistd.h>
#include <termios.h>
#include <unistd.h>
#include "kbd.h"

static struct termios ori_attr, cur_attr;
int tty_set_flag;

static __inline int tty_reset(void)
{
   if (tcsetattr(STDIN_FILENO, TCSANOW, &ori_attr) != 0)   return -1;
   
   return 0;
}


static __inline int tty_set(void)
{
   if ( tcgetattr(STDIN_FILENO, &ori_attr) )   return -1;

   memcpy(&cur_attr, &ori_attr, sizeof(cur_attr) );
   cur_attr.c_lflag &= ~ICANON;
   // cur_attr.c_lflag |= ECHO;
   cur_attr.c_lflag &= ~ECHO;
   //cur_attr.c_cc = 1;
   //cur_attr.c_cc = 0;

   if (tcsetattr(STDIN_FILENO, TCSANOW, &cur_attr)!= 0)   return -1;

   return 0;
}
/*
static __inline int kbd_hit(void)
{
   fd_set rfds;
   struct timeval tv;
   int retval;
 
   FD_ZERO(&rfds);
   FD_SET(0, &rfds);

   tv.tv_sec= 0;
   tv.tv_usec = 0;

   retval = select(1, &rfds, NULL, NULL, &tv);

   if (retval == -1) 
   {
//      perror("select()");
   
      return 0;
   }
   else if (retval)   return 1;
   else               return 0;

   return 0;
}
*/
void kbd_init(void)
{
   tty_set_flag = tty_set();
}

void kbd_exit(void)
{
   if(tty_set_flag == 0)   tty_reset();
}


int kbd_ready()  
{
   static const int STDIN = 0;
   static int nInit = 0;
   int nKeyReady = 0;
   struct termios theTerm;
   if (!nInit)
   {
      // Use termios to turn off line buffering 
      tcgetattr(STDIN, &theTerm);
      theTerm.c_lflag &= ~ICANON;
      tcsetattr(STDIN, TCSANOW, &theTerm);
      setbuf(stdin, NULL);
      nInit = 1;
   }
   
   ioctl(STDIN, FIONREAD, &nKeyReady);
   
   return nKeyReady;
} 

int kbd_read()
{
   int key = 0;
   while(kbd_ready())   key = (key<<8)+getchar();
   
   return key;
}
/*
int main()
{
   kbd_init();
   while(1) 
   {
      if( kbd_hit() ) 
      {
         const int key = getchar();
         printf("%c pressed\n", key);
         if(key == 'q')   break;
      } 
      else 
      {
         //fprintf(stderr, "<no key detected>\n");
      }
   }

   kbd_exit();
   
   return 0;
}
*/