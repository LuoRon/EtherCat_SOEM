#ifndef __KEYBOARD_H__
#define __KEYBOARD_H__

#ifdef WIN32

#define KEY_ESC        0x1b
#define KEY_SPACE      0x20
#define KEY_BACKSPACE  0x8//0x7f
#define KEY_ENTER      0xd///0x0a
#define KEY_ALT_Q      0x71//0x1b71   

#else
#define KEY_ESC        0x1b
#define KEY_SPACE      0x20
#define KEY_BACKSPACE  0x7f
#define KEY_ENTER      0x0a
#define KEY_ALT_Q      0x1b71
#endif

#ifdef __cplusplus
extern "C" {
#endif
	void kbd_init(void);
	void kbd_exit(void);
	int  kbd_read();
	
#ifdef __cplusplus
}
#endif
#endif
