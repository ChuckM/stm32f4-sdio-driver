/*
 * Copyright (c) 2013 - Chuck McManis, All Rights Reserved
 *
 * Some 'curses' like functions that come in handy.
 */

#ifndef __TERM_H
#define __TERM_H

enum TEXT_COLOR { DEFAULT, RED, YELLOW, WHITE, GREEN, BLUE };

/* Prototypes */
void move_cursor(int channel, int row, int col);
void clear_screen(int channel);
void clear_eol(int channel);
void text_color(int channel, enum TEXT_COLOR color);

#endif /* __TERM_H */
