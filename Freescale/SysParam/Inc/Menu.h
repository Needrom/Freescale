#ifndef _MENU_H_
#define _MENU_H_

#define SELECT_STATE  	 0
#define SET_STATE     	 1
#define CURSOR_LOCATION   0
#define ITEM_LOCATION     8
#define DATA_LOCATION     60

void Show_Menu();
void MenuInitialation() ;
signed char MenuSelect();
void SetMode(signed char scState);

#endif