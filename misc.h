#ifndef MISC_H
#define MISC_H
void LCD(int x,int y, int v)
{
    char c[10];
    sprintf(c,"%d ",v);
    lcd_gotoxy(x,y);
    lcd_puts(c);
}
#endif