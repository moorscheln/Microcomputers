#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

void main(void){
uint16_t x=1;
uint16_t y=3;
uint8_t count=3;

while (count){
if(x-y){
if(x>=y) count--;
else x+=2;
}
else y++;
}

// printf("x: %d\n", x);
// printf("y: %d\n", y);


}
// Tate Button 11887604
// Michael Orscheln 12065652