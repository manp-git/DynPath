![image](https://user-images.githubusercontent.com/82860513/192842799-d96282e3-1838-4d76-8125-651643cd2c08.png)


#include<stdio.h>

int a=5;
int i=0;

int F3 ()
{
printf("\n F3 \n");
for(int x=0; x<3;x++)
{
  a=a+1;
printf("\n L2 \n");}
return 0;
}

int F2()
{
    printf("\n F2 \n");
    for(int x=0; x<5;x++)
    {
        printf("\n L1 \n");
        a=F3(); 
        
    }
    for(int x=0; x<3;x++){printf("\n L3 \n");
        a=a+1; }
  return 0;
  }

int F4 ()
{
a=a+1;
printf("\n F4 \n");

return 0;
}

int F10 (int i)
{
printf("\n Inside recursive F10-1,i= %d\n",i);

if (i<8)
{
i=i+1;
printf("\n Inside recursive F10-2,i= %d\n", i);
a=F10(i);
//printf("\n Inside recursive F10-3,i= %d\n", i);
return 0;
}

return 0;
}

int F5 ()
{
printf("\n F5 \n");
return 0;
}

int F6 ()
{
printf("\n F6 \n");
return 0;
}

int F7 ()
{
printf("\n F7 \n");
return 0;
}

int F8 ()
{
printf("\n F8 \n");
return 0;
}

int F9 ()
{
printf("\n F9 \n");
return 0;
}


int F11 ()
{
printf("\n F11 \n");
return 0;
}

int F12()
{
a=F10(i); 
printf("\n F10 inside F12 \n");
return 0;
}

int F1() 
{
printf("\n F1 \n");
a=F2();
a=F4();

for(int x=0; x<12;x++){printf("\n L4 \n");
        a=a+1;
    
if(x<6){printf("\n x<6 \n");a=F5();}else if(x>=6){printf("\n x=>6 \n");a=F6();} else {a=F7();}
}
for(int x=0; x<20;x++){printf("\n L5 \n");
        a=a+1;a=F8();
a=F9();}
a=F10(i);
a=F11();
for(int x=0; x<16;x++){printf("\n L6 \n");
        a=F11();}
a=F12();
return 0;
}

int F13()
{
printf("\n F13 \n");return 0;}

int main()
{
a=F1();
a=F13();

return 0;

}