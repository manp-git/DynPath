


#include<stdio.h>

int a=5;
int i=0;

int F3 ()
{
	printf("\n F3 \n");
	for(int x=0; x<3;x++)
	{
	 a=a+1;
	}
return 0;
}

int F2()
{
    printf("\n F2 \n");
    for(int x=0; x<5;x++)
    {        
        a=F3();        
    }
    for(int x=0; x<3;x++)
	{
        a=a+1; 
		}
  return 0;
  }

int F4 ()
{
a=a+1;
return 0;
}

int F10 (int i)
{
if (i<8)
{
i=i+1;
a=F10(i);
return 0;
}
return 0;
}

int F5 ()
{
a=a+1;
return 0;
}

int F6 ()
{
a=a+1;
return 0;
}

int F7 ()
{
a=a+1;
return 0;
}

int F8 ()
{
a=a+1;
return 0;
}

int F9 ()
{
a=a+1;
return 0;
}


int F11 ()
{
a=a+1;
return 0;
}

int F12()
{
a=F10(i); 
return 0;
}

int F1() 
{
a=F2();
a=F4();

for(int x=0; x<12;x++)
{
        a=a+1;
    
if(x<6)
{a=F5();}
else if(x>=6)
{	a=F6();} 
	else {a=F7();}
}

for(int x=0; x<20;x++)
{
a=a+1;a=F8();
a=F9();}
a=F10(i);
a=F11();
for(int x=0; x<16;x++)
{
a=F11();
}
a=F12();
return 0;
}

int F13()
{
a=a+1;
return 0;}

int main()
{
a=F1();
a=F13();

return 0;

}
