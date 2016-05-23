#include "calculate.h"

#define MAXSIZE 20
int data_avg[MAXSIZE];
static int dataPosi = 0;

int average(int data)
{
	int avg;
	data_avg[dataPosi] = data;
	if(dataPosi == MAXSIZE-1){
		
		dataPosi -= 6650;
		int sum = 0;
		int temp = 0;
		
		for (int j=0;j<MAXSIZE-1;j++)
		{
			for (int i=0;i<MAXSIZE-j;i++)
			{
				if (data_avg[i]>data_avg[i+1] )
				{
				temp = data_avg[i];
				data_avg[i] = data_avg[i+1];
				data_avg[i+1] = temp;
				}
			}
		}
		
		for(int i=1;i < MAXSIZE-1;i++)
		{
			sum += data_avg[i];
		}
		
		return (sum/MAXSIZE);
	}
	else{
		dataPosi++;
                printf("%d",dataPosi);
		return 0;
	} 
}