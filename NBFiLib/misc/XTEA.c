#include "xtea.h"

typedef  uint32_t*  xtea_key_t;

const unsigned long DELTA = 0x9E3779B9;

#define NUM_ITERATIONS 64

#define XTEA_KEY_LENGTH 8   // In 32-bit words

uint32_t local_key[8] = {0,0,0,0,0,0,0,0};

xtea_key_t XTEA_KEY_PTR = (uint32_t *)local_key;

static _Bool xtea_enabled = 1;

static void Encode(unsigned long * data, unsigned char dataLength, xtea_key_t key)
{
	unsigned char i=0;
	unsigned long x1;
	unsigned long x2;
	unsigned long sum;
	unsigned char iterationCount;

	while(i<dataLength)
	{
		sum = 0;
		x1=*data;
		x2=*(data+1);
		iterationCount = NUM_ITERATIONS;

		while(iterationCount > 0)
		{
			x1 += ((x2<<4 ^ x2>>5) + x2) ^ (sum + *(key+(sum&0x03)));
			sum+=DELTA;
			x2 += ((x1<<4 ^ x1>>5) + x1) ^ (sum + *(key+(sum>>11&0x03)));

			iterationCount--;
		}
		*(data++)=x1;
		*(data++)=x2;
		i+=2;
	}
}


static void Decode(unsigned long * data, unsigned char dataLength, xtea_key_t key)
{
	unsigned char i=0;
	unsigned long x1;
	unsigned long x2;
	unsigned long sum;
	unsigned char iterations;

	iterations = NUM_ITERATIONS;

	while(i<dataLength)
	{
		sum = DELTA*iterations;
		x1=*data;
		x2=*(data+1);

		while(sum != 0)
		{
			x2 -= ((x1<<4 ^ x1>>5) + x1) ^ (sum + *(key+(sum>>11&0x03)));
			sum-=DELTA;
			x1 -= ((x2<<4 ^ x2>>5) + x2) ^ (sum + *(key+(sum&0x03)));
		}
		*(data++)=x1;
		*(data++)=x2;
		i+=2;
	}
}

extern uint32_t* (* __nbfi_read_key)(void);

_Bool XTEA_Available()
{
    // Check if KEY area not empty
    for(int i = 0; i < XTEA_KEY_LENGTH; i++)
    {
        if(((xtea_key_t)XTEA_KEY_PTR)[i] != 0) return 1;
    }
    return 0;
}

_Bool XTEA_Enabled()
{
    return xtea_enabled;
}

void XTEA_Enable(_Bool enable)
{
    if(enable)  xtea_enabled = 1;
    else        xtea_enabled = 0;
}

void XTEA_Encode(uint8_t * buf)
{
    Encode((unsigned long *)buf, 2, XTEA_KEY_PTR);
    Encode((unsigned long *)buf, 2, XTEA_KEY_PTR + 4);
}

void XTEA_Decode(uint8_t * buf)
{
    Decode((unsigned long *)buf, 2, XTEA_KEY_PTR + 4);
    Decode((unsigned long *)buf, 2, XTEA_KEY_PTR);
}

void XTEA_Set_KEY(uint32_t* key)
{
  for(int i = 0; i != XTEA_KEY_LENGTH; i++) local_key[i] = key[i];
  XTEA_KEY_PTR = (uint32_t *)local_key;
}

void XTEA_Set_KEY_Ptr(uint32_t* ptr)
{
  XTEA_KEY_PTR = ptr;
}

