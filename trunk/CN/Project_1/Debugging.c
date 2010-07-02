#include "Debugging.h"

char *strrev(char *str)
{
      char *p1, *p2;

      if (! str || ! *str)
            return str;
      for (p1 = str, p2 = str + strlen(str) - 1; p2 > p1; ++p1, --p2)
      {
            *p1 ^= *p2;
            *p2 ^= *p1;
            *p1 ^= *p2;
      }
      return str;
}

void itochar(int x, char *szBuffer, int radix)
{
int i = 0 , n,xx;
n = x;
while (n > 0)
{
xx = n%radix;
n = n/radix;
szBuffer[i++] = '0' + xx;
}
szBuffer[i] = '\0';
strrev(szBuffer);
}
