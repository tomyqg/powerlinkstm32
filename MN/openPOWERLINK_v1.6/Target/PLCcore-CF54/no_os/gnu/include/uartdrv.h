/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  definitons for the psc uart

  -------------------------------------------------------------------------

                $RCSfile: uartdrv.h,v $

                $Author: D.Krueger $

                $Revision: 1.3 $  $Date: 2008/04/17 21:36:33 $

                $State: Exp $

                Build Environment:
                DevC++
                GNU C 3.3.2

  -------------------------------------------------------------------------

  Revision History:

  2005/08/04 -ct:   start of the implementation

****************************************************************************/

#ifndef _UARTDRV_H_
#define _UARTDRV_H_


//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
// types
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
// function prototypes
//---------------------------------------------------------------------------

void UartInit        (int Port_p, uint32 dwBaudrate_p);
char UartInChar      (int Port_p);
void UartOutChar     (int Port_p, char cData_p);
int  UartCharPresent (int Port_p);

#endif  // #ifndef _UARTDRV_H_

