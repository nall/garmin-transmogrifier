/*
             LUFA Library
     Copyright (C) Dean Camera, 2009.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com
*/

/*
  Copyright 2009  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, and distribute this software
  and its documentation for any purpose and without fee is hereby
  granted, provided that the above copyright notice appear in all
  copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

#include "../HighLevel/USBMode.h"

#if defined(USB_CAN_BE_HOST)

#define  INCLUDE_FROM_PIPE_C
#include "Pipe.h"

uint8_t USB_ControlPipeSize = PIPE_CONTROLPIPE_DEFAULT_SIZE;

bool Pipe_ConfigurePipe(const uint8_t Number, const uint8_t Type, const uint8_t Token, const uint8_t EndpointNumber,
						const uint16_t Size, const uint8_t Banks)
{
	Pipe_SelectPipe(Number);
	Pipe_EnablePipe();

	UPCFG1X = 0;
	
	UPCFG0X = ((Type << EPTYPE0) | Token | (EndpointNumber << PEPNUM0));
	UPCFG1X = ((1 << ALLOC) | Banks | Pipe_BytesToEPSizeMask(Size));

	return Pipe_IsConfigured();
}

void Pipe_ClearPipes(void)
{
	UPINT = 0;

	for (uint8_t PNum = 0; PNum < PIPE_TOTAL_PIPES; PNum++)
	{
		Pipe_ResetPipe(PNum);
		Pipe_SelectPipe(PNum);
		UPIENX = 0;
		UPINTX = 0;
		Pipe_ClearError();
		Pipe_ClearErrorFlags();
		Pipe_DeallocateMemory();
		Pipe_DisablePipe();
	}
}

uint8_t Pipe_WaitUntilReady(void)
{
	#if (USB_STREAM_TIMEOUT_MS < 0xFF)
	uint8_t  TimeoutMSRem = USB_STREAM_TIMEOUT_MS;	
	#else
	uint16_t TimeoutMSRem = USB_STREAM_TIMEOUT_MS;
	#endif

	USB_INT_Clear(USB_INT_HSOFI);

	for (;;)
	{
		if (Pipe_GetPipeToken() == PIPE_TOKEN_IN)
		{
			if (Pipe_IsINReceived())
			  return PIPE_READYWAIT_NoError;
		}
		else
		{
			if (Pipe_IsOUTReady())
			  return PIPE_READYWAIT_NoError;		
		}

		if (Pipe_IsStalled())
		  return PIPE_READYWAIT_PipeStalled;
		else if (!(USB_IsConnected))
		  return PIPE_READYWAIT_DeviceDisconnected;
			  
		if (USB_INT_HasOccurred(USB_INT_HSOFI))
		{
			USB_INT_Clear(USB_INT_HSOFI);

			if (!(TimeoutMSRem--))
			  return PIPE_READYWAIT_Timeout;
		}
	}
}

uint8_t Pipe_Write_Stream_LE(const void* Data, uint16_t Length
#if !defined(NO_STREAM_CALLBACKS)
                                 , StreamCallbackPtr_t Callback
#endif
								 )
{
	uint8_t* DataStream = (uint8_t*)Data;
	uint8_t  ErrorCode;
	
	if ((ErrorCode = Pipe_WaitUntilReady()))
	  return ErrorCode;

	while (Length)
	{
		if (!(Pipe_IsReadWriteAllowed()))
		{
			Pipe_ClearOUT();
				
			#if !defined(NO_STREAM_CALLBACKS)
			if ((Callback != NULL) && (Callback() == STREAMCALLBACK_Abort))
			  return PIPE_RWSTREAM_CallbackAborted;
			#endif

			if ((ErrorCode = Pipe_WaitUntilReady()))
			  return ErrorCode;
              
		}
		else
		{
			Pipe_Write_Byte(*(DataStream++));
			Length--;
		}		
	}

	return PIPE_RWSTREAM_NoError;
}

uint8_t Pipe_Write_Stream_BE(const void* Data, uint16_t Length
#if !defined(NO_STREAM_CALLBACKS)
                                 , StreamCallbackPtr_t Callback
#endif
								 )
{
	uint8_t* DataStream = (uint8_t*)(Data + Length - 1);
	uint8_t  ErrorCode;
	
	if ((ErrorCode = Pipe_WaitUntilReady()))
	  return ErrorCode;

	while (Length)
	{
		if (!(Pipe_IsReadWriteAllowed()))
		{
			Pipe_ClearOUT();
				
			#if !defined(NO_STREAM_CALLBACKS)
			if ((Callback != NULL) && (Callback() == STREAMCALLBACK_Abort))
			  return PIPE_RWSTREAM_CallbackAborted;
			#endif

			if ((ErrorCode = Pipe_WaitUntilReady()))
			  return ErrorCode;
		}
		else
		{
			Pipe_Write_Byte(*(DataStream--));
			Length--;
		}
	}

	return PIPE_RWSTREAM_NoError;
}

uint8_t Pipe_Discard_Stream(uint16_t Length
#if !defined(NO_STREAM_CALLBACKS)
                                 , StreamCallbackPtr_t Callback
#endif
								 )
{
	uint8_t  ErrorCode;
	
	if ((ErrorCode = Pipe_WaitUntilReady()))
	  return ErrorCode;

	while (Length)
	{
		if (!(Pipe_IsReadWriteAllowed()))
		{
			Pipe_ClearIN();
				
			#if !defined(NO_STREAM_CALLBACKS)
			if ((Callback != NULL) && (Callback() == STREAMCALLBACK_Abort))
			  return PIPE_RWSTREAM_CallbackAborted;
			#endif

			if ((ErrorCode = Pipe_WaitUntilReady()))
			  return ErrorCode;
		}
		else
		{
			Pipe_Discard_Byte();
			Length--;
		}
	}

	return PIPE_RWSTREAM_NoError;
}

#if 0
uint8_t Pipe_Read_Stream_LE(void* Buffer, uint16_t Length
#if !defined(NO_STREAM_CALLBACKS)
                                 , StreamCallbackPtr_t Callback
#endif
								 )
{
	uint8_t* DataStream = (uint8_t*)Buffer;
	uint8_t  ErrorCode;
	
	if ((ErrorCode = Pipe_WaitUntilReady()))
	  return ErrorCode;

	while (Length)
	{
		if (!(Pipe_IsReadWriteAllowed()))
		{
		    printf("CLEAR %d\n", Length);
            
			Pipe_ClearIN();
				
			#if !defined(NO_STREAM_CALLBACKS)
			if ((Callback != NULL) && (Callback() == STREAMCALLBACK_Abort))
			  return PIPE_RWSTREAM_CallbackAborted;
			#endif

			Pipe_Unfreeze();

			if ((ErrorCode = Pipe_WaitUntilReady()))
			  return ErrorCode;
		
			Pipe_Freeze();
		}

		while (Pipe_Bytes_Available() && Length)
		{
            uint8_t b = Pipe_Read_Byte();
		    printf("PIPE: %d %d (%c / 0x%x)\n", Length, Pipe_Bytes_Available(), b, b);
  		  *(DataStream++) = b;
          --Length;
		    
		}
        printf("L: %d\n", Length);
	}

	return PIPE_RWSTREAM_NoError;
}
#endif 
#if 1
uint8_t Pipe_Read_Stream_LE(void* Buffer, uint16_t Length
#if !defined(NO_STREAM_CALLBACKS)
                                 , StreamCallbackPtr_t Callback
#endif
								 )
{
	uint8_t* DataStream = (uint8_t*)Buffer;
	uint8_t  ErrorCode;
	
	Pipe_Unfreeze();
	
	if ((ErrorCode = Pipe_WaitUntilReady()))
	  return ErrorCode;

  Pipe_Freeze();

	while (Length)
	{
        printf("BB: %d ", UPSTAX & 3);
		if (!(Pipe_IsReadWriteAllowed()))
		{
            printf("CLEAR %d\n", Length);
			Pipe_ClearIN();
            printf(">>BB: %d", UPSTAX & 3);
				
			#if !defined(NO_STREAM_CALLBACKS)
			if ((Callback != NULL) && (Callback() == STREAMCALLBACK_Abort))
			  return PIPE_RWSTREAM_CallbackAborted;
			#endif

      	Pipe_Unfreeze();

			if ((ErrorCode = Pipe_WaitUntilReady()))
			  return ErrorCode;
          Pipe_Freeze();
		}
		else
		{
            uint8_t b = Pipe_Read_Byte();
            uint8_t clear = (Pipe_BytesInPipe() & (Pipe_SizeOfPipe() - 1)) == 0;
		    printf("PIPE: %d %d (%c)\n", Pipe_BytesInPipe() & (Pipe_SizeOfPipe() - 1), clear, b);
            *(DataStream++) = b;
			Length--;
			
			if(clear && Pipe_IsReadWriteAllowed())
			{
                printf("myclear\n");
                Pipe_ClearIN();
			}
		}
	}

	return PIPE_RWSTREAM_NoError;
}
#endif

uint8_t Pipe_Read_Stream_BE(void* Buffer, uint16_t Length
#if !defined(NO_STREAM_CALLBACKS)
                                 , StreamCallbackPtr_t Callback
#endif
								 )
{
	uint8_t* DataStream = (uint8_t*)(Buffer + Length - 1);
	uint8_t  ErrorCode;
	
	if ((ErrorCode = Pipe_WaitUntilReady()))
	  return ErrorCode;

	while (Length)
	{
		if (!(Pipe_IsReadWriteAllowed()))
		{
			Pipe_ClearIN();
				
			#if !defined(NO_STREAM_CALLBACKS)
			if ((Callback != NULL) && (Callback() == STREAMCALLBACK_Abort))
			  return PIPE_RWSTREAM_CallbackAborted;
			#endif

			if ((ErrorCode = Pipe_WaitUntilReady()))
			  return ErrorCode;
		}
		else
		{
			*(DataStream--) = Pipe_Read_Byte();
			Length--;
		}
	}
	
	return PIPE_RWSTREAM_NoError;
}

#endif
