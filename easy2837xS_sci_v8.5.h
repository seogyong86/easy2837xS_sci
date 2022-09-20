/***************************************************************
    easy2837xS_sci.h
	copyright (c) 2014 by Dae-Woong Chung
	All Rights Reserved.
****************************************************************/
#ifndef _EASY2837XS_SCI_H__
#define _EASY2837XS_SCI_H__

extern void	easyDSP_SCI_Init(void);
extern interrupt void easy_RXINT_ISR(void);
extern interrupt void easy_TXINT_ISR(void);

#endif	// of _EASY2837XS_SCI_H__
