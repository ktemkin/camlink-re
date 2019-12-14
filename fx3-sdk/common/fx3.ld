/*
   Cypress USB 3.0 Platform linker script file (fx3.ld)
 
   Copyright Cypress Semiconductor Corporation, 2010-2013,
   All Rights Reserved
   UNPUBLISHED, LICENSED SOFTWARE.

   CONFIDENTIAL AND PROPRIETARY INFORMATION
   WHICH IS THE PROPERTY OF CYPRESS.

   Use of this file is governed
   by the license agreement included in the file
 
      <install>/license/license.txt

   where <install> is the Cypress software
   installation root directory path.
*/

/*
   This is the GNU linker file for FX3 memory map.
   The FX3 device has 16 KB of I-TCM memory which can be used for
   code (typically ISRs) and 512 KB of SYSTEM RAM which is shared
   between code, data and DMA buffers.

   The default memory map used for FX3 applications is as follows:

   Descriptor area Base: 0x40000000 Size: 12KB
   Code area       Base: 0x40003000 Size: 180KB
   Data area       Base: 0x40030000 Size: 32KB
   Driver heap     Base: 0x40038000 Size: 32KB  (Update cyfxtx.c to change this.)
   Buffer area     Base: 0x40040000 Size: 256KB (Update cyfxtx.c to change this.)

   Interrupt handlers to be placed in I-TCM (16KB).
   The first 256 bytes of ITCM are reserved for Exception Vectors.
   These vectors will be loaded to ITCM at address 0x0 as part of
   device initialization.

   Kernel stacks are to be placed in the D-TCM (8KB).
   This is done internal to the library as part of CyU3PFirmwareEntry()
   and is not expected to be modified by the FX3 application.
   SYS_STACK       Base: 0x10000000 Size 2KB
   ABT_STACK       Base: 0x10000800 Size 256B
   UND_STACK       Base: 0x10000900 Size 256B
   FIQ_STACK       Base: 0x10000A00 Size 512B
   IRQ_STACK       Base: 0x10000C00 Size 1KB
   SVC_STACK       Base: 0x10001000 Size 4KB
*/

ENTRY(CyU3PFirmwareEntry);

MEMORY
{
	I-TCM		: ORIGIN = 0x100,	LENGTH = 0x3F00
	SYS_MEM		: ORIGIN = 0x40003000	LENGTH = 0x2D000
	DATA		: ORIGIN = 0x40030000	LENGTH = 0x8000
}

SECTIONS
{
	.vectors :
	{
		*(CYU3P_ITCM_SECTION);
                tx_thread_irq_nesting*(.text)
                tx_thread_context*(.text)
                tx_thread_vectored*(.text)
	} >I-TCM

	.text :
	{
		*(.text)
		*(.rodata*)
		*(.constdata)
		*(.emb_text)
		*(CYU3P_EXCEPTION_VECTORS);
		 _etext = .;
	} > SYS_MEM

	.data :
	{
		_data = .;
		*(.data*)
		* (+RW, +ZI)
		_edata = .;
	} > DATA

	.bss :
	{
		_bss_start = .;
		*(.bss)
	} >DATA 
	. = ALIGN(4);
	_bss_end = . ;

	.ARM.extab :
	{
		*(.ARM.extab* .gnu.linkonce.armextab.*)
	} > DATA

	__exidx_start = .;
	.ARM.exidx :
	{
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
	} > DATA
	__exidx_end = .;
}

