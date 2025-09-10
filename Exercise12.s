            TTL CMPE 250 Exercise Twelve
;****************************************************************
;This program runs a simple game where you have to
;input the LED's shown on the board. It uses a random
;number generator based on the current time.
;Name:  	Jonathan Hubbard, Jay Mantini
;Date:  	11-30-17
;Class:  	CMPE-250
;Section:  	L4, Thursday 11:00am-1:00pm
;---------------------------------------------------------------
;Keil Template for KL46
;R. W. Melton
;September 25, 2017
;****************************************************************
;Assembler directives
            THUMB
            OPT    64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL46Z4.s		;Included by start.s
            OPT  1   			;Turn on listing
;****************************************************************
;EQUates
QBuffer		EQU		4
RxQBuffer	EQU		80
TxQBuffer	EQU		80

IN_PTR		EQU		0
OUT_PTR		EQU		4
BUF_STRT	EQU		8
BUF_PAST	EQU		12
BUF_SIZE	EQU		16
NUM_ENQD	EQU		17
	
MAX_STRING	EQU		79
;---------------------------------------------------------------
;LED Stuff
;Port D
PTD5_MUX_GPIO  EQU  (1 << PORT_PCR_MUX_SHIFT)
SET_PTD5_GPIO  EQU  (PORT_PCR_ISF_MASK :OR: PTD5_MUX_GPIO)
;Port E
PTE29_MUX_GPIO  EQU  (1 << PORT_PCR_MUX_SHIFT)
SET_PTE29_GPIO  EQU  (PORT_PCR_ISF_MASK :OR: PTE29_MUX_GPIO)
	
POS_RED         EQU  29
POS_GREEN       EQU  5
LED_RED_MASK    EQU  (1 << POS_RED)
LED_GREEN_MASK  EQU  (1 << POS_GREEN)
LED_PORTD_MASK  EQU  LED_GREEN_MASK
LED_PORTE_MASK  EQU  LED_RED_MASK
;---------------------------------------------------------------
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ICER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;22:PIT IRQ pending status
;12:UART0 IRQ pending status
NVIC_ICPR_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
;--PIT
PIT_IRQ_PRIORITY    EQU  0
NVIC_IPR_PIT_MASK   EQU  (3 << PIT_PRI_POS)
NVIC_IPR_PIT_PRI_0  EQU  (PIT_IRQ_PRIORITY << UART0_PRI_POS)
;--UART0
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU  (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU  (UART0_IRQ_PRIORITY << UART0_PRI_POS)
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ISER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;PIT_LDVALn:  PIT load value register n
;31-00:TSV=timer start value (period in clock cycles - 1)
;Clock ticks for 0.01 s at 24 MHz count rate
;0.01 s * 24,000,000 Hz = 240,000
;TSV = 240,000 - 1
PIT_LDVAL_10ms  EQU  239999
;---------------------------------------------------------------
;PIT_MCR:  PIT module control register
;1-->    0:FRZ=freeze (continue'/stop in debug mode)
;0-->    1:MDIS=module disable (PIT section)
;               RTI timer not affected
;               must be enabled before any other PIT setup
PIT_MCR_EN_FRZ  EQU  PIT_MCR_FRZ_MASK
;---------------------------------------------------------------
;PIT_TCTRLn:  PIT timer control register n
;0-->   2:CHN=chain mode (enable)
;1-->   1:TIE=timer interrupt enable
;1-->   0:TEN=timer enable
PIT_TCTRL_CH_IE  EQU  (PIT_TCTRL_TEN_MASK :OR: PIT_TCTRL_TIE_MASK)
;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port A
PORT_PCR_SET_PTA1_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTA2_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
;---------------------------------------------------------------
;SIM_SCGC4
;1->10:UART0 clock gate control (enabled)
;Use provided SIM_SCGC4_UART0_MASK
;---------------------------------------------------------------
;SIM_SCGC5
;1->09:Port A clock gate control (enabled)
;Use provided SIM_SCGC5_PORTA_MASK
;---------------------------------------------------------------
;SIM_SOPT2
;01=27-26:UART0SRC=UART0 clock source select
;         (PLLFLLSEL determines MCGFLLCLK' or MCGPLLCLK/2)
; 1=   16:PLLFLLSEL=PLL/FLL clock select (MCGPLLCLK/2)
SIM_SOPT2_UART0SRC_MCGPLLCLK  EQU  \
                                 (1 << SIM_SOPT2_UART0SRC_SHIFT)
SIM_SOPT2_UART0_MCGPLLCLK_DIV2 EQU \
    (SIM_SOPT2_UART0SRC_MCGPLLCLK :OR: SIM_SOPT2_PLLFLLSEL_MASK)
;---------------------------------------------------------------
;SIM_SOPT5
; 0->   16:UART0 open drain enable (disabled)
; 0->   02:UART0 receive data select (UART0_RX)
;00->01-00:UART0 transmit data select source (UART0_TX)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR  EQU  \
                               (SIM_SOPT5_UART0ODE_MASK :OR: \
                                SIM_SOPT5_UART0RXSRC_MASK :OR: \
                                SIM_SOPT5_UART0TXSRC_MASK)
;---------------------------------------------------------------
;UART0_BDH
;    0->  7:LIN break detect IE (disabled)
;    0->  6:RxD input active edge IE (disabled)
;    0->  5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (UART0CLK / [9600 * (OSR + 1)]) 
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDH_9600  EQU  0x01
;---------------------------------------------------------------
;UART0_BDL
;26->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDL_9600  EQU  0x38
;---------------------------------------------------------------
;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select 
;        (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=parity enable (disabled)
;0-->0:PT=parity type (even parity--no effect PE=0)
UART0_C1_8N1  EQU  0x00
;---------------------------------------------------------------
;UART0_C2
;0-->7:TIE=transmit IE for TDRE (disabled)
;0-->6:TCIE=transmission complete IE for TC (disabled)
;0-->5:RIE=receiver IE for RDRF (disabled)
;0-->4:ILIE=idle line IE for IDLE (disabled)
;1-->3:TE=transmitter enable (enabled)
;1-->2:RE=receiver enable (enabled)
;0-->1:RWU=receiver wakeup control (normal)
;0-->0:SBK=send break (disabled, normal)
UART0_C2_T_R    EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
UART0_C2_T_RI   EQU  (UART0_C2_RIE_MASK :OR: UART0_C2_T_R)
UART0_C2_TI_RI  EQU  (UART0_C2_TIE_MASK :OR: UART0_C2_T_RI)
;---------------------------------------------------------------
;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
;           10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
;           10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
;            (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:ORIE=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=parity error IE for PF (disabled)
UART0_C3_NO_TXINV  EQU  0x00
;---------------------------------------------------------------
;UART0_C4
;    0-->  7:MAEN1=match address mode enable 1 (disabled)
;    0-->  6:MAEN2=match address mode enable 2 (disabled)
;    0-->  5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
;               = 1 + OSR for 3 <= OSR <= 31
;               = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16           EQU  0x0F
UART0_C4_NO_MATCH_OSR_16  EQU  UART0_C4_OSR_16
;---------------------------------------------------------------
;UART0_C5
;  0-->  7:TDMAE=transmitter DMA enable (disabled)
;  0-->  6:Reserved; read-only; always 0
;  0-->  5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
;  0-->  1:BOTHEDGE=both edge sampling (rising edge only)
;  0-->  0:RESYNCDIS=resynchronization disable (enabled)
UART0_C5_NO_DMA_SSR_SYNC  EQU  0x00
;---------------------------------------------------------------
;UART0_S1
;0-->7:TDRE=transmit data register empty flag; read-only
;0-->6:TC=transmission complete flag; read-only
;0-->5:RDRF=receive data register full flag; read-only
;1-->4:IDLE=idle line flag; write 1 to clear (clear)
;1-->3:OR=receiver overrun flag; write 1 to clear (clear)
;1-->2:NF=noise flag; write 1 to clear (clear)
;1-->1:FE=framing error flag; write 1 to clear (clear)
;1-->0:PF=parity error flag; write 1 to clear (clear)
UART0_S1_CLEAR_FLAGS  EQU  0x1F
;---------------------------------------------------------------
;UART0_S2
;1-->7:LBKDIF=LIN break detect interrupt flag (clear)
;             write 1 to clear
;1-->6:RXEDGIF=RxD pin active edge interrupt flag (clear)
;              write 1 to clear
;0-->5:(reserved); read-only; always 0
;0-->4:RXINV=receive data inversion (disabled)
;0-->3:RWUID=receive wake-up idle detect
;0-->2:BRK13=break character generation length (10)
;0-->1:LBKDE=LIN break detect enable (disabled)
;0-->0:RAF=receiver active flag; read-only
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS  EQU  0xC0
;---------------------------------------------------------------
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
            EXPORT  Reset_Handler
            IMPORT  Startup
Reset_Handler  PROC  {},{}
main
;---------------------------------------------------------------
;Mask interrupts
            CPSID   I
;KL46 system startup with 48-MHz system clock
            BL      Startup
;---------------------------------------------------------------
;>>>>> begin main program code <<<<<
			BL		Init_UART0_IRQ
			BL		Init_PIT_IRQ
			BL		Init_LED
			BL		RedDisable
			BL		GreenDisable
			
			MOVS	R2,#0				;Initialize Round counter
			MOVS	R1,#MAX_STRING	
			LDR		R5,=Score
			STR		R2,[R5,#0]			;Initialize Score to 0
			
			MOVS	R0,#0
			LDR		R4,=Count
            STR		R0,[R4,#0]			;Set Count to 0
			MOVS	R0,#1
			LDR		R4,=RunStopWatch	
			STRB	R0,[R4,#0]			;Enable RunStopWatch
			
			LDR		R0,=Welcome			;Print the entire beginning message set
			BL		PutStringSB
			BL		NewLine
			LDR		R0,=Instruct1
			BL		PutStringSB
			BL		NewLine
			LDR		R0,=Instruct2
			BL		PutStringSB
			BL		NewLine
			LDR		R0,=Instruct3
			BL		PutStringSB
			BL		NewLine
			LDR		R0,=Instruct4
			BL		PutStringSB
			BL		NewLine
			LDR		R0,=Instruct5
			BL		PutStringSB
			BL		NewLine
			LDR		R0,=Instruct6
			BL		PutStringSB
			BL		NewLine
			
			BL		GetChar				;Wait for user to start game
		
;---------------------------------------------------------------
;Main Code Register Usages
;R0 = Temporary value
;R1 = MAX_STRING, used for PutStringSB
;R2 = Round counter, incremented each round
;R3 = Random number, used for lighting LED(s)
;R4 = Address for message corresponding to R3 (red, both, etc.)
;R5 = Temporary value
;---------------------------------------------------------------
			
MainLoop	
			ADDS	R2,R2,#1			;Increment Round counter
			BL		RandomGen			;R3 = Number from 0-3
			
;-------RANDOM NUMBER PROCESSING---------------------------------

			CMP		R3,#3
			BNE		Check1				;3 = Both
			BL		RedEnable
			BL		GreenEnable
			LDR		R4,=BothMsg			;Load message for later use
			B		StartRound
			
Check1		CMP		R3,#2
			BNE		Check2				;2 = Green
			BL		GreenEnable
			LDR		R4,=GreenMsg		;Load message for later use
			B		StartRound
			
Check2		CMP		R3,#1
			BNE		Check3				;1 = Red
			BL		RedEnable
			LDR		R4,=RedMsg			;Load message for later use
			B		StartRound
			
Check3									;0 = None
			LDR		R4,=NoneMsg			;Load message for later use
			B		StartRound

;-------ROUND GUESS PROCESSING-----------------------------------

StartRound	LDR		R0,=RightArrow
			BL		PutStringSB

			MOVS	R0,#0
			LDR		R5,=Count
            STR		R0,[R5,#0]			;Set Count to 0
			MOVS	R0,#1
			LDR		R5,=RunStopWatch	
			STRB	R0,[R5,#0]			;Enable RunStopWatch
			
			BL		GetInput			
			CMP		R0,#0
			BEQ		OutOfTime			;If R0 = 0, then there was no input
			
			BL		PutChar				;Display character
	
			CMP		R0,#91				;Compares R0 to the Ascii value right after the uppercase letters
			BLT		CheckB				;Branches if R0 is uppercase
			SUBS	R0,R0,#32			;Converts lowercase to uppercase
				
CheckB		CMP		R0,#'B'				;Checks if R0 is B
			BNE		CheckG				;If it isn't, branch to next check
			CMP		R3,#3
			BNE		WrongInput			;If both LED's aren't on, input is wrong
			
			LDR		R0,=CorrectMsg
			BL		PutStringSB
			MOVS	R0,R4				;Load message value from earlier
			BL		PutStringSB
			BL		ComputeScore
			B		EndRound
				
CheckG		CMP		R0,#'G'				;Checks if R0 is G
			BNE		CheckR				;If it isn't, branch to next check
			CMP		R3,#2
			BNE		WrongInput			;If the Green LED isn't lit, input is wrong
			
			LDR		R0,=CorrectMsg
			BL		PutStringSB
			MOVS	R0,R4				;Load message value from earlier
			BL		PutStringSB
			BL		ComputeScore
			B		EndRound
				
CheckR		CMP		R0,#'R'				;Checks if R0 is R
			BNE		CheckN				;If it isn't, branch to next check
			CMP		R3,#1
			BNE		WrongInput			;If the Red LED isn't lit, input is wrong
			
			LDR		R0,=CorrectMsg
			BL		PutStringSB
			MOVS	R0,R4				;Load message value from earlier
			BL		PutStringSB
			BL		ComputeScore
			B		EndRound
				
CheckN		CMP		R0,#'N'				;Checks if R0 is N
			BNE		WrongInput			;If it isn't, input is wrong
			CMP		R3,#0
			BNE		WrongInput			;If either LED is on, input is wrong
			
			LDR		R0,=CorrectMsg
			BL		PutStringSB
			MOVS	R0,R4				;Load message value from earlier
			BL		PutStringSB
			BL		ComputeScore
			B		EndRound
			
WrongInput	LDR		R0,=WrongMsg
			BL		PutStringSB
			B		EndRound
			
OutOfTime	LDR		R0,=OutOfTimeMsg
			BL		PutStringSB
			MOVS	R0,R4				;Load message value from earlier
			BL		PutStringSB

EndRound	BL		NewLine
			BL		RedDisable
			BL		GreenDisable
			CMP		R2,#10
			BNE		MainLoop			;Loop until Round 10
			
			LDR		R0,=ThanksMsg		;Display thanks and score messages
			BL		PutStringSB
			BL		NewLine
			LDR		R0,=ScoreMsg
			BL		PutStringSB
			
			LDR		R5,=Score
			LDR		R0,[R5,#0]			;Load score into R0 for PutNumU
			BL		PutNumU
			BL		NewLine
			
			LDR		R0,=ReplayMsg		;Display replay prompt
			BL		PutStringSB	
			BL		NewLine
			
ReplayLoop
			BL		GetChar
	
			CMP		R0,#91				;Compares R0 to the Ascii value right after the uppercase letters
			BLT		CheckYes			;Branches if R0 is uppercase
			SUBS	R0,R0,#32			;Converts lowercase to uppercase
				
CheckYes	CMP		R0,#'Y'				;Checks if R0 is Y
			BNE		CheckNo				;If it isn't, branch to next check
			
			BL		NewLine
			MOVS	R2,#0				;Reinitialize round counter
			STR		R2,[R5,#0]			;Reset score to 0
			B		MainLoop
			
CheckNo		CMP		R0,#'N'				;Checks if R0 is N
			BNE		ReplayLoop			;Loop until they enter Y or N
			
			LDR		R0,=GoodbyeMsg
			BL		PutStringSB
			
;>>>>>   end main program code <<<<<
;Stay here
            B       .
			LTORG
            ENDP
				
;>>>>> begin subroutine code <<<<<
Init_UART0_IRQ	PROC	{R0-R13}
	;Purpose: 	Initializes the KL46 for interrupts
	;Calls:		InitQueue
	;Inputs:	
	;Outputs:	
	;Modified:	R14-R15,APSR
	
			;Pushes R0-R3 and LR onto the stack
			PUSH	{R0-R3,LR}
			
			;Initialize recieve queue record structure
			LDR		R0,=RxQueue	
            LDR		R1,=RxQRecord
			LDR		R2,=RxQBuffer
			BL		InitQueue

			;Initialize transmit queue record structure
			LDR		R0,=TxQueue	
            LDR		R1,=TxQRecord
			LDR		R2,=TxQBuffer	
			BL		InitQueue
	
			;Select MCGPLLCLK / 2 as UART0 clock source
			LDR   	R0,=SIM_SOPT2     
			LDR   	R1,=SIM_SOPT2_UART0SRC_MASK     
			LDR   	R2,[R0,#0]     
			BICS  	R2,R2,R1    
			LDR   	R1,=SIM_SOPT2_UART0_MCGPLLCLK_DIV2    
			ORRS  	R2,R2,R1    
			STR   	R2,[R0,#0] 
			
			;Enable external connection for UART0     
			LDR   	R0,=SIM_SOPT5     
			LDR   	R1,=SIM_SOPT5_UART0_EXTERN_MASK_CLEAR     
			LDR   	R2,[R0,#0]     
			BICS  	R2,R2,R1    
			STR   	R2,[R0,#0] 
			
			;Enable clock for UART0 module     
			LDR   	R0,=SIM_SCGC4     
			LDR   	R1,=SIM_SCGC4_UART0_MASK     
			LDR   	R2,[R0,#0]     
			ORRS  	R2,R2,R1    
			STR   	R2,[R0,#0] 
			
			;Enable clock for Port A module     
			LDR   	R0,=SIM_SCGC5     
			LDR   	R1,=SIM_SCGC5_PORTA_MASK     
			LDR   	R2,[R0,#0]     
			ORRS  	R2,R2,R1    
			STR   	R2,[R0,#0] 
			
			;Connect PORT A Pin 1 (PTA1) to UART0 Rx (J1 Pin 02)     
			LDR     R0,=PORTA_PCR1     
			LDR     R1,=PORT_PCR_SET_PTA1_UART0_RX     
			STR     R1,[R0,#0] 
			
			;Connect PORT A Pin 2 (PTA2) to UART0 Tx (J1 Pin 04)     
			LDR     R0,=PORTA_PCR2     
			LDR     R1,=PORT_PCR_SET_PTA2_UART0_TX     
			STR     R1,[R0,#0]
			
			;Disable UART0 receiver and transmitter     
			LDR   	R0,=UART0_BASE     
			MOVS  	R1,#UART0_C2_T_R     
			LDRB  	R2,[R0,#UART0_C2_OFFSET]     
			BICS  	R2,R2,R1    
			STRB  	R2,[R0,#UART0_C2_OFFSET] 
			
			;Set UART0 IRQ priority
			LDR		R0,=UART0_IPR
			LDR		R2,=NVIC_IPR_UART0_PRI_3
			LDR		R3,[R0,#0]
			ORRS	R3,R3,R2
			STR		R3,[R0,#0]
			
			;Clear any pending UART0 interrupts
			LDR		R0,=NVIC_ICPR
			LDR		R1,=NVIC_ICPR_UART0_MASK
			STR		R1,[R0,#0]
			
			;Unmask UART0 interrupts
			LDR		R0,=NVIC_ISER
			LDR		R1,=NVIC_ISER_UART0_MASK
			STR		R1,[R0,#0]
			
			;Set UART0 for 9600 baud, 8N1 protocol   
            LDR   	R0,=UART0_BASE    
			MOVS  	R1,#UART0_BDH_9600     
			STRB  	R1,[R0,#UART0_BDH_OFFSET]     
			MOVS  	R1,#UART0_BDL_9600     
			STRB  	R1,[R0,#UART0_BDL_OFFSET]     
			MOVS  	R1,#UART0_C1_8N1     
			STRB  	R1,[R0,#UART0_C1_OFFSET]     
			MOVS  	R1,#UART0_C3_NO_TXINV     
			STRB  	R1,[R0,#UART0_C3_OFFSET]     
			MOVS  	R1,#UART0_C4_NO_MATCH_OSR_16     
			STRB  	R1,[R0,#UART0_C4_OFFSET]     
			MOVS  	R1,#UART0_C5_NO_DMA_SSR_SYNC    
			STRB  	R1,[R0,#UART0_C5_OFFSET]     
			MOVS  	R1,#UART0_S1_CLEAR_FLAGS     
			STRB  	R1,[R0,#UART0_S1_OFFSET]     
			MOVS  	R1,#UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS     
			STRB  	R1,[R0,#UART0_S2_OFFSET] 
			
			;Enable UART0 receiver and transmitter     
			MOVS  	R1,#UART0_C2_T_RI     
			STRB  	R1,[R0,#UART0_C2_OFFSET]
			
			;Pops R0-R3 and PC off the stack
			POP		{R0-R3,PC}
			ENDP
				
UART0_ISR	PROC	{R0-R13}
	;Purpose: 	Handles any interrupts that occur
	;			while the program is running
	;Calls:		Enqueue,Dequeue
	;Inputs:	
	;Outputs:	
	;Modified:	
			CPSID	I
			PUSH	{LR}
			LDR   	R1,=UART0_BASE   

			;If TxInterruptEnabled
			MOVS  	R2,#UART0_C2_TIE_MASK  
			LDRB  	R0,[R1,#UART0_C2_OFFSET]          
			TST		R0,R2         
			BEQ   	ISRDeq

			;if TxInterrupt
			MOVS  	R2,#UART0_S1_TDRE_MASK  
			LDRB  	R0,[R1,#UART0_S1_OFFSET]          
			TST		R0,R2         
			BEQ   	ISRDeq
			
			;Dequeue character from TxQueue
			LDR		R1,=TxQRecord
			BL		Dequeue
			LDR   	R1,=UART0_BASE 
			BCS		ISRDisable
			
            ;Write character to UART0 transmit data reg.
			STRB	R0,[R1,#UART0_D_OFFSET]	
			B		ISRDeq
			
ISRDisable	;disable TxInterrupt
			MOVS  	R0,#UART0_C2_T_RI  
			STRB  	R0,[R1,#UART0_C2_OFFSET]
			
			;if RxInterrupt
ISRDeq 		MOVS  	R2,#UART0_S1_RDRF_MASK  
			LDRB  	R0,[R1,#UART0_S1_OFFSET]          
			TST		R0,R2         
			BEQ   	ISREnd
			
			LDRB	R0,[R1,#UART0_D_OFFSET]
			LDR		R1,=RxQRecord
			BL		Enqueue

ISREnd		CPSIE	I
			POP		{PC}
			ENDP	
				
Init_PIT_IRQ	PROC	{R0-R13}
	;Purpose: 	Initializes the PIT for interrupts
	;Inputs:	
	;Outputs:	
	;Modified:	R14-R15,APSR
	
			;Pushes R0-R2 onto the stack
			PUSH	{R0-R2}
			
			;Set SIM_CGC6 for PIT Clock Enabled
			LDR   	R0,=SIM_SCGC6
			LDR   	R1,=SIM_SCGC6_PIT_MASK
			LDR   	R2,[R0,#0]				;current SIM_SCGC6 value
			ORRS  	R2,R2,R1				;only PIT bit set
			STR   	R2,[R0,#0]				;update SIM_SCGC6
			
			;Disable PIT timer 0
			LDR		R0,=PIT_CH0_BASE		
			LDR		R1,=PIT_TCTRL_TEN_MASK
			LDR		R2,[R0,#PIT_TCTRL_OFFSET]
			BICS	R2,R2,R1
			STR		R2,[R0,#PIT_TCTRL_OFFSET]
			
			;Set PIT interrupt priority
			LDR 	R0,=PIT_IPR	
			LDR 	R1,=(NVIC_IPR_PIT_MASK)
			LDR 	R2,[R0,#0]
			BICS 	R2,R2,R1
			STR 	R2,[R0,#0]
			
			;Clear any pending PIT interrupts
			LDR		R0,=NVIC_ICPR
			LDR		R1,=NVIC_ICPR_PIT_MASK
			STR		R1,[R0,#0]
			
			;Unmask PIT interrupts
			LDR 	R0,=NVIC_ISER
			LDR 	R1,=NVIC_ISER_PIT_MASK
			STR 	R1,[R0,#0]
			
			;Enable PIT timer module
			LDR   	R0,=PIT_BASE
			LDR   	R1,=PIT_MCR_EN_FRZ
			STR   	R1,[R0,#PIT_MCR_OFFSET]
			
			;Set PIT timer 0 period for 0.01 s
			LDR   	R0,=PIT_CH0_BASE
			LDR   	R1,=PIT_LDVAL_10ms
			STR   	R1,[R0,#PIT_LDVAL_OFFSET]
			
			;Enable PIT timer channel 0 for interrupts
			LDR		R1,=PIT_TCTRL_CH_IE
			STR		R1,[R0,#PIT_TCTRL_OFFSET]
			
			;Pops R0-R2 off the stack
			POP		{R0-R2}
			BX		LR
			ENDP
				
PIT_ISR		PROC	{R0-R13}
	;Purpose: 	Handles any PIT interrupts that occur
	;			while the program is running
	;Inputs:	
	;Outputs:	
	;Modified:	
			CPSID	I
			PUSH	{R0-R2}
			
			;If (RunStopWatch)
			LDR		R0,=RunStopWatch
			LDR		R0,[R0,#0]
			CMP		R0,#0
			BEQ		PITSkip
			
			;Increment Count
			LDR		R1,=Count
			LDR		R2,[R1,#0]
			ADDS	R2,R2,#1
			STR		R2,[R1,#0]
			
			;Clear any pending PIT interrupts
PITSkip		LDR 	R0,=PIT_CH0_BASE
			LDR 	R1,=PIT_TFLG_TIF_MASK
			STR 	R1,[R0,#PIT_TFLG_OFFSET]

			CPSIE	I
			POP		{R0-R2}
			BX		LR
			ENDP
				
Init_LED	PROC	{R0-R14}
	;Purpose: 	
	;Calls:		
	;Inputs:	
	;Outputs:	
	;Modified:	
			PUSH	{R0-R2}

			;Enable clock for PORT D and E modules
			LDR     R0,=SIM_SCGC5
			LDR     R1,=(SIM_SCGC5_PORTD_MASK :OR: SIM_SCGC5_PORTE_MASK)
			LDR     R2,[R0,#0]
			ORRS    R2,R2,R1
			STR     R2,[R0,#0]
			
			;Select PORT E Pin 29 for GPIO to red LED
			LDR     R0,=PORTE_BASE
			LDR     R1,=SET_PTE29_GPIO
			STR     R1,[R0,#PORTE_PCR29_OFFSET]
			
			;Select PORT D Pin 5 for GPIO to green LED
			LDR     R0,=PORTD_BASE
			LDR     R1,=SET_PTD5_GPIO
			STR     R1,[R0,#PORTD_PCR5_OFFSET]
			
			;Select data direction to output for red LED
			LDR  	R0,=FGPIOD_BASE
			LDR  	R1,=LED_PORTD_MASK
			STR  	R1,[R0,#GPIO_PDDR_OFFSET]
			
			;Select data direction to output for green LED
			LDR  	R0,=FGPIOE_BASE
			LDR  	R1,=LED_PORTE_MASK
			STR  	R1,[R0,#GPIO_PDDR_OFFSET]

			POP		{R0-R2}
			BX		LR
			ENDP
				
RedEnable	PROC	{R0-R14}
	;Purpose: 	Enables the red LED
	;Calls:		
	;Inputs:	
	;Outputs:	
	;Modified:	
			PUSH	{R0-R1}
			
			LDR  R0,=FGPIOE_BASE
			LDR  R1,=LED_RED_MASK
			STR  R1,[R0,#GPIO_PCOR_OFFSET]

			POP		{R0-R1}
			BX		LR
			ENDP
				
RedDisable	PROC	{R0-R14}
	;Purpose: 	Disables the red LED
	;Calls:		
	;Inputs:	
	;Outputs:	
	;Modified:	
			PUSH	{R0-R1}

			LDR  R0,=FGPIOE_BASE
			LDR  R1,=LED_RED_MASK
			STR  R1,[R0,#GPIO_PSOR_OFFSET]

			POP		{R0-R1}
			BX		LR
			ENDP
				
GreenEnable	PROC	{R0-R14}
	;Purpose: 	Enables the green LED
	;Calls:		
	;Inputs:	
	;Outputs:	
	;Modified:	
			PUSH	{R0-R1}
			
			LDR  R0,=FGPIOD_BASE
			LDR  R1,=LED_GREEN_MASK
			STR  R1,[R0,#GPIO_PCOR_OFFSET]

			POP		{R0-R1}
			BX		LR
			ENDP
				
GreenDisable	PROC	{R0-R14}
	;Purpose: 	Disables the green LED
	;Calls:		
	;Inputs:	
	;Outputs:	
	;Modified:	
			PUSH	{R0-R1}
			
			LDR  R0,=FGPIOD_BASE
			LDR  R1,=LED_GREEN_MASK
			STR  R1,[R0,#GPIO_PSOR_OFFSET]

			POP		{R0-R1}
			BX		LR
			ENDP
				
RandomGen	PROC	{R0-R14}
	;Purpose: 	Generates a random number from 0-3 based on 
	;			the least significant bit of the PIT timer
	;Calls:		DIVU
	;Inputs:	
	;Outputs:	R3 - number between 0-3
	;Modified:	
			PUSH	{R0-R2,LR}
			
			LDR		R1,=Count
			LDR		R1,[R1,#0]			;Load count value into R1
			MOVS	R2,#0x00000003		;Create ANDS mask to isolate LSB of Count
			ANDS	R1,R1,R2			;Isolate least significant bit of Count
			;MOVS	R0,#4				;Load 4 into R0
			;BL		DIVU				;Divide LSB of Count by 4, resulting in a number from 0-3
			MOVS	R3,R1				;Move number into R3 for main code

			POP		{R0-R2,PC}
			ENDP
    
GetInput	PROC	{R1-R14}	
	;Purpose:	Waits for time to expire for the round 
	;			or for the user to enter their answer.
	;Calls:		
	;Inputs: 	R2 - Round Number
	;Outputs: 	R0 - answer to the game
	;Modifies:	R0
		PUSH	{R1-R7,LR}

		;get round time limit
		MOVS	R6,#100
		MOVS	R7,R2					;R7 = Round number
		MOVS	R5,#11
		SUBS	R7,R5,R7				;R7 = 11 - Round Number
		MULS	R7,R6,R7				;R7 = R7 * 100 (convert to scale of 0.01s)

		;getting access to the queue
		LDR		R6,=RxQRecord

		CPSIE	I
		
GetInputLoop

		;did they hit a key
		LDRB	R5,[R6,#NUM_ENQD]
		CMP		R5,#0
		BNE		StopTimer				;if yes, stop the timer

		;do they still have time
		LDR		R3,=Count
		LDR		R3,[R3,#0]
		CMP		R3,R7
		BLT		GetInputLoop			;if yes, check again to see if they have entered anything
		MOVS	R0,#0					;Load zero into R0 to indicate no input
		B		NoInput

StopTimer

		;CPSID	I
		
		;stop the timer
		;LDR     R2,=RunStopWatch
		;MOVS	R1,#0
		;STRB    R1,[R2,#0]

		BL		GetChar

NoInput
		POP 	{R1-R7,PC}
		ENDP
				
ComputeScore	PROC	{R0-R14}
	;Purpose: 	Computes the new current score, by adding
	;			together the previous score, the current
	;			round number squared, and the number of 
	;			seconds left in the current round.
	;Calls:		
	;Inputs:	R2 - Round number
	;Outputs:	
	;Modified:	
			PUSH	{R0-R4,LR}
			
            LDR		R3,=Score
			LDR		R4,[R3,#0]			;Load Score into R4
			LDR		R1,=Count
			LDR		R1,[R1,#0]			;Load Count into R1
			
			MULS	R2,R2,R2			;Square the round #
			ADDS	R4,R4,R2			;Add round^2 to Score
			
			MOVS	R0,#100
			BL		DIVU				;Count = Count/100
			MOVS	R1,R0
			;MOVS	R0,#10
			;BL		DIVU				;Count = Count/1000
			;MOVS	R1,R0
			
			MOVS	R0,#11
			SUBS	R1,R0,R1			;Time left = 11 - Count
			ADDS	R4,R4,R1			;Add seconds remaining to Score
			
			STR		R4,[R3,#0]			;Store back Score
			POP		{R0-R4,PC}
			ENDP
				
GetChar		PROC	{R1-R13}
	;Purpose: 	Reads a single character from
	;			the terminal keyboard into R0
	;Calls:		Dequeue
	;Inputs:	
	;Outputs:	
	;Modified:	R0,R14-R15,APSR
	
			;Pushes R1 and LR onto the stack
			PUSH	{R1,LR}
	
			LDR		R1,=RxQRecord
	
			;Mask other interrupts    
GetLoop		CPSID	I
			
			;Dequeue character from RxQueue
			BL		Dequeue
			
			;Unmask other interrupts
			CPSIE	I
			
			;Repeat until dequeue successful
			BLO		EndGet
			B		GetLoop
			
			;Pops R1 and PC off of the stack
EndGet		POP		{R1,PC}
			ENDP
				
PutChar		PROC	{R0-R13}
	;Purpose: 	Displays a single character in
	;			R0 to the terminal screen
	;Calls:		Enqueue
	;Inputs:	
	;Outputs:	
	;Modified:	R14-R15,APSR
	
			;Pushes R0-R1 and LR onto the stack
			PUSH	{R0-R1,LR}
			
			LDR		R1,=TxQRecord
			
			;Mask other interrupts    
PutLoop		CPSID	I
			
			;Enqueue character
			BL		Enqueue
			
			;Unmask other interrupts
			CPSIE	I
			
			;Repeat until enqueue successful
			BLO		EndPut
			B		PutLoop
			
			;Enable TxInterrupt
EndPut		LDR		R0,=UART0_BASE
			MOVS  	R1,#UART0_C2_TI_RI     
			STRB  	R1,[R0,#UART0_C2_OFFSET]
		
			;Pops R0-R1 and PC off of the stack
			POP		{R0-R1,PC}
			ENDP
			
NewLine		PROC	{R0-R14}
	;Purpose: 	Print a new line and carriage return
	;Calls:		PutChar
	;Inputs:	
	;Outputs:	
	;Modified:	
			PUSH	{R0,LR}
			MOVS	R0,#13
			BL		PutChar
			MOVS	R0,#10
			BL		PutChar
			POP		{R0,PC}
			ENDP
				
PutStringSB	PROC	{R0-R14}
	;Purpose: 	Displays a null-terminated string from memory, 
	; 			starting at the address where R0 points, to the
	; 			terminal screen, for a buffer capacity of R1.
	;Calls:		PutChar
	;Inputs:	R0,R1
	;Outputs:	
	;Modified:	APSR
			PUSH	{R0-R4,LR}
	
            MOVS    R3,#0			; Initialize offset counter
			MOVS	R4,R0			; R4 = *StringPtr
            LDRB    R0,[R4,R3]
			
PutSLoop	CMP		R0,#0			; Checks if it reached end of string
			BEQ		EndPutS			; While Character != 0
            CMP     R3,R1           ; Compares current iteration to buffer capacity
            BGE     EndPutS         ; If it goes beyond the buffer, stop
			BL		PutChar			; Displays R0 to the terminal
			
			ADDS    R3,R3,#1		; R3++
            LDRB    R0,[R4,R3]		; Loads next byte value
			B		PutSLoop
	
EndPutS		POP		{R0-R4,PC}
			ENDP
				
PutNumU		PROC	{R0-R14}
	;Purpose: 	Prints the decimal representation of the value
	;			stored in R0 to the terminal screen.
	;Calls:		DIVU
	;Inputs:	R0
	;Outputs:	
	;Modified:	APSR
			PUSH	{R0-R3,LR}
			
			MOVS	R1,R0			; Moves R0 into dividend register
			MOVS	R3,#0			; Initializes pop counter
PutULoop	MOVS	R0,#10			; Moves 10 into divisor register
			BL		DIVU			; Divides R0 by 10
			
NotZeroU	MOVS	R2,R0			; Moves quotient into R2 for later check
			MOVS	R0,R1			; Moves remainder into R0 for PutChar
			
			ADDS    R0,#48			; Converts to ascii decimal number
			PUSH	{R0}			; Pushes R0 onto the stack
			ADDS	R3,R3,#1		; R3++
			
			MOVS	R1,R2			; Puts quotient back into R1
			CMP		R1,#0			; Checks if R0 is 1
			BNE		PutULoop		; Loops until R1 is 0
	
EndNumU		POP		{R0}
			BL		PutChar
			SUBS	R3,R3,#1
			CMP		R3,#0
			BNE		EndNumU
			
			POP		{R0-R3,PC}
			ENDP
	
DIVU		PROC	{R2-R14}
	;Purpose: 	Performs standard integer division,
	;			dividing R1 by R0 and putting the result
	;			into R0 and the remainder in R1.
	;Inputs:	R0,R1
	;Outputs:	R0,R1,APSR
	;Changed:	R0-R1,APSR
			CMP		R0,#0			; Compares R0 to 0
			BEQ		ZeroDenDIVU		; If R0 = 0, branch
			
			PUSH	{R2}
			PUSH	{R3}
			MOVS	R2,#0			; R2 = 0 (Result)
			
			CMP		R1,#0			; Compares R1 to 0
			BEQ		EndDIVU			; If R1 = 0, branch
			
WhileDIVU	CMP		R1,R0
			BLO		EndDIVU
			SUBS	R1,R1,R0		; R1 -= R0
			ADDS	R2,R2,#1		; Result += 1
			B		WhileDIVU		; Loop until R1-R0 < 0
			
ZeroDenDIVU	MRS   	R0,APSR			; This whole section sets the C flag,
			MOVS  	R1,#0x20		; while keeping the other flags unchanged
			LSLS  	R1,R1,#24
			ORRS  	R0,R0,R1
			MSR   	APSR,R0
			B		SkipDIVU		; Ensures nothing changes if R0 = 0
			
EndDIVU		MOVS	R0,R2			; Moves Result into R0
			
			MRS   	R2,APSR			; This whole section clears the C flag,
			MOVS  	R3,#0x20		; while keeping the other flags unchanged
			LSLS  	R3,R3,#24
			BICS  	R2,R2,R3
			MSR   	APSR,R2
			
			POP		{R3}
			POP		{R2}
SkipDIVU	BX		LR
			ENDP
				
InitQueue	PROC	{R0-R14}
	;Purpose: 	Initializes the queue record structure at the 
	;			address in R1 for the empty queue buffer at 
	;			the address in R0 of size given in R2.
	;Calls:		
	;Inputs:	R0,R1,R2
	;Outputs:	
	;Changed:	
			PUSH	{R0-R2}
			
			STR		R0,[R1,#IN_PTR]		; Moves queue address to InPointer
			STR		R0,[R1,#OUT_PTR]	; Moves queue address to OutPointer
			STR		R0,[R1,#BUF_STRT]	; Moves queue address to BufferStart
			
			ADDS	R0,R0,R2			; Adds size to queue address to obtain BufferPast
			STR		R0,[R1,#BUF_PAST]	; Moves that value to BufferPast
			STRB	R2,[R1,#BUF_SIZE]	; Moves size into BufferSize
			
			MOVS	R2,#0				
			STRB	R2,[R1,#NUM_ENQD]	; Moves 0 into NumberEnqueued
			
			POP		{R0-R2}
			BX		LR
			ENDP
				
Dequeue		PROC	{R0-R14}
	;Purpose: 	Attempts to get a character from the queue whose record 
	;			structure’s address is in R1:  if the queue is not empty, 
	;			dequeues a single character from the queue to R0, and returns 
	;			with the PSR C bit cleared to report dequeue success; otherwise, 
	;			returns with the PSRC bit set to report dequeue failure.
	;Calls:		
	;Inputs:	R1
	;Outputs:	R0,APSR
	;Changed:	R0,APSR
			PUSH	{R1-R3}
			
			LDRB	R2,[R1,#NUM_ENQD]	; Load NumberEnqueued
			CMP		R2,#0
			BNE		DeqSkipC			; If queue is empty (NumberEnqueued = 0)
			
			MRS   	R1,APSR				;
			MOVS  	R2,#0x20			;
			LSLS  	R2,R2,#24			; Set the C flag
			ORRS  	R1,R1,R2			;
			MSR   	APSR,R1				;
			
			B		EndDeq
	
DeqSkipC	SUBS	R2,R2,#1			; NumberEnqueued--
			STRB	R2,[R1,#NUM_ENQD]

			LDR		R3,[R1,#OUT_PTR]	; Load memory address at OutPointer
			LDRB	R0,[R3,#0]			; Get queue item at OutPointer
			
			ADDS	R3,R3,#1			; Increment OutPointer
			LDR		R2,[R1,#BUF_PAST]	; Load BufferPast
			CMP		R3,R2				
			BLT		DeqSkipP			; If OutPointer >= BufferPast
			LDR		R3,[R1,#BUF_STRT]	; Set OutPointer to BufferStart
			
DeqSkipP	STR		R3,[R1,#OUT_PTR]	; OutPointer = R3

			MRS  	R1,APSR				;
			MOVS 	R2,#0x20			;
			LSLS  	R2,R2,#24			; Clear the C flag
			BICS 	R1,R1,R2			;
			MSR   	APSR,R1				;
	
EndDeq		POP		{R1-R3}
			BX		LR
			ENDP

Enqueue		PROC	{R0-R14}
	;Purpose: 	Attempts to put a character in the queue whose queue record 
	;			structure’s address is in R1—if the queue is not full, enqueues 
	;			the single character from R0 to the queue, and returns with 
	;			the PSR C cleared to report enqueue success; otherwise, returns 
	;			with the PSR C bit set to report enqueue failure.
	;Calls:		
	;Inputs:	R0,R1
	;Outputs:	APSR
	;Changed:	APSR
			PUSH	{R0-R4}
			
			LDRB	R2,[R1,#NUM_ENQD]	; Load NumberEnqueued
			LDRB	R3,[R1,#BUF_SIZE]	; Load BufferSize
			CMP		R2,R3
			BNE		EnqSkipC			; If queue is full (NumberEnqueued = BufferSize)
			
			MRS   	R1,APSR				;
			MOVS  	R2,#0x20			;
			LSLS  	R2,R2,#24			; Set the C flag
			ORRS  	R1,R1,R2			;
			MSR   	APSR,R1				;
			
			B		EndEnq
	
EnqSkipC	ADDS	R2,R2,#1			; NumberEnqueued++
			STRB	R2,[R1,#NUM_ENQD]

			LDR		R3,[R1,#IN_PTR]		; Load memory address at InPointer
			STRB	R0,[R3,#0]			; Store queue item at InPointer
			
			ADDS	R3,R3,#1			; Increment InPointer
			LDR		R2,[R1,#BUF_PAST]	; Load BufferPast
			CMP		R3,R2				
			BLT		EnqSkipP			; If InPointer >= BufferPast
			LDR		R3,[R1,#BUF_STRT]	; Set InPointer to BufferStart
			
EnqSkipP	STR		R3,[R1,#IN_PTR]		; InPointer = R3

			MRS  	R3,APSR				;
			MOVS 	R4,#0x20			;
			LSLS  	R4,R4,#24			; Clear the C flag
			BICS 	R3,R3,R4			;
			MSR   	APSR,R3				;
			
EndEnq		POP		{R0-R4}
			BX		LR
			ENDP
				
PutNumHex	PROC	{R0-R14}
	;Purpose: 	This subroutine prints to the terminal screen the text 
	;			hexadecimal representation of the unsigned word value in R0.
	;Calls:		PutChar
	;Inputs:	R0
	;Outputs:	
	;Changed:	
			PUSH	{R0-R4,LR}
			
			MOVS	R1,#0			; Initialize loop counter
			MOVS	R2,#0x0000000F	; Create ANDS mask to apply after shift

PutHLoop	MOVS	R3,R0			; Copy R0 into R3
			
			LSRS	R3,R3,R1		; Shift right R1 times
			ANDS	R3,R3,R2		; Isolate least significant bit of R3
			
			CMP		R3,#9
			BGT		PutHHex			; If R3 <= 9 (decimal number)
			ADDS	R3,R3,#48		; Convert to ASCII
			B		PutHPrint
PutHHex		ADDS	R3,R3,#55		; Convert to ASCII

PutHPrint	PUSH	{R3}			; Push the value onto the stack
			
			ADDS	R1,R1,#4		; R1 = R1 + 4
			CMP		R1,#29			; Check if it's gone through every bit
			BLT		PutHLoop		; Loop until it has
			
PutPopLoop	POP		{R0}
			BL		PutChar			; Print value to terminal
			SUBS	R1,R1,#4		; R1 = R1 - 4
			CMP		R1,#0
			BNE		PutPopLoop
			
			POP		{R0-R4,PC}
			ENDP
				
PutNumUB	PROC	{R0-R14}
	;Purpose: 	This subroutine prints to the terminal screen the text 
	;			decimal representation of the unsigned byte value in R0.  
	;Calls:		PutNumU
	;Inputs:	R0
	;Outputs:	
	;Changed:	
			PUSH	{R0-R1,LR}
			
			MOVS	R1,#0x000000FF	; Create ANDS mask to isolate byte
			ANDS	R0,R0,R1		; Clears all but least significant byte
			BL		PutNumU
			
			POP		{R0-R1,PC}
			ENDP
; >>>>>   end subroutine code <<<<<
            ALIGN
;****************************************************************
;Vector Table Mapped to Address 0 at Reset
;Linker requires __Vectors to be exported
            AREA    RESET, DATA, READONLY
            EXPORT  __Vectors
            EXPORT  __Vectors_End
            EXPORT  __Vectors_Size
            IMPORT  __initial_sp
            IMPORT  Dummy_Handler
            IMPORT  HardFault_Handler
__Vectors 
                                      ;ARM core vectors
            DCD    __initial_sp       ;00:end of stack
            DCD    Reset_Handler      ;01:reset vector
            DCD    Dummy_Handler      ;02:NMI
            DCD    HardFault_Handler  ;03:hard fault
            DCD    Dummy_Handler      ;04:(reserved)
            DCD    Dummy_Handler      ;05:(reserved)
            DCD    Dummy_Handler      ;06:(reserved)
            DCD    Dummy_Handler      ;07:(reserved)
            DCD    Dummy_Handler      ;08:(reserved)
            DCD    Dummy_Handler      ;09:(reserved)
            DCD    Dummy_Handler      ;10:(reserved)
            DCD    Dummy_Handler      ;11:SVCall (supervisor call)
            DCD    Dummy_Handler      ;12:(reserved)
            DCD    Dummy_Handler      ;13:(reserved)
            DCD    Dummy_Handler      ;14:PendableSrvReq (pendable request 
                                      ;   for system service)
            DCD    Dummy_Handler      ;15:SysTick (system tick timer)
            DCD    Dummy_Handler      ;16:DMA channel 0 xfer complete/error
            DCD    Dummy_Handler      ;17:DMA channel 1 xfer complete/error
            DCD    Dummy_Handler      ;18:DMA channel 2 xfer complete/error
            DCD    Dummy_Handler      ;19:DMA channel 3 xfer complete/error
            DCD    Dummy_Handler      ;20:(reserved)
            DCD    Dummy_Handler      ;21:command complete; read collision
            DCD    Dummy_Handler      ;22:low-voltage detect;
                                      ;   low-voltage warning
            DCD    Dummy_Handler      ;23:low leakage wakeup
            DCD    Dummy_Handler      ;24:I2C0
            DCD    Dummy_Handler      ;25:I2C1
            DCD    Dummy_Handler      ;26:SPI0 (all IRQ sources)
            DCD    Dummy_Handler      ;27:SPI1 (all IRQ sources)
            DCD    UART0_ISR	      ;28:UART0 (status; error)
            DCD    Dummy_Handler      ;29:UART1 (status; error)
            DCD    Dummy_Handler      ;30:UART2 (status; error)
            DCD    Dummy_Handler      ;31:ADC0
            DCD    Dummy_Handler      ;32:CMP0
            DCD    Dummy_Handler      ;33:TPM0
            DCD    Dummy_Handler      ;34:TPM1
            DCD    Dummy_Handler      ;35:TPM2
            DCD    Dummy_Handler      ;36:RTC (alarm)
            DCD    Dummy_Handler      ;37:RTC (seconds)
            DCD    PIT_ISR		      ;38:PIT (all IRQ sources)
            DCD    Dummy_Handler      ;39:I2S0
            DCD    Dummy_Handler      ;40:USB0
            DCD    Dummy_Handler      ;41:DAC0
            DCD    Dummy_Handler      ;42:TSI0
            DCD    Dummy_Handler      ;43:MCG
            DCD    Dummy_Handler      ;44:LPTMR0
            DCD    Dummy_Handler      ;45:Segment LCD
            DCD    Dummy_Handler      ;46:PORTA pin detect
            DCD    Dummy_Handler      ;47:PORTC and PORTD pin detect
__Vectors_End
__Vectors_Size  EQU     __Vectors_End - __Vectors
            ALIGN
;****************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
Welcome		DCB		"Welcome to the Super Fun Deluxe LED Fun Game(TM)!", 0
Instruct1	DCB		"The game is split into 10 rounds, and for each round,", 0
Instruct2	DCB		"you refer to the board and see which LEDs are lit up.", 0
Instruct3	DCB		"You then type the character corresponding to the LED(s).", 0
Instruct4	DCB		"Red = R, Green = G, Both = B, None = N.", 0
Instruct5	DCB		"There are 10 rounds of increasing speed. Think you can handle it?", 0
Instruct6	DCB		"Press any key to begin. Good luck!", 0

CorrectMsg  DCB      ":  Correct--color was ", 0
WrongMsg    DCB      ":  Wrong", 0
OutOfTimeMsg   DCB  "X:  Out of time--color was ", 0
NoneMsg     DCB     "none", 0
RedMsg      DCB     "red", 0
GreenMsg    DCB     "green", 0
BothMsg     DCB     "both", 0

ThanksMsg	DCB		"Thank you for playing the Super Fun Deluxe LED Fun Game(TM)!", 0
ScoreMsg	DCB		"Your score was: ", 0
ReplayMsg	DCB		"Would you like to play again? (Y/N)", 0
GoodbyeMsg	DCB		"Goodbye!", 0

RightArrow	DCB		">", 0
LeftArrow	DCB		"<", 0
;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
Score       SPACE   4
Count		SPACE	4
RunStopWatch	SPACE	1
	
			ALIGN
String      SPACE   MAX_STRING
		
			ALIGN
RxQueue		SPACE	RxQBuffer
RxQRecord	SPACE	18

			ALIGN
TxQueue		SPACE	TxQBuffer
TxQRecord	SPACE	18
;>>>>>   end variables here <<<<<
            ALIGN
            END