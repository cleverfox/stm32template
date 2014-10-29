NAME   = test

PREFIX  = arm-none-eabi

CC      = $(PREFIX)-gcc
LD      = $(PREFIX)-ld -v
AR      = $(PREFIX)-ar
AS      = $(PREFIX)-as
CP      = $(PREFIX)-objcopy
OD	= $(PREFIX)-objdump
SER	= md
  
CFLAGS  =  -I./ -Ilib -c -fno-common -O0 -g -mcpu=cortex-m3 -mthumb 
#-ffunction-sections -fdata-sections 
#-ffunction-sections
#-ffunction-sections -fdata-sections 
AFLAGS  = -ahls -mapcs-32 -o crt.o
LFLAGS  = -Tld.cmd -nostartfiles #--gc-sections
ODFLAGS	= -S

all: test
	$(PREFIX)-size $(NAME).out

clean:
	-rm main.o $(NAME).out $(NAME).hex $(NAME).bin $(NAME).list *.o

test: $(NAME).out
	@ echo "...copying"
	$(CP) -Obinary $(NAME).out $(NAME).bin
	$(CP) -Oihex $(NAME).out $(NAME).hex
	$(OD) $(ODFLAGS) $(NAME).out > $(NAME).list

$(NAME).out: main.o stm32f10x_rcc.o stm32f10x_gpio.o stm32f10x_exti.o stm32f10x_tim.o misc.o ld.cmd startup.o usart.o stm32f10x_usart.o stm32f10x_pwr.o stm32f10x_bkp.o stm32f10x_rtc.o
	@ echo "..linking"
	$(LD) $(LFLAGS) -Map=$(NAME).map -o $(NAME).out  main.o stm32f10x_rcc.o stm32f10x_gpio.o stm32f10x_exti.o stm32f10x_tim.o misc.o startup.o usart.o stm32f10x_usart.o stm32f10x_pwr.o stm32f10x_bkp.o stm32f10x_rtc.o

startup.o: lib/startup_stm32f10x_$(SER).c
	$(CC) $(CFLAGS) lib/startup_stm32f10x_$(SER).c -o startup.o

stm32f10x_usart.o: lib/stm32f10x_usart.c  
	@ echo ".compiling"
	$(CC) $(CFLAGS) lib/stm32f10x_usart.c 

stm32f10x_tim.o: lib/stm32f10x_tim.c  
	@ echo ".compiling"
	$(CC) $(CFLAGS) lib/stm32f10x_tim.c 

stm32f10x_rcc.o: lib/stm32f10x_rcc.c 
	@ echo ".compiling"
	 $(CC) $(CFLAGS) lib/stm32f10x_rcc.c 
	 
misc.o: lib/misc.c
	 $(CC) $(CFLAGS) lib/misc.c 
stm32f10x_gpio.o: lib/stm32f10x_gpio.c
	 $(CC) $(CFLAGS) lib/stm32f10x_gpio.c 
stm32f10x_exti.o: lib/stm32f10x_exti.c
	 $(CC) $(CFLAGS) lib/stm32f10x_exti.c 
stm32f10x_pwr.o: lib/stm32f10x_pwr.c
	 $(CC) $(CFLAGS) lib/stm32f10x_pwr.c 
stm32f10x_bkp.o: lib/stm32f10x_bkp.c
	 $(CC) $(CFLAGS) lib/stm32f10x_bkp.c 
stm32f10x_rtc.o: lib/stm32f10x_rtc.c
	 $(CC) $(CFLAGS) lib/stm32f10x_rtc.c 

main.o: main.c
	@ echo ".compiling"
	 $(CC) $(CFLAGS) main.c
