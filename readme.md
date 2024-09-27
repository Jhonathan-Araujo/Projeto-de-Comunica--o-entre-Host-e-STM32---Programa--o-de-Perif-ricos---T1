# README

## Projeto de Comunicação entre Host e STM32

Este projeto foi desenvolvido por:

- **Jhonathan Araújo Mesquita de Freitas**
- **Tomas Haddad Caldas**

No âmbito do curso de **Engenharia de Computação** da **PUCRS - Pontifícia Universidade Católica do Rio Grande do Sul**, sob a orientação do **Prof. Dr. Sérgio Johann Filho**.

---

## Sumário

- [Descrição Geral](#descrição-geral)
- [Ambiente de Desenvolvimento](#ambiente-de-desenvolvimento)
- [Aplicação Host](#aplicação-host)
  - [Funcionamento](#funcionamento-da-aplicação-host)
  - [Uso de Threads e Mutex](#uso-de-threads-e-mutex)
  - [Compilação da Aplicação Host](#compilação-da-aplicação-host)
- [Aplicação STM32](#aplicação-stm32)
  - [Funcionamento](#funcionamento-da-aplicação-stm32)
  - [Compilação da Aplicação STM32](#compilação-da-aplicação-stm32)
- [Problemas Resolvidos](#problemas-resolvidos)
- [Montagem e Desmontagem de Pacotes](#montagem-e-desmontagem-de-pacotes)
- [Conexões de Hardware](#conexões-de-hardware)
- [Vídeo Demonstrativo](#vídeo-demonstrativo)
- [Conclusão](#conclusão)
- [Instruções de Compilação](#instruções-de-compilação)

---

## Descrição Geral

O objetivo deste projeto é estabelecer uma comunicação confiável entre um computador host e uma placa STM32, permitindo:

- Controle de LEDs conectados ao STM32 a partir do host.
- Leitura do estado de switches (botões) conectados ao STM32.
- Configuração de gatilhos para notificações assíncronas de eventos.

A comunicação utiliza um protocolo personalizado que inclui framing, byte stuffing e suporte a dados binários.

---

## Ambiente de Desenvolvimento

O ambiente de desenvolvimento utilizado foi um **macOS em arquitetura ARM64**, utilizando as ferramentas **arm-none-eabi-binutils** e **arm-none-eabi-gcc**, que foram instaladas através do **Homebrew**.

Para instalar as ferramentas necessárias, utilizamos os seguintes comandos:

```bash
brew install armmbed/formulae/arm-none-eabi-gcc
brew install armmbed/formulae/arm-none-eabi-binutils
```

---

## Aplicação Host

### Funcionamento da Aplicação Host

A aplicação host é um programa em C que roda no computador e permite ao usuário interagir com o STM32 através de um menu interativo. As principais funcionalidades são:

- **Ler Entrada:** Permite ler o estado dos switches conectados ao STM32.
- **Ler Saída:** Permite ler o estado atual dos LEDs.
- **Escrever Saída:** Permite alterar o estado dos LEDs (ligar/desligar).
- **Configurar Gatilho de Entrada:** Configura gatilhos em switches para receber notificações assíncronas quando ocorrerem eventos (borda de subida, descida ou ambas).
- **Receber Notificações Assíncronas:** Recebe e processa mensagens enviadas pelo STM32 quando um evento configurado ocorre.

### Uso de Threads e Mutex

Para permitir o recebimento de mensagens assíncronas do STM32 sem interferir na interação do usuário com o menu, a aplicação host utiliza:

- **Thread Secundária (`asynchronous_reader`):**
  - Responsável por ler continuamente a porta serial e processar quaisquer mensagens recebidas do STM32.
  - Processa tanto respostas a comandos enviados quanto notificações assíncronas de eventos.
  - Permite que o usuário continue interagindo com o menu enquanto as mensagens são recebidas em segundo plano.

- **Mutex (`print_mutex`):**
  - Utilizado para sincronizar o acesso à saída padrão (`stdout`) e evitar que mensagens de diferentes threads se misturem.
  - Sempre que a thread principal ou a thread secundária precisa imprimir algo na tela, ela adquire o mutex antes de imprimir e o libera em seguida.
  - Garante que as mensagens sejam exibidas de forma ordenada e legível.

### Compilação da Aplicação Host

Para compilar a aplicação host, certifique-se de ter o GCC instalado. Utilize o seguinte comando:

```bash
gcc -pthread -o host_application host_serial.c
```

- `-pthread`: Inclui o suporte a threads POSIX.
- `-o host_application`: Especifica o nome do executável gerado.
- `host_serial.c`: Arquivo fonte da aplicação host.

---

## Aplicação STM32

### Funcionamento da Aplicação STM32

A aplicação STM32 é o firmware que roda na placa STM32. Suas principais responsabilidades são:

- **Receber Comandos do Host:**
  - Processa comandos recebidos via USB CDC.
  - Controla os LEDs de acordo com os comandos de escrita.
  - Lê o estado dos switches e envia as informações quando solicitado.

- **Enviar Notificações Assíncronas:**
  - Envia mensagens ao host quando eventos ocorrem nos switches configurados com gatilhos.
  - Utiliza interrupções para detectar mudanças nos switches.

- **Protocolo de Comunicação:**
  - Implementa framing e byte stuffing para garantir a integridade dos dados, incluindo dados binários.
  - Lida corretamente com bytes especiais, como `0x00`, evitando interpretações errôneas de fim de string.

### Compilação da Aplicação STM32

Para compilar a aplicação para o STM32, utilizamos o Makefile fornecido. Certifique-se de ter as ferramentas de compilação ARM instaladas (`arm-none-eabi-gcc` e outras).

#### Compilação

No diretório do projeto, execute:

```bash
make
```

#### Flash na Placa STM32

Para enviar o firmware compilado para a placa STM32, coloque a placa em modo DFU e execute:

```bash
make flash
```

**Makefile Utilizado:**

```makefile
CC = arm-none-eabi-gcc
AS = arm-none-eabi-as
LD = arm-none-eabi-ld
DUMP = arm-none-eabi-objdump
READ = arm-none-eabi-readelf
OBJ = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size
AR = arm-none-eabi-ar

PROG		= firmware

PROJECT_DIR	= .
HAL_DIR		= $(PROJECT_DIR)/src/hal
CMSIS_DIR	= $(PROJECT_DIR)/src/cmsis
USBCDC_DIR	= $(PROJECT_DIR)/src/usb_cdc

# this is stuff specific to this architecture
INC_DIRS  = \
	-I $(HAL_DIR) \
	-I $(CMSIS_DIR)/core \
	-I $(CMSIS_DIR)/device \
	-I $(USBCDC_DIR)

# serial port
SERIAL_DEV = /dev/cu.usbmodem00000000050C1
# uart baud rate
SERIAL_BR = 115200
# uart port
SERIAL_PORT = 0

#remove unreferenced functions
CFLAGS_STRIP = -fdata-sections -ffunction-sections
LDFLAGS_STRIP = --gc-sections

# this is stuff used everywhere - compiler and flags should be declared (ASFLAGS, CFLAGS, LDFLAGS, LD_SCRIPT, CC, AS, LD, DUMP, READ, OBJ and SIZE).
MCU_DEFINES = -mcpu=cortex-m4 -mtune=cortex-m4 -mfloat-abi=hard -mthumb -fsingle-precision-constant -mfpu=fpv4-sp-d16 -Wdouble-promotion
#MCU_DEFINES = -mcpu=cortex-m4 -mtune=cortex-m4 -mfloat-abi=soft -mthumb -fsingle-precision-constant
C_DEFINES = -D STM32F401xC -D HSE_VALUE=25000000 -D LITTLE_ENDIAN
#C_DEFINES = -D STM32F407xx -D HSE_VALUE=8000000 -D LITTLE_ENDIAN
CFLAGS = -Wall -O2 -c $(MCU_DEFINES) -mapcs-frame -fverbose-asm -nostdlib -ffreestanding $(C_DEFINES) $(INC_DIRS) -D USART_BAUD=$(SERIAL_BR) -D USART_PORT=$(SERIAL_PORT) $(CFLAGS_STRIP)

LDFLAGS = $(LDFLAGS_STRIP)
LDSCRIPT = $(HAL_DIR)/stm32f4_flash.ld

CMSIS_SRC = \
	$(CMSIS_DIR)/device/stm32f4xx_rcc.c \
	$(CMSIS_DIR)/device/stm32f4xx_gpio.c \
	$(CMSIS_DIR)/device/stm32f4xx_tim.c \
	$(CMSIS_DIR)/device/stm32f4xx_adc.c \
	$(CMSIS_DIR)/device/stm32f4xx_i2c.c \
	$(CMSIS_DIR)/device/stm32f4xx_spi.c \
	$(CMSIS_DIR)/device/stm32f4xx_usart.c \
	$(CMSIS_DIR)/device/stm32f4xx_syscfg.c \
	$(CMSIS_DIR)/device/stm32f4xx_exti.c \
	$(CMSIS_DIR)/device/misc.c \
	$(CMSIS_DIR)/device/system_stm32f4xx.c

USBCDC_SRC = \
	$(USBCDC_DIR)/usb_bsp.c \
	$(USBCDC_DIR)/usb_core.c \
	$(USBCDC_DIR)/usb_dcd.c \
	$(USBCDC_DIR)/usb_dcd_int.c \
	$(USBCDC_DIR)/usbd_cdc_core.c \
	$(USBCDC_DIR)/usbd_cdc_vcp.c \
	$(USBCDC_DIR)/usbd_core.c \
	$(USBCDC_DIR)/usbd_desc.c \
	$(USBCDC_DIR)/usbd_ioreq.c \
	$(USBCDC_DIR)/usbd_req.c \
	$(USBCDC_DIR)/usbd_usr.c

HAL_SRC = \
	$(HAL_DIR)/setjmp.s \
	$(HAL_DIR)/aeabi.s \
	$(HAL_DIR)/muldiv.c \
	$(HAL_DIR)/stm32f4_vector.c \
	$(HAL_DIR)/usart.c \
	$(HAL_DIR)/libc.c

APP_SRC = \
	$(PROJECT_DIR)/src/main.c

all: cmsis usbcdc hal app link

cmsis:
	$(CC) $(CFLAGS) $(CMSIS_SRC)

usbcdc:
	$(CC) $(CFLAGS) $(USBCDC_SRC)
		
hal:
	$(CC) $(CFLAGS) $(HAL_SRC)
	
app:
	$(CC) $(CFLAGS) $(APP_SRC)

link:
	$(LD) $(LDFLAGS) -T$(LDSCRIPT) -Map $(PROG).map -o $(PROG).elf *.o 
	$(DUMP) --disassemble --reloc $(PROG).elf > $(PROG).lst	
	$(DUMP) -h $(PROG).elf > $(PROG).sec
	$(DUMP) -s $(PROG).elf > $(PROG).cnt
	$(OBJ) -O binary $(PROG).elf $(PROG).bin
	$(OBJ) -R .eeprom -O ihex $(PROG).elf $(PROG).hex
	$(SIZE) $(PROG).elf

flash:
	dfu-util -a 0 -s 0x08000000 -D $(PROG).bin

serial:
	stty -f ${SERIAL_DEV} ${SERIAL_BR} raw cs8 -echo
	cat ${SERIAL_DEV}

clean:
	rm -rf *.o *~ firmware.*
```

---

## Problemas Resolvidos

Durante o desenvolvimento, enfrentamos alguns desafios que foram superados da seguinte forma:

### 1. Comunicação com Dados Binários

- **Problema:**
  - A comunicação entre o host e o STM32 não funcionava corretamente quando enviávamos dados binários que incluíam o valor `0x00`.
  - O código original utilizava funções que tratavam os dados como strings, interpretando o `0x00` como um terminador de string.
  - Isso fazia com que a placa reconhecesse o fim do frame antes do término real, resultando em frames incompletos.

- **Solução:**
  - Modificamos o código para utilizar funções capazes de lidar com dados binários.
  - No STM32, analisamos o arquivo `usart.c` e substituímos a função `kbhit` pela `VCP_getchar`, específica para USB CDC e adequada para dados binários.
  - Isso permitiu que toda a mensagem fosse recebida corretamente, independentemente dos valores dos bytes.

### 2. Conflito de Pinos (LED conectado ao pino A8)

- **Problema:**
  - Inicialmente, um dos LEDs estava conectado ao pino A8, que é utilizado pela USB.
  - Isso causava conflitos e problemas na comunicação USB.

- **Solução:**
  - Realocamos o LED para outro pino (A4), eliminando o conflito com a interface USB.
  - Após a realocação, a comunicação USB passou a funcionar corretamente.

---

## Montagem e Desmontagem de Pacotes

### Byte Stuffing e Framing

Para garantir a correta transmissão de dados, especialmente binários, implementamos um protocolo com as seguintes características:

- **Delimitadores de Quadro:**
  - Utilizamos o byte `0x7E` como delimitador de início e fim de quadro.
  - Isso permite identificar o início e o fim de uma mensagem.

- **Byte Stuffing:**
  - Se os dados contêm bytes iguais aos delimitadores (`0x7E`) ou ao caractere de escape (`0x7D`), aplicamos o byte stuffing.
  - **Processo:**
    - Inserimos o byte `0x7D` (escape) antes do byte a ser escapado.
    - O byte a ser escapado é substituído pelo seu valor original XOR `0x20`.

#### Exemplo de Montagem de Pacote

Dados originais: `0x01 0x7E 0x02`

1. **Aplicar Byte Stuffing:**
   - O byte `0x7E` precisa ser escapado.
   - Resultado após stuffing: `0x01 0x7D 0x5E 0x02` (onde `0x5E` é `0x7E` XOR `0x20`).

2. **Adicionar Delimitadores:**
   - Quadro final: `0x7E 0x01 0x7D 0x5E 0x02 0x7E`

### Desmontagem de Pacote (Byte Unstuffing)

Ao receber os dados, realizamos o processo inverso:

1. **Remover Delimitadores:**
   - Identificamos os bytes de início e fim do quadro (`0x7E`) e extraímos os dados entre eles.

2. **Aplicar Byte Unstuffing:**
   - Procuramos por bytes de escape (`0x7D`).
   - O byte seguinte ao escape é restaurado aplicando XOR `0x20`.

#### Exemplo de Desmontagem de Pacote

Dados recebidos: `0x7E 0x01 0x7D 0x5E 0x02 0x7E`

1. **Remover Delimitadores:**
   - Dados sem delimitadores: `0x01 0x7D 0x5E 0x02`

2. **Aplicar Byte Unstuffing:**
   - Encontramos `0x7D`, o próximo byte é `0x5E`.
   - `0x5E` XOR `0x20` = `0x7E`
   - Dados após unstuffing: `0x01 0x7E 0x02`

---

## Conexões de Hardware

### LEDs

Os LEDs estão conectados aos seguintes pinos do STM32:

- **LED1:** Pino **A7**
- **LED2:** Pino **A6**
- **LED3:** Pino **A5**
- **LED4:** Pino **A4**

### Switches (Botões)

Os switches estão conectados aos seguintes pinos do STM32:

- **SW1:** Pino **B3**
- **SW2:** Pino **B4**
- **SW3:** Pino **B5**
- **SW4:** Pino **B6**

**Observações:**

- Os switches são conectados ao **GND** quando pressionados.
- Os pinos do STM32 têm o **pull-up interno ativado**, garantindo que o estado lógico seja alto (`1`) quando o botão não está pressionado e baixo (`0`) quando pressionado.

---

## Vídeo Demonstrativo

Para visualizar o projeto em funcionamento, assista ao vídeo demonstrativo no YouTube:

[![Vídeo Demonstrativo](https://img.youtube.com/vi/LOvI9sV6q8Q/0.jpg)](https://youtu.be/LOvI9sV6q8Q)

Link direto: [https://youtu.be/LOvI9sV6q8Q](https://youtu.be/LOvI9sV6q8Q)

---

## Conclusão

Este projeto permitiu a implementação de uma comunicação confiável entre um host e uma placa STM32, lidando com dados binários e garantindo a integridade das mensagens através de um protocolo com framing e byte stuffing. Os desafios encontrados nos proporcionaram um aprendizado significativo na área de comunicação serial e programação de sistemas embarcados.

A utilização de threads e mutexes na aplicação host aprimorou nossa compreensão sobre programação concorrente, permitindo um design mais robusto e responsivo.

Este trabalho foi muito valioso para aprofundarmos nossos conhecimentos na área, e os problemas enfrentados nos incentivaram a pesquisar e nos aprofundar ainda mais nos conceitos envolvidos.

---

## Instruções de Compilação

Reunindo as informações apresentadas anteriormente, aqui estão as instruções de compilação resumidas:

### Compilação da Aplicação Host

Certifique-se de ter o GCC instalado em sua máquina host. Utilize o seguinte comando para compilar a aplicação:

```bash
gcc -pthread -o host_application host_serial.c
```

- **Executando a Aplicação Host:**

  ```bash
  ./host_application
  ```

### Compilação da Aplicação STM32

Certifique-se de ter as ferramentas ARM instaladas (`arm-none-eabi-gcc`, `dfu-util`, etc.). No macOS em ARM64, as ferramentas foram instaladas através do **Homebrew**:

```bash
brew install armmbed/formulae/arm-none-eabi-gcc
brew install armmbed/formulae/arm-none-eabi-binutils
```

- **Compilação:**

  No diretório do projeto, execute:

  ```bash
  make
  ```

- **Flash na Placa STM32:**

  Coloque a placa STM32 em modo DFU e execute:

  ```bash
  make flash
  ```

---

**Observação:** Certifique-se de que todas as dependências e ferramentas necessárias estejam instaladas em seu sistema. Se houver quaisquer dúvidas ou problemas durante a compilação ou execução, consulte a documentação das ferramentas utilizadas ou entre em contato para suporte adicional.