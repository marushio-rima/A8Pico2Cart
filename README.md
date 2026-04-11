# A8Pico2Cart

Atari 8-bit cartridge emulator for WeAct RP2350B  
Based on [A8PicoCart](https://github.com/robinhedwards/A8PicoCart) by Robin Edwards  
Port by marushio-rima — 2025

---

## Hardware

| Componente | Especificação |
|---|---|
| MCU | WeAct RP2350B (RP2350B, Cortex-M33) |
| SRAM | 520KB |
| Flash | 8MB (versão 16MB a confirmar) |
| GPIOs | 48 disponíveis |
| USB | USB-C integrado |
| Regulador | LDO 3.3V integrado |
| Versão | XL/XE com case 3D impresso |

---

## Mapeamento GPIO

| GPIO | Sinal | Direção |
|---|---|---|
| GP0–GP12 | A0–A12 (Address bus) | Input |
| GP26 | RD4 | Input |
| GP27 | RD5 | Input |
| GP28–GP35 | D0–D7 (Data bus) | Bidirectional |
| GP36 | /CCTL | Input |
| GP37 | PHI2 | Input |
| GP38 | R/W | Input |
| GP39 | /S4 | Input |
| GP40 | /S5 | Input |
| VSYS | +5V | Power |
| GND | GND | Power |
| RUN | RESET | Input |

### Máscaras GPIO (64-bit)
```c
#define ADDR_GPIO_MASK  0x00001FFF          // GP0–GP12
#define DATA_GPIO_MASK  0x000000FF00000000  // GP28–GP35
#define DATA_SHIFT      28
#define ATARI_PHI2_PIN  37
Compilação
Requisitos
WSL2 Ubuntu (ou Linux nativo)
Pico SDK 2.1.1
arm-none-eabi-gcc 13.x
cmake 3.13+
Setup do SDK
git clone --recurse-submodules --branch 2.1.1 \
  https://github.com/raspberrypi/pico-sdk.git ~/pico-sdk
export PICO_SDK_PATH=$HOME/pico-sdk
Build
git clone https://github.com/marushio-rima/A8Pico2Cart.git
cd A8Pico2Cart/"source/Pico VSCode Project/a8_pico_cart"
mkdir build && cd build
cmake .. -DPICO_SDK_PATH=$HOME/pico-sdk
make -j4
O arquivo gerado será: build/a8pico2cart.uf2

Flash
Segure BOOTSEL no WeAct RP2350B
Conecte USB-C ao computador
Solte BOOTSEL — aparece drive RP2350
Copie a8pico2cart.uf2 para o drive
O módulo reinicia automaticamente
Uso
Conecte via USB-C sem o Atari → aparece drive A8PICO2CART
Copie arquivos .rom, .car, .xex, .atr para o drive
Ejete com segurança
Insira no Atari 800XL e ligue
Especificações da PCB
Item	Valor
Dimensões	53.8 × 70 mm
Espessura	1.6mm
Gold fingers	Chanfrados
Módulo WeAct	22.90 × 33.00 mm
Headers	2× 1×24 pinos, distância 20.32mm
Montagem	Face traseira, headers fêmea, USB-C para baixo
Referências
A8PicoCart original — Robin Edwards
PCB XL/XE original
WeAct RP2350B HDK
Licença
Baseado em A8PicoCart — MIT License
Port e modificações: marushio-rima 2025
