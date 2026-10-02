/*
 * ethernet-to-servo-middleware-controller rev0: RP2350B (QFN-80) 칩 내장형 보드.
 * 핀 배치는 config/boards/middleware_pcb_rev0.yaml 에 있고, 여기에는 SDK 가 요구하는 칩/부트 정보만 둔다.
 *
 * 주의: rev0 PCB 에는 QSPI 플래시가 없다 (U2 QSPI 핀이 비어 있음). 플래시를 추가할 때
 * 부품에 맞춰 PICO_FLASH_SIZE_BYTES 와 부트 2단계(boot2) 선택을 고친다. 아래는 W25Q080 계열 가정.
 */
#ifndef _BOARDS_MIDDLEWARE_PCB_REV0_H
#define _BOARDS_MIDDLEWARE_PCB_REV0_H

// pico_cmake_set PICO_PLATFORM=rp2350

#define PICO_RP2350A 0              /* RP2350B: GPIO 0~47 */
#define PICO_XOSC_STARTUP_DELAY_MULTIPLIER 64   /* Y1 12 MHz 외부 크리스털 */

#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1
#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif
// pico_cmake_set_default PICO_FLASH_SIZE_BYTES = (4 * 1024 * 1024)
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (4 * 1024 * 1024)
#endif
// pico_cmake_set_default PICO_RP2350_A2_SUPPORTED = 1
#ifndef PICO_RP2350_A2_SUPPORTED
#define PICO_RP2350_A2_SUPPORTED 1
#endif

/* 디버그 출력은 USB CDC 로만 한다 (UART 는 서보 버스 전용이므로 PICO_DEFAULT_UART 를 두지 않는다) */

#endif
