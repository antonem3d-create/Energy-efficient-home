# Slow-PWM Heater Controller System 🏡⚡

**EN:** A complete hardware and software package for a distributed heating system. This project includes the STM8 firmware for the slave heaters, the PC software for the master controller, and all necessary schematics.

**RU:** Аппаратно-программный комплекс для распределенной системы обогрева. Проект включает в себя прошивку на STM8 для ведомых обогревателей, программу для ПК (мастер-контроллер) и все необходимые схемы.

## 📦 What's inside? / Что внутри?
*   **`/firmware`** — C code for the STM8S103F3 microcontroller (Slave devices).
*   **`/pc_software`** — Master control program for PC (UART sync).
*   **`/docs`** — Schematics, PCB layouts, and user manuals.

## ⚙️ Core Technology: Slow-PWM
**EN:** Instead of standard high-frequency PWM (inefficient for thermal loads), the system uses **Time-Proportional Control** (a 100-second window). The Master PC synchronizes up to 9 slave heaters via UART, allocating specific time slots and power setpoints (0-99%).

**RU:** Вместо классического высокочастотного ШИМ (неэффективного для мощных ТЭНов) используется **время-импульсное управление** (окно 100 секунд). ПК-мастер синхронизирует до 9 ведомых обогревателей по UART, выделяя каждому свой временной слот и уставку мощности (0-99%).
