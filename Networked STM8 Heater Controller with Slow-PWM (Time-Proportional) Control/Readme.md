What is it?
This project is a firmware for a slave heater controller forming a distributed heating network. Instead of standard high-frequency PWM (which is inefficient for high-power thermal loads), it uses Time-Proportional Control (a 100-second window).
The controller synchronizes with a Master PC via UART, receiving its specific time slot and power setpoint (0-99%), allowing for precise, centralized management of multiple heaters without electrical interference.

Что это такое?
Прошивка для ведомого контроллера нагревателя, работающего в распределенной сети. Вместо классического высокочастотного ШИМ (который неэффективен для мощных ТЭНов) используется время-импульсное управление (окно 100 секунд).
Устройство синхронизируется с ПК-мастером по UART, получая свой временной слот и уставку мощности (0-99%). Это позволяет централизованно управлять сетью обогревателей без радиопомех и перегрузки электросети.
