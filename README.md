EN
Arduino based polymer paint baking oven controller.
The control is carried out by an encoder with a button, and the information and the menu are displayed on the LCD2004 screen
The controller controls the gate, the Fan and the heater with a PID controller(https://github.com/br3ttb/Arduino-PID-Library) depending on the mode of operation.
There are automatic adjustment coefficients at a certain temperature.
The coefficients and some other data are stored in EEPROM
Overheating protection, low heating speed, RCD control is provided to open the circuit in case of solid-state relay failure.
Watchdog is set up in case the controller freezes. In the case that arduino nano is used, then in order for WDT to work properly, you need to replace the boot loader with optiboot(arduino uno), otherwise the board goes to the bootLoop.

ESP
Controlador de horno para hornear pintura polimérica a base de Arduino.
El control se lleva a cabo mediante un codificador con un botón, y la información y el menú se muestran en la pantalla LCD2004
El controlador controla la compuerta, el Ventilador y el calentador con un regulador PID(https://github.com/br3ttb/Arduino-PID-Library) dependiendo del modo de operación.
Hay coeficientes de ajuste automático a una temperatura determinada.
Los coeficientes y algunos otros datos se almacenan en EEPROM
Se proporciona protección contra sobrecalentamiento, baja velocidad de calentamiento, control RCD para abrir el circuito en caso de falla del relé de estado sólido.
Watchdog está configurado en caso de que el controlador se congele. En el caso de que se use arduino nano, entonces para que WDT funcione adecuadamente, debe reemplazar el cargador de arranque con optiboot(arduino uno), de lo contrario, la placa va al bootLoop.

RU
Контроллер печи для запекания полимерной краски на базе ардуино.
Управление осуществаляется энкодером с кнопкой, а отображается информация и меню на экране LCD2004
Контроллер управляет заслонкой, вентилятором и нагревателем с PID регулятором(https://github.com/br3ttb/Arduino-PID-Library) в зависимости от режима работы.
Есть автонастройка коэффициентов на заданную температуру.
Коэффициенты и некоторые другие данные хранятся в EEPROM
Предусмотрены защита от перегрева, низкой скорости нагрева, управление УЗО для размыкания цепи в случае пробоя твердотельного реле.
Настроен watchDog на случай зависания контроллера. В случае, если используется arduino nano, то для адекватной работы WDT необходимо заменить загрузчик на optiboot(arduino uno), иначе плата уходит в bootLoop.
