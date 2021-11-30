# furnace
Furnace controller. Based on arduino.

# Контроллер печи для запекания полимерной краски на базе ардуино. 
# Управление осуществаляется энкодером с кнопкой, а отображается информация и меню на экране LCD2004
# Контроллер управляет заслонкой, вентилятором и нагревателем с PID регулятором в зависимости от режима работы.
# Есть автонастройка коэффициентов на заданную температуру.
# Коэффициенты и некоторые другие данные хранятся в EEPROM
# Предусмотрены защита от перегрева, низкой скорости нагрева, управление УЗО для размыкания цепи в случае пробоя твердотельного реле. 
# Настроен watchDog на случай зависания контроллера. В случае, если используется arduino nano, то для адекватной работы WDT необходимо заменить загрузчик на optiboot(arduino uno), иначе плата уходит в bootLoop.
