# simple_programs

Repo na losowe programy, które mają kod gotowy do uruchomienia na naszej płytce. Na przykład kod, który obsługuje pilota.<br />

Programy:
ir_pilot - programpokazujący przykładowe nawiązanie połaczenia z pilotem. <br />
sound_detector - program do testowania czujników dźwięku. <br />
motor_driver - Sterownik do silników, wymagana biblioteka "Cytron Motor Drivers Library", podpięcie pinów w komentarzach. Zmiana prędkości metodą `motor1.setSpeed(x)`, dla `x` z zakresu od -255 do 255. Dwoma silnikami można sterować niezależnie, narazie podpięty tylko silnik 1.<br />
pilot_names.h - biblioteka z definicjami kodów pilota (int code =  IrReceiver.decodedIRData.command;)<br />
