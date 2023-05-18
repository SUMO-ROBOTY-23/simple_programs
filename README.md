# simple_programs

Repo na losowe programy, które mają kod gotowy do uruchomienia na naszej płytce. Na przykład kod, który obsługuje pilota.

Programy:
ir_pilot - programpokazujący przykładowe nawiązanie połaczenia z pilotem.  \\
sound_detector - program do testowania czujników dźwięku \\
motor_driver - Sterownik do silników, wymagana biblioteka "Cytron Motor Drivers Library", podpięcie pinów w komentarzach. Zmiana prędkości metodą `motor1.setSpeed(x)`, dla `x` z zakresu od -255 do 255. Dwoma silnikami można sterować niezależnie, narazie podpięty tylko silnik 1.
