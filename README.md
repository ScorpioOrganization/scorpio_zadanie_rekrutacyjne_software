# Projekt Scorpio - zadanie rekrutacyjne do działu Software

Cześć! 
Przed Tobą opis zadania rekrutacyjnego do naszego koła naukowego! :)
Celem zadania jest stworzenie oprogramowania obsługującego stworzony przez nas symulator silnika z enkoderem. W zadaniu trzeba skorzystać z Robot Operating System (ROS). 
>**Uwaga!** Przed przystąpieniem do realizacji zadania przeczytaj **całe** README.
## Spis treści
- [Zadania do wykonania](#zadania-do-wykonania)
- [Specyfikacja techniczna zadania](#specyfikacja-techniczna-zadania)
  - [Dane ogólne](#dane-ogólne)
  - [Specyfikacja danych](#specyfikacja-danych)
  - [Uruchamianie symulatora](#uruchamianie-symulatora)
- [Wskazówki i przydatne linki](#wskazówki-i-przydatne-linki)
## Zadania do wykonania 
W tej części znajdziesz ogólny opis zadań, szczegółowy opis wraz ze specyfikacją techniczną znajdziesz w sekcji [specyfikacja techniczna zadania](#specyfikacja-techniczna-zadania).

1. W repozytorium, na którego stronie się właśnie najprawdopodobniej znajdujesz ([link](https://github.com/ScorpioOrganization/scorpio_zadanie_rekrutacyjne_software)) została przygotowana paczka ROS zawierająca napisany przez nas symulator silnika z enkoderem absolutnym. Repozytorium należy sklonować i zbudować paczkę w ROS. Szczegóły działania paczki są opisane w sekcji [specyfikacja techniczna zadania](#specyfikacja-techniczna-zadania).
> **Wskazówka!** Dobrym rozwiązaniem jest "fork" paczki!

2. Stwórz node ROS, który pobiera dane o pozycji silnika z topicu `/virtual_dc_motor/get_position`, a następnie oblicza prędkość kątową silnika (w RPM) i publikuje to na topic `/virtual_dc_motor_driver/get_velocity`.

3. Stwórz node ROS, który pozwoli na sterowanie prędkością silnika na podstawie odczytów uzyskanych w poprzednim etapie. Docelowa prędkość powinna być subskrybowana z topicu `/virtual_dc_motor_controller/set_velocity_goal`. Sterowanie powinno polegać na wysłaniu odpowiednich wiadomości na topic `/virtual_dc_motor/set_cs` w celu osiągnięcia i utrzymania wymaganej prędkości.

## Specyfikacja techniczna zadania
### Dane ogólne
**Nazwa paczki ROS** - `virtual_dc_motor`  

**virtual_dc_motor** - przygotowany przez nas node symulujący silnik prądu stałego z enkoderem absolutnym. Jego kod znajdziesz w `include/virtual_dc_motor/virtual_dc_motor.hpp` oraz `src/virtual_dc_motor.cpp`.
> **Uwaga!** Nie modyfikuj tych plików, w celu wykonania zadania stwórz nowe pliki, w których zawrzesz node'y. Dla plików C++ w katalogu `src`, dla plików Python w katalogu `scripts`.

### Specyfikacja danych
Node `virtual_dc_motor` subskrybuje dane z topicu `/virtual_dc_motor/set_cs`. Jest to sygnał sterujący (_ang._ Control Signal, CS), który pozwala na sterowanie silnikiem. Są to 8-bitowe dane całkowitoliczbowe (w ROS - Int8) **w zakresie od -100 do 100**(!). Znak tej wartości decyduje o kierunku obrotu silnika, a moduł (wartość bezwzględna) wartości o mocy silnika. Wysłanie wartości spoza zakresu nie będzie miało żadnego efektu. 

Na topicu `/virtual_dc_motor/get_position` znajdują się dane informujące o aktualnej pozycji silnika odczytanej za pomocą enkodera absolutnego. Dane te są 12 bitowe (w zakresie 0 - 4095), oczywiście w przypadku przekroczenia zakresu wartość się "przekręca" (np. 4094 -> 4095 -> 0 -> 1 przy obrocie w prawo).

Dane o prędkości silnika powinny zostać wysłane na topic `/virtual_dc_motor_driver/get_velocity` w formacie Float32.

> **Uwaga!** Symulator stara się wiernie oddać zachowanie silnika, dlatego wartości sygnału sterującego bliskie zeru (|cs| < 13) nie zapewnią odpowiedniej mocy do wprawienia silnika w ruch.

### Uruchamianie symulatora
Po zbudowaniu paczki symulator silnika można uruchomić dzięki launchfile za pomocą komendy:
```bash
roslaunch virtual_dc_motor virtual_dc_motor.launch
```
> **Uwaga!** Pamiętaj, że po zbudowaniu należy również wykonać `source devel/setup.bash` w workspace ROS!

> **Wskazówka!** Do przygotowanego launchfile (`launch/virtual_dc_motor.launch`) możesz dodać również swoje node, aby uruchamiać wszystko za jednym razem :)

## Wskazówki i przydatne linki
-	W celu realizacji zadania konieczne będzie zainstalowanie ROS w wersji Noetic (zalecany system operacyjny to Ubuntu 20.04) lub ROS w wersji Melodic (Ubuntu 18.04).
-	Zachęcamy do zapoznania się z poradnikiem przedstawiającym podstawy pracy w ROS: www.youtube.com/watch?v=wfDJAYTMTdk&ab_channel=RoboticsBack-End
-	Oficjalny tutorial ROS znajdziesz pod linkiem: wiki.ros.org/ROS/Tutorials
-	Do instalacji ROS można wykorzystać instrukcję (należy wybrać wersję desktop-full install): http://wiki.ros.org/noetic/Installation/Ubuntu 
- Do ręcznego wysłania danych na topic w ROS możesz użyć komendy `rostopic pub <nazwa_topicu> <typ_danych> <dane>` (po wpisaniu nazwy topicu dobrze jest od razu użyć TAB aby powłoka pomogła w wpisywaniu i zajęła się typem danych i formatem). Możesz to wykorzystać do weryfikacji działania node'a którego otrzymałeś oraz swoich node'ów.
- Możesz użyć komendy `rostopic echo <nazwa_topicu>` aby wyświetlić dane wysyłane na określony topic.
- Zadanie rekrutacyjne można oddać niepełne.
- Rozwiązane zadanie należy umieścić w repozytorium (np. GitHub) i przesłać linka do tego repozytorium na mail projekt@scorpio.pwr.edu.pl. Ewentualne pytania lub wątpliwości co do treści zadania można kierować na tego samego maila. Zadania przyjmujemy do 22.10.2023 do końca dnia.

**Jeżeli będziesz miał jakiekolwiek wątpliwości i problemy z zadaniem śmiało skontaktuj się z nami! :)**
