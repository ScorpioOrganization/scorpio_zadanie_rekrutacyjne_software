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
  - [Uwagi](#uwagi)
  - [Uruchamianie symulatora](#uruchamianie-symulatora)
- [Wskazówki i przydatne linki](#wskazówki-i-przydatne-linki)
## Zadania do wykonania 
W tej części znajdziesz ogólny opis zadań, szczegółowy opis wraz ze specyfikacją techniczną znajdziesz w sekcji [specyfikacja techniczna zadania](#specyfikacja-techniczna-zadania).

1. W repozytorium, na którego stronie się właśnie najprawdopodobniej znajdujesz ([link](https://github.com/ScorpioOrganization/scorpio_zadanie_rekrutacyjne_software)) została przygotowana paczka ROS zawierająca napisany przez nas symulator silnika z enkoderem absolutnym. Repozytorium należy sklonować i zbudować paczkę w ROS. Szczegóły działania paczki są opisane w sekcji [specyfikacja techniczna zadania](#specyfikacja-techniczna-zadania).
> **Wskazówka!** Dobrym rozwiązaniem jest "fork" paczki!

2. Stwórz aplikacje webową (z użyciem HTML, CSS, JS, oraz ewentualnego Reacta), która wyświetli aktualne pozycje silników (są one do pobrania na ROSowych topicach `/virtual_dc_motor_node/set_cs_0`, `/virtual_dc_motor_node/set_cs_1`, `/virtual_dc_motor_node/set_cs_2`) oraz ich prędkości w RPM (te należy obliczyć na podstawie pozycji).

> **Uwaga!** Silnik pierwotnie się nie rusza, więc będzie wysyłał pozycję `0` `+/-1` (czyli 4095, 0, 1 - więcej w sekcji [Uwagi](#uwagi))

3. Do istniejącej aplikacji webowej dodaj możliwość sterowania silnikami za pomocą klawiszy (sterowanie przebiega poprzez publikowanie wartości z zakresu [-100;100] na jesden z topiców `/virtual_dc_motor_node/set_cs_0`, `/virtual_dc_motor_node/set_cs_1`, `/virtual_dc_motor_node/set_cs_2`). Pamiętaj, ażeby dodać przycisk pozwalający wyłączyć sterowanie (aplikacja ma przestać reagować na przyciskanie klawiszy), w celu uniknięcia przypadkowych ruchów silnikiem.

> **Wskazówka!** Pamiętaj o regularnym commitowaniu zmian

4. Dodaj suwak pozwalający na regulacje mocy (wartości wysyłanej na topic).

5. Wykonaj prostą wizualizację silników.

6. Do silnika z indexem `0` jest podłączony silnik z indexem `1`, do któego z kolei jest podłączomy silnik z indexem `2`. Długości tych połączeń można pobrać z serwisu `/virtual_dc_motor_node/get_joints_length`. Pobierz te informacje i je wyświel.

> **Uwaga!** Serwis za każdym razem zwraca inne losowe wartości (ma to Ciebie zmusić do dynamicznego dostosowywania parametrów w dalszej części zadania)

7. Uzależnij od siebie pozycję silników.

8. Dodaj przycisk, który sprawi, iż powstała w ten sposób konstrukcja się wyprostuje (pozcyje silników będą ustawione na `0`).

## Specyfikacja techniczna zadania
> **Uwaga!** Nie modyfikuj plików znajdujących się w paczce ROS.
### Dane ogólne
**Nazwa paczki ROS** - `virtual_dc_motor`  

**virtual_dc_motor** - przygotowany przez nas node symulujący silnik prądu stałego z enkoderem absolutnym. Jego kod znajdziesz w `include/virtual_dc_motor/virtual_dc_motor.hpp` oraz `src/virtual_dc_motor.cpp`.

### Specyfikacja danych
Node `virtual_dc_motor` subskrybuje dane z topicu `/virtual_dc_motor_node/set_cs_0`, `/virtual_dc_motor_node/set_cs_1`, `/virtual_dc_motor_node/set_cs_2`. Jest to sygnał sterujący (_ang._ Control Signal, CS), który pozwala na sterowanie poszczególnymi silnikami. Są to 8-bitowe dane całkowitoliczbowe (w ROS - Int8) **w zakresie od -100 do 100**(!). Znak tej wartości decyduje o kierunku obrotu silnika, a moduł (wartość bezwzględna) wartości o mocy silnika. Wysłanie wartości spoza zakresu nie będzie miało żadnego efektu. 

Na topicu `/virtual_dc_motor_node/get_position_0`, `/virtual_dc_motor_node/get_position_1`, `/virtual_dc_motor_node/get_position_2` znajdują się dane informujące o aktualnej pozycji poszczególnych silników odczytanej za pomocą enkodera absolutnego. Dane te są 12 bitowe (w zakresie 0 - 4095), oczywiście w przypadku przekroczenia zakresu wartość się "przekręca" (np. 4094 -> 4095 -> 0 -> 1 przy obrocie w prawo - podobnie jak zegar).

### Uwagi:
 - Wartości podawane przez encoder mają symulować rzeczywiste odczyty - zatem mogą się one wahać o `+/- 1`
 - Symulator stara się wiernie oddać zachowanie silnika, dlatego wartości sygnału sterującego bliskie zeru (|cs| < 13) nie zapewnią odpowiedniej mocy do wprawienia silnika w ruch.

### Uruchamianie symulatora
Po zbudowaniu paczki symulator silnika można uruchomić dzięki launchfile za pomocą komendy:
```bash
roslaunch virtual_dc_motor virtual_dc_motor.launch
```
> **Uwaga!** Pamiętaj, że po zbudowaniu należy również wykonać `source devel/setup.bash` w workspace ROS!

## Wskazówki i przydatne linki
-	W celu realizacji zadania konieczne będzie zainstalowanie ROS w wersji Noetic (zalecany system operacyjny to Ubuntu 20.04) lub ROS w wersji Melodic (Ubuntu 18.04).
-	Zachęcamy do zapoznania się z poradnikiem przedstawiającym podstawy pracy w ROS: www.youtube.com/watch?v=wfDJAYTMTdk&ab_channel=RoboticsBack-End
-	Oficjalny tutorial ROS znajdziesz pod linkiem: wiki.ros.org/ROS/Tutorials
-	Do instalacji ROS można wykorzystać instrukcję (należy wybrać wersję desktop-full install): http://wiki.ros.org/noetic/Installation/Ubuntu 
- Do ręcznego wysłania danych na topic w ROS możesz użyć komendy `rostopic pub <nazwa_topicu> <typ_danych> <dane>` (po wpisaniu nazwy topicu dobrze jest od razu użyć TAB aby powłoka pomogła w wpisywaniu i zajęła się typem danych i formatem). Możesz to wykorzystać do weryfikacji działania node'a którego otrzymałeś oraz swoich node'ów.
- Możesz użyć komendy `rostopic echo <nazwa_topicu>` aby wyświetlić dane wysyłane na określony topic.
- Zadanie rekrutacyjne można oddać niepełne.
- Rozwiązane zadanie należy umieścić w repozytorium (np. GitHub) i przesłać linka do tego repozytorium na mail projekt@scorpio.pwr.edu.pl. Ewentualne pytania lub wątpliwości co do treści zadania można kierować na tego samego maila. Zadania przyjmujemy do 31.03.2024 do końca dnia.

**Jeżeli będziesz miał jakiekolwiek wątpliwości i problemy z zadaniem śmiało skontaktuj się z nami! :)**
