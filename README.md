# Projekt Scorpio - zadanie rekrutacyjne do działu Software
W celu realizacji zadania konieczne będzie zainstalowanie ROS w wersji Noetic (zalecany system operacyjny to Ubuntu 20.04) lub ROS w wersji Melodic (Ubuntu 18.04).
Na repozytorium znajduje się paczka ROS zawierająca symulację jazdy autonomicznej łazika.
>**Uwaga!** Przed przystąpieniem do realizacji zadania przeczytaj **całe** README.
## Spis treści
- [Zadania do wykonania](#zadania-do-wykonania)
- [Specyfikacja techniczna zadania](#specyfikacja-techniczna-zadania)
  - [Dane ogólne](#dane-ogólne)
  - [Specyfikacja danych](#specyfikacja-danych)
  - [Uwagi](#uwagi)
  - [Uruchamianie symulatora](#uruchamianie-symulatora)
- [Wskazówki i przydatne linki](#wskazówki-i-przydatne-linki)
- [Przydatne ROSowe komendy CLI](#przydatne-rosowe-komendy-cli)
## Zadania do wykonania 
W tej części znajdziesz ogólny opis zadań, szczegółowy opis wraz ze specyfikacją techniczną znajdziesz w sekcji [specyfikacja techniczna zadania](#specyfikacja-techniczna-zadania).

Pamiętaj, że zadanie służy sprawdzeniu wielu umiejętności - nie tylko programowania i znajomości algorytmów -  więc nawet w przypadku zrealizowania tylko części z poniższych punktów, zachęcamy do przesłania rozwiązania. Postępy w zadaniu powinny być udokumentowane w repozytorium na githubie (po każdym etapie zadania powinien zostać stworzony nowy commit).

> **Uwaga!** Kolejność wykonania zadań nie jest ważna

1. W repozytorium została przygotowana paczka ROS zawierająca napisaną przez nas symulacją jazdy autonomicznej łazika. Repozytorium należy sklonować i zbudować paczkę w ROS. Szczegóły działania paczki są opisane w sekcji [specyfikacja techniczna zadania](#specyfikacja-techniczna-zadania).
> **Wskazówka!** Dobrym rozwiązaniem jest "fork" paczki

2. Stwórz node ROSowy, który zaplanuje ścieżkę (jak najszybszą) dojazdu do zadanego na topic `/set_goal` punktu mapy (x,y), a następnie dojedzie do danego punktu poprzez zadawanie odpowiednich komend ruchu na topic `/rover/move`. Node ma subskrybować topic `/set_goal` który ma własny typ SetGoal zawierający koordynaty x i y w postaci dodatnich liczb całkowitych (UInt8). Cel może być zadawany w terminalu za pomocą komendy `rostopic pub`

> **Uwaga!** tutaj przyjmujemy, że teren, po którym porusza się łazik jest płaski i nie ma na nim żadnych przeszkód (mapa jest pusta). Ważnym jest jednak, aby łazik nie wyjechał poza teren mapy - więcej w sekcji [Uwagi](#uwagi))

3. Wewnątrz stworzonego node'a pobierz za pomocą serwisu `/get_map` obecną mapę całości terenu. Ta mapa jest mapą wysokości - zawiera elewację terenu, jak i nieprzejezdne przeszkody. Trzeba uzupełnić moduł planowania ścieżki tak, aby łazik był w stanie zaplanować i przejechać ścieżkę zaplanowaną podobnie jak w punkcie poprzednim. Jeżeli zadany cel nie jest możliwy do osiągnięcia należy na ROS_ERROR wypisać komunikat, że dojazd do celu jest niemożliwy, zaniechać wszelką jazdę i czekać na kolejne zadanie celu.

> **Wskazówka!** Pamiętaj o regularnym commitowaniu zmian

4. Łazik podobnie jak w poprzednim punkcie porusza się po mapie wysokości, jednak jest ona nieznana. Na topicu `/rover/sensor` znajdują się dane o wysokości względnej punktów 2x3 komórki mapy przed łazikiem. Są to dane z symulowanego czujnika służącego do mapowania terenu na łaziku. Podobnie jak wcześniej zadawany jest cel dojazdu łazika i należy zaplanować ścieżkę na mapie i próbować dojechać do celu. Ścieżka powinna być aktualizowana na bieżąco w trakcie przejazdu na podstawie nowych danych. Należy stworzyć i na bieżąco po każdym ruchu aktualizować mapę uzupełnioną przez dane z czujnika. Na tej mapie należy bazować planowanie ścieżki. Dodatkowo mapa powinna być publikowana na topic `/rover/map`. Podobnie jak w punkcie poprzednim należy obsłużyć niemożliwy dojazd do celu.

> **Uwaga!** wysokość względna oznacza że łazik znajdując się na komórce o wysokości 10 i widząc przed sobą komórkę o wysokości 20 to wysokość komórki przed łazikiem wynosi 30 i jako taką wartość należy ją do mapy zapisać.

## Specyfikacja techniczna zadania
> **Uwaga!** Nie modyfikuj plików utworzonych przez nas znajdujących się w paczce ROS.
### Informacje ogólne

Łazik porusza się po mapie 50x50 zawsze zaczynając w pozycji (0,0) - lewy dolny róg mapy. Punkt (0,0) zawsze znajduje się na wysokości 0. Na początku łazik zawsze zorientowany jest na północ.

Komórki mapy wysokości, po której łazik porusza się w zadaniach 2 i 3 zawierają wartości liczbowe ze zbioru (0,10,20,30,40,50,100) - jest to odwzorowanie nierównego terenu, po którym łazik się przemieszcza. Łazik może przy jednym ruchu maksymalnie zmienić wysokość o 10 np:
- będąc na komórce o wysokości 10 może wjechać na komórkę o wysokości 0, 10 lub 20
- będąć w komórce o wysokości 0 może wjechać na komórkę o wysokości 0 lub 10
- będąc w komórce o wysokości 50 może wjechać na komórkę o wysokości 50 lub 40 (nie 30, 20, 10 ani 0)
- łazik nigdy nie może wjechać na komórkę o wysokości 100

W zadaniu 3 należy w pamięci przechowywać własną mapę i publikować ją na topic `/rover/map`. Na topic należy publikować tablicę **jednowymiarową** ośmiobitowych liczb całkowitych (Int8). Do tego należy wykorzystać stworzony przez nas własny typ wiadomości ROSowej - RoverMap. Tablicę należy uzupełnić w sposób identyczny do mapy pozyskiwanej za pomocą serwisu `/get_map` opisanego poniżej.

**Nazwa paczki ROS** - `autonomy_simulation`  

**autonomy_simulation** - przygotowany przez nas node symulujący łazika i teren, po którym ma on przejechać. Jego kod znajdziesz w `include/autonomy_simulation/autonomy_simulation.hpp` oraz `src/autonomy_simulation.cpp`.

### Specyfikacja danych
Node `autonomy_simulation` subskrybuje dane z topicu `/rover/move`. Jest to komenda sterująca łazikiem. Są to dodatnie 8-bitowe dane całkowitoliczbowe (w ROS - UInt8) **w zakresie od 0 do 3 włącznie**. Każdy numer odpowiada innemu ruchowi, a mapowania są następujące:
- 0: obrót łazika w miejscu w lewo o 90 stopni
- 1: obrót łazika w miejscu w prawo o 90 stopni
- 2: jazda do przodu
- 3: jazda do tyłu
Topic ma własny typ RoverMove - jego szczegóły można znaleźć w `msg/RoverMove.msg`
**Na topic `/rover/move` nie należy wysyłać danych częściej niż 10 razy na sekundę**

Na topic `/rover/pose` publikowane są dane o obecnej pozycji łazika - zawiera 3 pola (wszystkie są 8-bitowymi danymi całkowitoliczbowymi - Int8):
- x: pozycja x na mapie
- y: pozycja x na mapie
- orientation: 1-północ, 2-wschód, 3-południe, 4-zachód

Dodatkowo jeżeli łazik znajdzie się w pozycji niedozwolonej (w przeszkodzie, na którą nie powinien wjechać lub poza mapą) to wszystkie pola będą miały wartość -1, a łazik nie będzie przyjmować już wtedy poleceń ruchu. W takiej sytuacji należy zrestartować symulację i spróbować wykonać przejazd ponownie. Pozycja jest publikowana 10 razy na sekundę.
Topic ma własny typ RoverPose - jego szczegóły można znaleźć w `msg/RoverPose.msg`

Node rozgłasza serwis `/get_map`, który pozwala pobrać mapę terenu wymaganą do zadania 2. Informacje zwracane przez mapę są w postaci tablicy **jednowymiarowej** ośmiobitowych liczb całkowitych (Int8). Każdy rząd w tej tablicy jest wypisany po kolei. To oznacza że:
- komórka (0,0) będzie się znajdować na pierwszym miejscu tablicy (indeks 0)
- komórka (0,1) będzie się znajdować na drugim miejscu tablicy (indeks 1)
- komórka (1,0) będzie się znajdować na pięćdziesiątym pierwszym miejscu tablicy (indeks 50)
- w sumie tablica będzie miała 2500 pól, gdzie ostatnie (o indeksie 2499) będzie oznaczać pole (50,50)
Serwis ma własny typ GetMap - jego szczegóły można znaleźć w `srv/GetMap.srv`

Do zadania 3 node na topic `/rover/sensor` będą wysyłane 10 razy na sekundę informacje o terenie przed łazikiem (3x2 komórki). Topic ma własny typ RoverMap - jego szczegóły można znaleźć w `msg/RoverMap.msg`. Jest on efektywnie małą mapą widzianą przed łazikiem i czytana jest podobnie jak wcześniej opisane mapy. Należy jednak pamiętać, że pokrycie danych z czujnika z mapą rzeczywistą będzie zależeć od rotacji. Indeksy komórek publikowanej przez sensor tablicy jednowymiarowej w zależności od rotacji łazika wizualizuje poniższy rysunek (X to pozycja łazika):

![Wizualizacja danych z sensora](img/zad3_sensor.png)

### Uruchamianie symulatora
Po zbudowaniu paczki symulator można uruchomić dzięki launchfile za pomocą komendy:
```bash
roslaunch autonomy_simulation autonomy_simulation.launch
```
> **Uwaga!** Pamiętaj, że po zbudowaniu należy również wykonać `source devel/setup.bash` w workspace ROS!

## Wskazówki i przydatne linki
-	Zachęcamy do zapoznania się z poradnikiem przedstawiającym podstawy pracy w ROS: www.youtube.com/watch?v=wfDJAYTMTdk&ab_channel=RoboticsBack-End
-	Oficjalny tutorial ROS znajdziesz pod linkiem: http://wiki.ros.org/ROS/Tutorials
-	Do instalacji ROS można wykorzystać instrukcję (należy wybrać wersję desktop-full install): http://wiki.ros.org/noetic/Installation/Ubuntu 
- Do ręcznego wysłania danych na topic w ROS możesz użyć komendy `rostopic pub <nazwa_topicu> <typ_danych> <dane>` (po wpisaniu nazwy topicu dobrze jest od razu użyć TAB aby powłoka pomogła w wpisywaniu i zajęła się typem danych i formatem). Możesz to wykorzystać do weryfikacji działania node'a którego otrzymałeś oraz swoich node'ów.
- Możesz użyć komendy `rostopic echo <nazwa_topicu>` aby wyświetlić dane wysyłane na określony topic.
- Zadanie rekrutacyjne można oddać niepełne.
- Rozwiązane zadanie należy umieścić w **publicznym** repozytorium (np. GitHub) i przesłać linka do tego repozytorium na mail projekt@scorpio.pwr.edu.pl. Ewentualne pytania lub wątpliwości co do treści zadania można kierować na tego samego maila. Zadania przyjmujemy do 31.10.2024 do końca dnia.

## Przydatne ROSowe komendy CLI: 
- rostopic list - zwraca liste wszystkich dostępnych topiców
- rostopic echo <nazwa_topicu> - zwraca dane publikowane na topicu
- rostopic info <nazwa_topicu> - zwraca informacje o topicu
- rostopic pub <nazwa_topicu> <typ_message'a> <zawartość_message'a> - publikuje dane na topic
- rostopic hz <nazwa_topicu> - zwraca częstotliwość publikacji na topic
- rosservice list - zwraca liste wszystkich dostępnych serwisów
- rosservice info <nazwa_serwisu> - zwraca informacje o serwisie
- rosservice call <nazwa_serwisu> <typ_serwisu> <zawartość_requestu> - wywołuje serwis i zwraca odpowiedź wywołania
- rosnode info <nazwa node'a> - zwraca informacje o node'dzie

TODO: dodać przykładowe komendy zadawania celu, publikowanie mapy o customowej wiadomośći i kilka innych, zrobić "tutorial" korzystania z customowych msg i serwisów

**Jeżeli będziesz miał jakiekolwiek wątpliwości i problemy z zadaniem śmiało skontaktuj się z nami na maila projekt@scorpio.pwr.edu.pl! Powodzenia :)**
